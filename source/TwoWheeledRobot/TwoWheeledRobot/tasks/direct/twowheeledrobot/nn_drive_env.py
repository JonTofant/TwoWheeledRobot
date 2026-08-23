"""Joystick-commanded NN drive task: balance + drive + terrain, sim2real hardened.

Extends PureNNBalanceEnv with:
  - velocity / yaw-rate commands and integrated position/heading references
    (the exact contract the STM32 firmware implements for the joystick),
  - 6-dim actions: 4 CyberGear stance targets + 2 DDSM115 wheel currents,
  - generated terrain (flat / bumps / inclines) with station keeping on slopes,
  - per-episode mass/inertia scaling, platform COM shifts, odometry scale error,
    gyro biases, CyberGear gain randomization, and continuous force noise on top
    of the inherited motor/wheel/IMU randomization and push/payload disturbances.

Observation (20) and action (6) layouts are documented in
pure_nn_components.py::DriveObservationBuilder and STM32_DEPLOYMENT.md.
"""

from __future__ import annotations

import math
import random
from collections.abc import Sequence

import torch

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.sensors import ContactSensor, Imu
from isaaclab.terrains import TerrainImporterCfg

from .nn_drive_env_cfg import NNDriveEnvCfg
from .pure_nn_balance_env import PureNNBalanceEnv
from .pure_nn_components import (
    CommandGenerator,
    CyberGearStanceProcessor,
    DriveObservationBuilder,
    DriveReward,
    SteadinessTracker,
    pitch_from_projected_gravity,
    roll_from_projected_gravity,
    yaw_from_quat_wxyz,
)
from .sim_params import (
    DDSM115_KT,
    DDSM115_NO_LOAD_SPEED,
    DDSM115_TAU_PEAK,
    GROUND_DYNAMIC_FRICTION,
    GROUND_STATIC_FRICTION,
    R_WHEEL,
)


class NNDriveEnv(PureNNBalanceEnv):
    cfg: NNDriveEnvCfg

    # Class-level defaults so the inherited termination path can read these
    # safely at any point during construction, before _setup_scene has run.
    ground_contact = None
    _contact_wheel_cols = None
    _contact_nonwheel_cols = None

    def __init__(self, cfg: NNDriveEnvCfg, render_mode: str | None = None, **kwargs):
        expected_actions = {"policy": 6, "fixed": 2}
        if cfg.leg_action_mode not in expected_actions:
            raise ValueError(f"Unsupported leg_action_mode: {cfg.leg_action_mode}")
        if cfg.action_space != expected_actions[cfg.leg_action_mode]:
            raise ValueError(
                f"leg_action_mode={cfg.leg_action_mode!r} requires action_space="
                f"{expected_actions[cfg.leg_action_mode]}, got {cfg.action_space}"
            )
        if len(cfg.fixed_leg_stance_rad) != 4:
            raise ValueError("fixed_leg_stance_rad must contain fl, fr, bl, br targets")
        # The inherited NormalizedObservationBuilder validates observation_scale
        # against observation_space; alias it to the drive scale (it is unused
        # by this env — DriveObservationBuilder replaces it).
        cfg.observation_scale = cfg.drive_observation_scale
        super().__init__(cfg, render_mode, **kwargs)

        self._commands = CommandGenerator(cfg, self.num_envs, self.device)
        self._cg_processor = CyberGearStanceProcessor(cfg, self.num_envs, self.device)
        self._drive_obs_builder = DriveObservationBuilder(cfg, self.device)
        self._drive_reward = DriveReward(cfg)

        # Per-episode sensor-model randomization state.
        self._odometry_scale = torch.ones(self.num_envs, device=self.device)
        self._pitch_rate_bias = torch.zeros(self.num_envs, device=self.device)
        self._yaw_rate_bias = torch.zeros(self.num_envs, device=self.device)
        # Roll equivalents of the pitch mounting/gyro bias (_pitch_bias lives on
        # PureNNBalanceEnv; roll is only observed by this task, so both live here).
        self._roll_bias = torch.zeros(self.num_envs, device=self.device)
        self._roll_rate_bias = torch.zeros(self.num_envs, device=self.device)

        # Continuous force-noise state (filtered white noise on the platform).
        self._force_noise = torch.zeros(self.num_envs, 2, device=self.device)
        # High-pass references for the station-keeping steadiness terms. Two
        # trackers rather than one 3-wide tracker: pitch and the wheel actions
        # are unrelated quantities that happen to share a time constant, and a
        # single buffer would invite reusing one's residual for the other.
        self._pitch_ac_tracker = SteadinessTracker(self.num_envs, 1, self.device, self.cfg.hold_ac_tau_s)
        self._action_ac_tracker = SteadinessTracker(self.num_envs, 2, self.device, self.cfg.hold_ac_tau_s)
        self._force_noise_amp = torch.zeros(self.num_envs, device=self.device)

        # Physical-property randomization baselines (CPU tensors, PhysX API).
        self._default_body_masses = self.robot.root_physx_view.get_masses().clone()
        self._default_body_inertias = self.robot.root_physx_view.get_inertias().clone()
        self._default_body_coms = self.robot.root_physx_view.get_coms().clone()
        self._platform_body_col = int(self._body_ids[0].item()) if self._body_ids is not None else 0
        self._resolve_contact_bodies()

        print(
            "[NNDriveEnv] joystick drive controller active, "
            f"dt={self.step_dt:.3f}s, obs={cfg.observation_space}, act={cfg.action_space}, "
            f"terrain={cfg.terrain_mode}, curriculum_stage={cfg.curriculum_stage}, "
            f"leg_actions={cfg.leg_action_mode}, "
            f"v_max={self._commands._stage_limits(cfg.curriculum_stage)[0]:.2f} m/s, "
            f"w_max={self._commands._stage_limits(cfg.curriculum_stage)[1]:.2f} rad/s"
        )

    def _resolve_contact_bodies(self) -> None:
        """Split the contact sensor's bodies into wheels and everything else.

        ``net_forces_w`` is indexed by the *sensor's* body ordering, which is not
        the articulation's, so the split is done against ``sensor.body_names``.
        Failing loudly here is deliberate: a silently empty non-wheel set would
        mean nothing ever terminates and every training curve would look great.
        """
        if self.ground_contact is None:
            return
        names = list(self.ground_contact.body_names)
        wheel_cols = [i for i, name in enumerate(names) if "DDSM115" in name]
        nonwheel_cols = [i for i, name in enumerate(names) if "DDSM115" not in name]
        if len(wheel_cols) != 2 or not nonwheel_cols:
            raise RuntimeError(
                "Contact sensor body split failed: expected exactly 2 DDSM115 wheel bodies "
                f"and at least one non-wheel body, got wheels={len(wheel_cols)}, "
                f"non-wheels={len(nonwheel_cols)}, bodies={names}"
            )
        self._contact_wheel_cols = torch.tensor(wheel_cols, device=self.device, dtype=torch.long)
        self._contact_nonwheel_cols = torch.tensor(nonwheel_cols, device=self.device, dtype=torch.long)
        print(
            f"[NNDriveEnv] fall_mode=contact, threshold={self.cfg.contact_force_threshold_n:.2f} N, "
            f"wheels={[names[i] for i in wheel_cols]}, "
            f"non-wheel bodies ({len(nonwheel_cols)})={[names[i] for i in nonwheel_cols]}"
        )

    # ── Scene: terrain instead of flat ground plane ──────────────────────────

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        self.scene.articulations["robot"] = self.robot
        self.bno080 = Imu(self.cfg.bno080)
        self.scene.sensors["bno080"] = self.bno080

        # Contact-based falling. The sensor spans every body; the non-wheel subset
        # is selected after construction (see _resolve_contact_bodies), so the
        # wheel channels stay available as the "is contact reporting actually on?"
        # check. NOTE: this is inert unless robot_cfg has
        # activate_contact_sensors=True -- it reports zeros rather than failing.
        self.ground_contact = None
        if self.cfg.fall_mode == "contact":
            self.ground_contact = ContactSensor(self.cfg.ground_contact)
            self.scene.sensors["ground_contact"] = self.ground_contact
        elif self.cfg.fall_mode != "tilt":
            raise ValueError(f"Unsupported fall_mode: {self.cfg.fall_mode!r} (expected 'tilt' or 'contact')")

        ground_static = GROUND_STATIC_FRICTION
        ground_dynamic = GROUND_DYNAMIC_FRICTION
        ground_mode = getattr(self.cfg, "ground_friction_randomization_mode", "inactive")
        if ground_mode == "per_run":
            ground_static = random.uniform(*self.cfg.ground_static_friction_range)
            ground_dynamic = random.uniform(*self.cfg.ground_dynamic_friction_range)
        self._ground_friction_randomization_mode = ground_mode
        self._ground_static_friction = ground_static
        self._ground_dynamic_friction = ground_dynamic

        if self.cfg.terrain_mode == "flat":
            terrain_cfg = TerrainImporterCfg(
                prim_path="/World/ground",
                terrain_type="plane",
                collision_group=-1,
                physics_material=self.cfg.terrain.physics_material,
                debug_vis=False,
            )
        elif self.cfg.terrain_mode == "generator":
            terrain_cfg = self.cfg.terrain
        else:
            raise ValueError(f"Unsupported terrain_mode: {self.cfg.terrain_mode}")
        terrain_cfg.physics_material.static_friction = ground_static
        terrain_cfg.physics_material.dynamic_friction = ground_dynamic
        terrain_cfg.num_envs = self.scene.cfg.num_envs
        terrain_cfg.env_spacing = self.scene.cfg.env_spacing
        self._terrain = terrain_cfg.class_type(terrain_cfg)

        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[terrain_cfg.prim_path])

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
        light_cfg.func("/World/Light", light_cfg)
        print(
            f"[NNDriveEnv] terrain_mode={self.cfg.terrain_mode}, "
            f"ground friction static/dynamic={ground_static:.3f}/{ground_dynamic:.3f} ({ground_mode})"
        )

    def _print_randomization_summary(self) -> None:
        super()._print_randomization_summary()
        print("NN DRIVE EXTRA RANDOMIZATION")
        print(f"body mass/inertia scale: active, range {self.cfg.body_mass_scale_range}")
        print(
            "platform COM offset: active, y (fore/aft) "
            f"{self.cfg.com_offset_y_range_m} m, z {self.cfg.com_offset_z_range_m} m"
        )
        print(f"odometry scale (obs): active, range {self.cfg.odometry_scale_range}")
        print(f"pitch-rate gyro bias: active, range {self.cfg.pitch_rate_bias_radps_range} rad/s")
        print(f"yaw-rate gyro bias: active, range {self.cfg.yaw_rate_bias_radps_range} rad/s")
        print(f"cybergear kp/kd: active, ranges {self.cfg.cg_kp_range} / {self.cfg.cg_kd_range}")
        print(f"cybergear calibration bias: active, range {self.cfg.cg_calib_bias_rad_range} rad")
        noise_state = "active" if self.cfg.enable_force_noise else "inactive"
        print(f"force noise: {noise_state}, amp {self.cfg.force_noise_amp_n_range} N")

    # ── State terms against the moving command references ────────────────────

    def _state_terms(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        raw_wheel_pos = self.robot.data.joint_pos[:, self._wheel_ids] * self._wheel_sign
        raw_wheel_vel = self.robot.data.joint_vel[:, self._wheel_ids] * self._wheel_sign
        x_rel = 0.5 * raw_wheel_pos.sum(dim=1) * R_WHEEL
        velocity = 0.5 * raw_wheel_vel.sum(dim=1) * R_WHEEL
        pitch = pitch_from_projected_gravity(self.bno080.data.projected_gravity_b)
        pitch_rate = -self.bno080.data.ang_vel_b[:, 0]
        # Kept as a single wrapped scalar for callers that just want "how far
        # off," reconstructed from the same sin/cos pair _get_observations()/
        # _get_rewards() use directly (see CommandGenerator.yaw_error_sin_cos).
        yaw_err_sin, yaw_err_cos = self._commands.yaw_error_sin_cos(yaw_from_quat_wxyz(self.robot.data.root_quat_w))
        yaw_error = torch.atan2(yaw_err_sin, yaw_err_cos)
        yaw_rate = self.robot.data.root_ang_vel_w[:, 2]
        return x_rel, velocity, pitch, pitch_rate, yaw_error, yaw_rate

    # ── Actions: policy legs + wheels, or fixed legs + wheels for the A/B ────

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._enforce_cybergear_joint_state_limits()
        self._prev_actions = self._cur_actions.clone()
        self._cur_actions = actions.clone()

        # Feed measured odometry/heading in so the references cannot outrun the
        # robot (anti-windup); the firmware does the same with its own odometry.
        raw_wheel_pos = self.robot.data.joint_pos[:, self._wheel_ids] * self._wheel_sign
        x_odom_now = 0.5 * raw_wheel_pos.sum(dim=1) * R_WHEEL
        yaw_now = yaw_from_quat_wxyz(self.robot.data.root_quat_w)
        self._commands.step(self.episode_length_buf, self.cfg.curriculum_stage, self.step_dt, x_odom_now, yaw_now)

        if self.cfg.leg_action_mode == "fixed":
            cg_targets = self._cg_processor.process_fixed(
                self.cfg.fixed_leg_stance_rad, self._cg_joint_lo, self._cg_joint_hi, self.step_dt
            )
            wheel_actions = actions[:, 0:2]
        else:
            cg_targets = self._cg_processor.process(actions[:, 0:4], self._cg_joint_lo, self._cg_joint_hi, self.step_dt)
            wheel_actions = actions[:, 4:6]
        self.robot.set_joint_position_target(cg_targets, joint_ids=self._cg_ids)

        # Wheel current path — identical motor model to PureNNBalanceEnv.
        # Wheel speed BEFORE this step's torque is applied -- the friction the
        # command must overcome is the one at the current state.
        self._wheel_i_cmd = self._action_processor.process(
            wheel_actions, self.step_dt, self.robot.data.joint_vel[:, self._wheel_ids]
        )
        self._wheel_i_des = self._action_processor.net_current.clone()
        self._wheel_tau_current = self._wheel_i_cmd * DDSM115_KT
        self._wheel_velocity_raw = self.robot.data.joint_vel[:, self._wheel_ids].clone()
        self._wheel_velocity_used = self._wheel_velocity_raw.clone()
        # Back-EMF only derates torque that does positive work against rotation
        # (motoring). Torque opposing rotation (braking — exactly what a
        # recovery controller needs near top speed) is current/thermal limited,
        # not back-EMF limited, so it must not be derated by |omega| here.
        same_sign = (self._wheel_tau_current * self._wheel_velocity_used) >= 0.0
        self._wheel_omega_for_limiter = torch.where(
            same_sign, self._wheel_velocity_used.abs(), torch.zeros_like(self._wheel_velocity_used)
        )
        self._wheel_tau_speed_limit = DDSM115_TAU_PEAK * (1.0 - self._wheel_omega_for_limiter / DDSM115_NO_LOAD_SPEED)
        self._wheel_tau_speed_limit = self._wheel_tau_speed_limit.clamp(0.0, DDSM115_TAU_PEAK)
        self._wheel_torque_cmd = torch.maximum(
            -self._wheel_tau_speed_limit,
            torch.minimum(self._wheel_tau_current, self._wheel_tau_speed_limit),
        )
        self._efforts_buf[:, 0] = -self._wheel_torque_cmd[:, 0]
        self._efforts_buf[:, 1] = self._wheel_torque_cmd[:, 1]
        self.robot.set_joint_effort_target(self._efforts_buf, joint_ids=self._wheel_ids)

        # Discrete disturbances (pushes/payload) + continuous force noise.
        if self._body_ids is not None:
            t = self.episode_length_buf.float() * self.step_dt
            force, torque = self._disturbance.force_and_torque(self.episode_length_buf, t)
            if self.cfg.enable_force_noise:
                alpha = math.exp(-self.step_dt / max(self.cfg.force_noise_tau_s, self.step_dt))
                self._force_noise = alpha * self._force_noise + (1.0 - alpha) * (
                    torch.randn(self.num_envs, 2, device=self.device) * self._force_noise_amp.unsqueeze(1)
                )
                force = force.clone()
                force[:, 0] += self._force_noise[:, 0]
                force[:, 1] += self._force_noise[:, 1]
            self._last_disturbance_force = force
            self._last_disturbance_torque = torque
            self._body_force[:, 0, :] = force
            self._body_torque[:, 0, :] = torque
            try:
                self.robot.set_external_force_and_torque(self._body_force, self._body_torque, body_ids=self._body_ids)
            except Exception:
                self._last_disturbance_force.zero_()
                self._last_disturbance_torque.zero_()

    # ── Observations ─────────────────────────────────────────────────────────

    def _get_observations(self) -> dict:
        self._enforce_cybergear_joint_state_limits()
        x_rel, velocity, pitch, pitch_rate, yaw_error, yaw_rate = self._state_terms()
        roll = roll_from_projected_gravity(self.bno080.data.projected_gravity_b)
        roll_rate = self.bno080.data.ang_vel_b[:, 1]

        # Sensor models: mounting bias, gyro biases, odometry scale, noise.
        pitch_m = pitch + self._pitch_bias + torch.randn_like(pitch) * self.cfg.pitch_noise_std
        pitch_rate_m = pitch_rate + self._pitch_rate_bias + torch.randn_like(pitch_rate) * self.cfg.pitch_rate_noise_std
        # Roll comes off the same BNO080 as pitch, so it carries a mounting bias
        # and gyro bias of the same magnitude — sampled separately because the
        # two axes of one mount are independent errors, not a shared one.
        roll_m = roll + self._roll_bias + torch.randn_like(roll) * self.cfg.pitch_noise_std
        roll_rate_m = roll_rate + self._roll_rate_bias + torch.randn_like(roll_rate) * self.cfg.pitch_rate_noise_std
        yaw_rate_m = yaw_rate + self._yaw_rate_bias + torch.randn_like(yaw_rate) * self.cfg.yaw_rate_noise_std
        velocity_m = velocity * self._odometry_scale + torch.randn_like(velocity) * self.cfg.velocity_noise_std
        pos_err_m = self._commands.position_error(x_rel * self._odometry_scale)

        cg_pos = self.robot.data.joint_pos[:, self._cg_ids]
        cg_pos = cg_pos + torch.randn_like(cg_pos) * self.cfg.noise_cg_pos_std
        cg_pos_norm = cg_pos / self.cfg.cg_position_scale_rad

        # No simulated noise/bias on the underlying yaw (matching a fused-IMU
        # heading, not raw gyro integration — see DriveObservationBuilder's
        # docstring). Sin/cos of the raw, unwrapped difference: bounded and
        # smooth for any error magnitude, no anti-windup needed on yaw_ref.
        yaw_err_sin, yaw_err_cos = self._commands.yaw_error_sin_cos(yaw_from_quat_wxyz(self.robot.data.root_quat_w))

        self._obs_now = self._drive_obs_builder.build(
            pos_err_m,
            velocity_m,
            pitch_m,
            pitch_rate_m,
            yaw_err_sin,
            yaw_err_cos,
            yaw_rate_m,
            self._commands.v_cmd,
            self._commands.w_cmd,
            cg_pos_norm,
            self._action_processor.command_current.clone(),
            self._cg_processor.tanh_action.clone(),
            roll_m,
            roll_rate_m,
        )
        obs = torch.where(self._obs_delay_samples.view(-1, 1) > 0, self._obs_delay, self._obs_now)
        self._obs_delay = self._obs_now.clone()
        return {"policy": obs}

    # ── Rewards ──────────────────────────────────────────────────────────────

    def _get_rewards(self) -> torch.Tensor:
        x_rel, velocity, pitch, pitch_rate, yaw_error, yaw_rate = self._state_terms()
        roll = roll_from_projected_gravity(self.bno080.data.projected_gravity_b)
        roll_rate = self.bno080.data.ang_vel_b[:, 1]
        self._update_termination_flags(pitch, pitch_rate, velocity, yaw_error, yaw_rate)
        # Reward sees the unclamped drift so the position bonus keeps pulling home
        # past the clamp; the observation stays clamped for firmware parity.
        pos_err_raw = self._commands.position_error_raw(x_rel)
        pos_err = pos_err_raw.clamp(-self.cfg.cmd_pos_err_clamp_m, self.cfg.cmd_pos_err_clamp_m)
        # Ground truth, immune to the reference anti-windup absorbing drift
        # (see rew_hold_world_drift). Never fed to the observation -- a real
        # robot has no ground-truth world position either.
        world_drift = torch.linalg.vector_norm(
            self.robot.data.root_pos_w[:, :2] - self._spawn_pos_xy, dim=1
        )
        # yaw_err_cos alone is enough for the 1-cos(error) reward shape (an
        # even function of the error -- sign doesn't matter for "how far off").
        _, yaw_err_cos = self._commands.yaw_error_sin_cos(yaw_from_quat_wxyz(self.robot.data.root_quat_w))
        # Advance the high-pass references once per control step, here rather
        # than in _pre_physics_step, so the EMA state is updated exactly where
        # its residual is consumed. Pitch is the TRUE pitch the reward already
        # reads -- the residual removes the DC mounting bias by construction,
        # so there is nothing for the biased observation copy to contribute.
        pitch_ac = self._pitch_ac_tracker.update(pitch.unsqueeze(1), self.step_dt).squeeze(1)
        # net_current is tanh(action) * i_max_a, i.e. the policy's own commanded
        # intent before this episode's motor gain/deadzone/bias draw. Dividing
        # it back out gives tanh units. Deliberately NOT command_current: that
        # carries the randomized motor response, and charging the policy for
        # smoothness it did not cause would make the term a lottery on the draw.
        action_ac = self._action_ac_tracker.update(
            self._wheel_i_des / self.cfg.i_max_a, self.step_dt
        )
        reward, components = self._drive_reward.compute(
            pos_err_raw,
            velocity,
            pitch,
            pitch_rate,
            roll,
            roll_rate,
            yaw_err_cos,
            yaw_rate,
            self._commands.v_cmd,
            self._commands.w_cmd,
            self._action_processor.command_current,
            self._action_processor.delta_current(),
            self._cg_processor.target_angle,
            self._cg_processor.delta_target_angle(),
            world_drift,
            pitch_ac,
            action_ac,
            self._last_terminal_penalty,
        )
        # Per-env components kept for scripts/verify_contact_and_reward.py, which
        # asserts the per-step reward stays non-negative. extras["log"] only keeps
        # means, and a mean cannot show a single env going negative.
        self._last_reward_components = components
        hold_mask = (self._commands.v_cmd.abs() < self.cfg.hold_velocity_cmd_threshold_mps) & (
            self._commands.w_cmd.abs() < self.cfg.hold_yaw_rate_cmd_threshold_radps
        )
        self._episode_reward += reward
        self.extras["log"] = {
            "reward": reward.mean(),
            "episode_reward": self._episode_reward.mean(),
            **{f"reward_{name}": value.mean() for name, value in components.items()},
            "pitch_abs_deg": pitch.abs().mean() * 180.0 / math.pi,
            "roll_abs_deg": roll.abs().mean() * 180.0 / math.pi,
            "total_tilt_abs_deg": self._last_total_tilt.mean() * 180.0 / math.pi,
            "vel_err_abs": (velocity - self._commands.v_cmd).abs().mean(),
            "yaw_rate_err_abs": (yaw_rate - self._commands.w_cmd).abs().mean(),
            "pos_err_abs": pos_err.abs().mean(),
            # Unclamped drift and the stationary-only subset: these two are the
            # numbers that track the hardware "drives away" complaint.
            "pos_err_raw_abs": pos_err_raw.abs().mean(),
            "hold_pos_err_abs": self._masked_mean(pos_err_raw.abs(), hold_mask),
            "hold_velocity_abs": self._masked_mean(velocity.abs(), hold_mask),
            # The quantity this experiment exists to reduce. Reward components
            # move for many reasons (a weight change, a different hold fraction);
            # this is the behaviour itself, in the same degrees the hardware
            # capture reports, so a run can be compared straight against the
            # 1.89 deg the deployed GRU holds. Hold-masked: the AC residual is
            # meaningless while a velocity command is legitimately swinging pitch.
            "hold_pitch_ac_deg": self._masked_mean(pitch_ac.abs(), hold_mask) * 180.0 / math.pi,
            "hold_action_ac_abs": self._masked_mean(action_ac.abs().mean(dim=1), hold_mask),
            "yaw_error_abs": yaw_error.abs().mean(),
            "v_cmd_abs": self._commands.v_cmd.abs().mean(),
            "w_cmd_abs": self._commands.w_cmd.abs().mean(),
            "current_rms": torch.sqrt(self._action_processor.command_current.pow(2).mean()),
            "cg_action_abs": self._cg_processor.tanh_action.abs().mean(),
            "fall_rate": self._last_fall.float().mean(),
            "termination_timeout": self._last_timeout.float().mean(),
            "termination_fall": self._last_fall.float().mean(),
            "termination_physics_broken": self._last_physics_broken.float().mean(),
            "termination_invalid_state": self._last_invalid_state.float().mean(),
            "disturbance_force_n": self._last_disturbance_force.norm(dim=1).mean(),
            "disturbance_torque_nm": self._last_disturbance_torque.norm(dim=1).mean(),
        }
        return reward

    @staticmethod
    def _masked_mean(values: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
        """Mean of ``values`` over ``mask``; zero when no env matches this step."""
        count = mask.sum()
        if count == 0:
            return torch.zeros((), device=values.device)
        return (values * mask.float()).sum() / count

    # ── Reset ────────────────────────────────────────────────────────────────

    def _reset_idx(self, env_ids: Sequence[int] | None):
        super()._reset_idx(env_ids)
        if not hasattr(self, "_commands"):
            return
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        env_ids_t = (
            env_ids
            if isinstance(env_ids, torch.Tensor)
            else torch.tensor(env_ids, device=self.device, dtype=torch.long)
        )
        n = len(env_ids_t)

        # Re-place the robot on its terrain origin with a random heading. The
        # parent reset already placed it on the (flat) scene grid; terrain
        # origins differ, so this write wins.
        pitch_range, pitch_rate_range, velocity_range = self._curriculum.reset_ranges()
        pitch = torch.empty(n, device=self.device).uniform_(-pitch_range, pitch_range)
        pitch_rate = torch.empty(n, device=self.device).uniform_(-pitch_rate_range, pitch_rate_range)
        velocity = torch.empty(n, device=self.device).uniform_(-velocity_range, velocity_range)
        wheel_omega = velocity / R_WHEEL

        if self.cfg.reset_yaw_random:
            yaw = torch.empty(n, device=self.device).uniform_(-math.pi, math.pi)
        else:
            yaw = torch.zeros(n, device=self.device)
        zeros = torch.zeros(n, device=self.device)
        quat_pitch = torch.stack([torch.cos(0.5 * pitch), -torch.sin(0.5 * pitch), zeros, zeros], dim=1)
        quat_yaw = torch.stack([torch.cos(0.5 * yaw), zeros, zeros, torch.sin(0.5 * yaw)], dim=1)
        quat = math_utils.quat_mul(quat_yaw, quat_pitch)

        terrain_origins = self._terrain.env_origins[env_ids_t]
        root_state = self.robot.data.default_root_state[env_ids_t].clone()
        root_state[:, :2] = terrain_origins[:, :2]
        root_state[:, 2] = terrain_origins[:, 2] + self.cfg.spawn_upright_z + self.cfg.spawn_extra_clearance_m
        root_state[:, 3:7] = quat
        root_state[:, 7:] = 0.0
        root_state[:, 10] = pitch_rate
        self.robot.write_root_pose_to_sim(root_state[:, :7], env_ids_t)
        self.robot.write_root_velocity_to_sim(root_state[:, 7:], env_ids_t)
        self._spawn_pos_xy[env_ids_t] = root_state[:, :2]
        self._yaw_reference[env_ids_t] = yaw
        self._physics_broken_z[env_ids_t] = root_state[:, 2] - 1.0

        joint_pos = self.robot.data.default_joint_pos[env_ids_t].clone()
        joint_vel = self.robot.data.default_joint_vel[env_ids_t].clone()
        joint_pos[:, self._cg_ids] = 0.0
        joint_vel[:, self._cg_ids] = 0.0
        joint_vel[:, self._wheel_ids] = wheel_omega.unsqueeze(1) * self._wheel_sign
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids_t)
        self.robot.set_joint_position_target(joint_pos, env_ids=env_ids_t)

        # Drive-specific per-episode randomization.
        self._commands.reset(env_ids_t, self.cfg.curriculum_stage, self.step_dt, yaw)
        self._cg_processor.reset(env_ids_t)
        self._odometry_scale[env_ids_t] = torch.empty(n, device=self.device).uniform_(*self.cfg.odometry_scale_range)
        self._pitch_rate_bias[env_ids_t] = torch.empty(n, device=self.device).uniform_(
            *self.cfg.pitch_rate_bias_radps_range
        )
        self._yaw_rate_bias[env_ids_t] = torch.empty(n, device=self.device).uniform_(
            *self.cfg.yaw_rate_bias_radps_range
        )
        self._roll_bias[env_ids_t] = torch.empty(n, device=self.device).uniform_(*self.cfg.roll_bias_rad_range)
        self._roll_rate_bias[env_ids_t] = torch.empty(n, device=self.device).uniform_(
            *self.cfg.roll_rate_bias_radps_range
        )
        self._force_noise[env_ids_t] = 0.0
        # Reset (not zero) the steadiness references: SteadinessTracker re-seeds
        # from the first post-reset sample, so a fresh episode's spawn attitude
        # is not read as a step change away from the previous episode's mean.
        self._pitch_ac_tracker.reset(env_ids_t)
        self._action_ac_tracker.reset(env_ids_t)
        self._force_noise_amp[env_ids_t] = torch.empty(n, device=self.device).uniform_(
            *self.cfg.force_noise_amp_n_range
        )
        self._randomize_cybergear_gains(env_ids_t)
        self._randomize_body_properties(env_ids_t)

    def _randomize_cybergear_gains(self, env_ids_t: torch.Tensor) -> None:
        env_ids_cpu = env_ids_t.detach().cpu()
        n = len(env_ids_t)
        cg_cols = [self._cg_fl_ids[0], self._cg_fr_ids[0], self._cg_bl_ids[0], self._cg_br_ids[0]]
        stiffness = self._default_joint_stiffness[env_ids_cpu].clone().to(self.device)
        damping = self._default_joint_damping[env_ids_cpu].clone().to(self.device)
        stiffness[:, cg_cols] = torch.empty(n, 4, device=self.device).uniform_(*self.cfg.cg_kp_range)
        damping[:, cg_cols] = torch.empty(n, 4, device=self.device).uniform_(*self.cfg.cg_kd_range)
        # `damping` was re-cloned from the static per-joint default above, which
        # would otherwise silently overwrite the wheel columns' per-episode
        # viscous-damping randomization already written by
        # PureNNBalanceEnv._apply_pure_nn_physical_randomization earlier this
        # reset. Splice the already-sampled wheel damping back in before writing.
        if self.cfg.wheel_viscous_damping_randomization_active:
            wheel_cols = [self._left_wheel_ids[0], self._right_wheel_ids[0]]
            damping[:, wheel_cols] = self._sampled_wheel_viscous_damping[env_ids_t]
        self.robot.write_joint_stiffness_to_sim(stiffness, env_ids=env_ids_cpu)
        self.robot.write_joint_damping_to_sim(damping, env_ids=env_ids_cpu)

    def _randomize_body_properties(self, env_ids_t: torch.Tensor) -> None:
        """Per-episode mass/inertia scale and platform COM shift (PhysX, CPU)."""
        env_ids_cpu = env_ids_t.detach().cpu()
        n = len(env_ids_cpu)
        scale = torch.empty(n, 1, dtype=self._default_body_masses.dtype).uniform_(*self.cfg.body_mass_scale_range)

        masses = self.robot.root_physx_view.get_masses().clone()
        masses[env_ids_cpu] = self._default_body_masses[env_ids_cpu] * scale
        self.robot.root_physx_view.set_masses(masses, env_ids_cpu)

        inertias = self.robot.root_physx_view.get_inertias().clone()
        inertias[env_ids_cpu] = self._default_body_inertias[env_ids_cpu] * scale.unsqueeze(-1)
        self.robot.root_physx_view.set_inertias(inertias, env_ids_cpu)

        # Column 1 is Y = fore/aft, the axis the pitch balance point lives on.
        # This was column 0 (X = lateral) until 2026-08-05; see the
        # com_offset_y_range_m comment in the cfg for what that cost.
        coms = self.robot.root_physx_view.get_coms().clone()
        com_dy = torch.empty(n, dtype=coms.dtype).uniform_(*self.cfg.com_offset_y_range_m)
        com_dz = torch.empty(n, dtype=coms.dtype).uniform_(*self.cfg.com_offset_z_range_m)
        coms[env_ids_cpu] = self._default_body_coms[env_ids_cpu]
        coms[env_ids_cpu, self._platform_body_col, 1] += com_dy
        coms[env_ids_cpu, self._platform_body_col, 2] += com_dz
        self.robot.root_physx_view.set_coms(coms, env_ids_cpu)
