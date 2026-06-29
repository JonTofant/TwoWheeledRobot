"""Pure neural-network balance task with direct left/right current commands."""

from __future__ import annotations

import math
from collections.abc import Sequence

import torch

from .pure_nn_balance_env_cfg import PureNNBalanceEnvCfg
from .pure_nn_components import (
    BalanceReward,
    CurrentActionProcessor,
    CurriculumSampler,
    DIST_DOUBLE_HUMAN_PUSH,
    DIST_HUMAN_PUSH,
    DIST_NONE,
    DIST_PAYLOAD,
    DIST_PAYLOAD_PUSH,
    DIST_SINE_DIAGNOSTIC,
    DIST_SLOPE,
    DisturbanceGenerator,
    NormalizedObservationBuilder,
    pitch_from_projected_gravity,
    wrap_angle_rad,
    yaw_from_quat_wxyz,
)
from .residual_lqr_env import R_WHEEL
from .sim_params import DDSM115_KT, DDSM115_NO_LOAD_SPEED, DDSM115_TAU_PEAK
from .standup_env import StandupEnv


class PureNNBalanceEnv(StandupEnv):
    cfg: PureNNBalanceEnvCfg

    def __init__(self, cfg: PureNNBalanceEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._wheel_sign = torch.tensor([-1.0, 1.0], device=self.device, dtype=torch.float32)
        self._yaw_reference = torch.zeros(self.num_envs, device=self.device)
        self._pitch_bias = torch.zeros(self.num_envs, device=self.device)
        self._obs_delay_samples = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self._obs_now = torch.zeros(self.num_envs, self.cfg.observation_space, device=self.device)
        self._obs_delay = torch.zeros_like(self._obs_now)
        self._episode_reward = torch.zeros(self.num_envs, device=self.device)
        self._last_disturbance_force = torch.zeros(self.num_envs, 3, device=self.device)
        self._last_disturbance_torque = torch.zeros(self.num_envs, 3, device=self.device)
        self._body_force = torch.zeros(self.num_envs, 1, 3, device=self.device)
        self._body_torque = torch.zeros_like(self._body_force)
        self._body_ids = self._resolve_push_body_ids()

        self._obs_builder = NormalizedObservationBuilder(cfg, self.device)
        self._action_processor = CurrentActionProcessor(cfg, self.num_envs, self.device)
        self._curriculum = CurriculumSampler(cfg, self.device)
        self._disturbance = DisturbanceGenerator(cfg, self.num_envs, self.device)
        self._reward = BalanceReward(cfg)
        print(
            "[PureNNBalanceEnv] pure NN current controller active, "
            f"dt={self.step_dt:.3f}s, obs={cfg.observation_space}, act={cfg.action_space}, "
            f"I_max={cfg.i_max_a:.2f}A, curriculum_stage={cfg.curriculum_stage}"
        )

    def _resolve_push_body_ids(self) -> torch.Tensor | None:
        for pattern in (".*Platform.*", ".*MainAssembly.*", ".*"):
            try:
                body_ids, _ = self.robot.find_bodies(pattern, preserve_order=True)
                if body_ids:
                    return torch.tensor([body_ids[0]], device=self.device, dtype=torch.long)
            except Exception:
                continue
        return None

    def _state_terms(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        raw_wheel_pos = self.robot.data.joint_pos[:, self._wheel_ids] * self._wheel_sign
        raw_wheel_vel = self.robot.data.joint_vel[:, self._wheel_ids] * self._wheel_sign
        x_rel = 0.5 * raw_wheel_pos.sum(dim=1) * R_WHEEL
        velocity = 0.5 * raw_wheel_vel.sum(dim=1) * R_WHEEL
        pitch = pitch_from_projected_gravity(self.bno080.data.projected_gravity_b)
        pitch_rate = -self.bno080.data.ang_vel_b[:, 0]
        yaw_error = wrap_angle_rad(yaw_from_quat_wxyz(self.robot.data.root_quat_w) - self._yaw_reference)
        yaw_rate = self.robot.data.root_ang_vel_w[:, 2]
        return x_rel, velocity, pitch, pitch_rate, yaw_error, yaw_rate

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._enforce_cybergear_joint_state_limits()
        self._prev_actions = self._cur_actions.clone()
        self._cur_actions = actions[:, :2].clone()

        zero_cg_targets = torch.zeros(self.num_envs, 4, device=self.device)
        zero_cg_targets = torch.max(self._cg_joint_lo, torch.min(self._cg_joint_hi, zero_cg_targets))
        self.robot.set_joint_position_target(zero_cg_targets, joint_ids=self._cg_ids)

        self._wheel_i_cmd = self._action_processor.process(actions, self.step_dt)
        self._wheel_i_des = self._action_processor.net_current.clone()
        self._wheel_tau_current = self._wheel_i_cmd * DDSM115_KT
        self._wheel_velocity_raw = self.robot.data.joint_vel[:, self._wheel_ids].clone()
        self._wheel_velocity_used = self._wheel_velocity_raw.clone()
        self._wheel_omega_for_limiter = self._wheel_velocity_used.abs()
        self._wheel_tau_speed_limit = DDSM115_TAU_PEAK * (1.0 - self._wheel_omega_for_limiter / DDSM115_NO_LOAD_SPEED)
        self._wheel_tau_speed_limit = self._wheel_tau_speed_limit.clamp(0.0, DDSM115_TAU_PEAK)
        self._wheel_torque_cmd = torch.maximum(
            -self._wheel_tau_speed_limit,
            torch.minimum(self._wheel_tau_current, self._wheel_tau_speed_limit),
        )
        self._efforts_buf[:, 0] = -self._wheel_torque_cmd[:, 0]
        self._efforts_buf[:, 1] = self._wheel_torque_cmd[:, 1]
        self.robot.set_joint_effort_target(self._efforts_buf, joint_ids=self._wheel_ids)

        # External disturbance convention for the selected platform/body:
        # force X = forward/back human push or slope-equivalent bias, force Y =
        # small lateral push, torque X = pitch-axis payload/COM-shift fallback,
        # torque Z = yaw moment from an off-center human push. Components are
        # passed to Isaac Lab without frame conversion and kept consistent with
        # the robot's upright spawn axes.
        if self._body_ids is not None:
            t = self.episode_length_buf.float() * self.step_dt
            force, torque = self._disturbance.force_and_torque(self.episode_length_buf, t)
            self._last_disturbance_force = force
            self._last_disturbance_torque = torque
            self._body_force[:, 0, :] = force
            self._body_torque[:, 0, :] = torque
            try:
                self.robot.set_external_force_and_torque(self._body_force, self._body_torque, body_ids=self._body_ids)
            except Exception:
                self._last_disturbance_force.zero_()
                self._last_disturbance_torque.zero_()

    def _get_observations(self) -> dict:
        self._enforce_cybergear_joint_state_limits()
        x_rel, velocity, pitch, pitch_rate, yaw_error, yaw_rate = self._state_terms()
        pitch = pitch + self._pitch_bias + torch.randn_like(pitch) * self.cfg.pitch_noise_std
        pitch_rate = pitch_rate + torch.randn_like(pitch_rate) * self.cfg.pitch_rate_noise_std
        velocity = velocity + torch.randn_like(velocity) * self.cfg.velocity_noise_std
        previous_current = self._action_processor.command_current.clone()
        self._obs_now = self._obs_builder.build(x_rel, velocity, pitch, pitch_rate, yaw_error, yaw_rate, previous_current)
        obs = torch.where(self._obs_delay_samples.view(-1, 1) > 0, self._obs_delay, self._obs_now)
        self._obs_delay = self._obs_now.clone()
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        x_rel, velocity, pitch, pitch_rate, yaw_error, _ = self._state_terms()
        reward = self._reward.compute(
            pitch,
            pitch_rate,
            velocity,
            x_rel,
            yaw_error,
            self._action_processor.command_current,
            self._action_processor.delta_current(),
        )
        self._episode_reward += reward
        fallen = pitch.abs() > math.radians(self.cfg.floor_stop_pitch_deg)
        self.extras["log"] = {
            "reward": reward.mean(),
            "episode_reward": self._episode_reward.mean(),
            "pitch_abs_deg": pitch.abs().mean() * 180.0 / math.pi,
            "pitch_rate_abs": pitch_rate.abs().mean(),
            "position_abs": x_rel.abs().mean(),
            "current_rms": torch.sqrt(self._action_processor.command_current.pow(2).mean()),
            "fall_rate": fallen.float().mean(),
            "disturbance_force_n": self._last_disturbance_force.norm(dim=1).mean(),
            "disturbance_torque_nm": self._last_disturbance_torque.norm(dim=1).mean(),
        }
        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        x_rel, _, pitch, _, _, _ = self._state_terms()
        body_z = torch.nan_to_num(self.robot.data.root_pos_w[:, 2], nan=-999.0)
        fallen = pitch.abs() > math.radians(self.cfg.floor_stop_pitch_deg)
        out_of_bounds = x_rel.abs() > self.cfg.position_stop_m
        physics_broken = body_z < -1.0
        terminated = fallen | out_of_bounds | physics_broken
        timeout = self.episode_length_buf >= self.max_episode_length - 1
        return terminated, timeout

    def _reset_idx(self, env_ids: Sequence[int] | None):
        super()._reset_idx(env_ids)
        if not hasattr(self, "_yaw_reference"):
            return
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        env_ids_t = env_ids if isinstance(env_ids, torch.Tensor) else torch.tensor(env_ids, device=self.device, dtype=torch.long)
        n = len(env_ids_t)

        pitch_range, pitch_rate_range, velocity_range = self._curriculum.reset_ranges()
        pitch = torch.empty(n, device=self.device).uniform_(-pitch_range, pitch_range)
        pitch_rate = torch.empty(n, device=self.device).uniform_(-pitch_rate_range, pitch_rate_range)
        velocity = torch.empty(n, device=self.device).uniform_(-velocity_range, velocity_range)
        wheel_omega = velocity / R_WHEEL

        root_state = self.robot.data.default_root_state[env_ids_t].clone()
        root_state[:, :3] += self.scene.env_origins[env_ids_t]
        root_state[:, 2] = self.scene.env_origins[env_ids_t, 2] + self.cfg.spawn_upright_z
        root_state[:, 3] = torch.cos(0.5 * pitch)
        root_state[:, 4] = -torch.sin(0.5 * pitch)
        root_state[:, 5:7] = 0.0
        root_state[:, 7:] = 0.0
        root_state[:, 10] = pitch_rate
        self.robot.write_root_pose_to_sim(root_state[:, :7], env_ids_t)
        self.robot.write_root_velocity_to_sim(root_state[:, 7:], env_ids_t)
        self._spawn_pos_xy[env_ids_t] = root_state[:, :2]
        self._yaw_reference[env_ids_t] = yaw_from_quat_wxyz(root_state[:, 3:7])

        joint_pos = self.robot.data.default_joint_pos[env_ids_t].clone()
        joint_vel = self.robot.data.default_joint_vel[env_ids_t].clone()
        joint_pos[:, self._cg_ids] = 0.0
        joint_vel[:, self._cg_ids] = 0.0
        joint_vel[:, self._wheel_ids] = wheel_omega.unsqueeze(1) * self._wheel_sign
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids_t)
        self.robot.set_joint_position_target(joint_pos, env_ids=env_ids_t)

        self._prev_actions[env_ids_t] = 0.0
        self._cur_actions[env_ids_t] = 0.0
        self._episode_reward[env_ids_t] = 0.0
        self._last_disturbance_force[env_ids_t] = 0.0
        self._last_disturbance_torque[env_ids_t] = 0.0
        self._body_force[env_ids_t] = 0.0
        self._body_torque[env_ids_t] = 0.0
        self._obs_now[env_ids_t] = 0.0
        self._obs_delay[env_ids_t] = 0.0
        self._obs_delay_samples[env_ids_t] = torch.randint(0, 2, (n,), device=self.device)
        self._pitch_bias[env_ids_t] = torch.empty(n, device=self.device).uniform_(*self.cfg.pitch_bias_rad_range)
        self._action_processor.reset(env_ids_t)
        self._disturbance.reset(env_ids_t, self.cfg.curriculum_stage, self.step_dt)
        forced_kind = {
            "none": DIST_NONE,
            "human_push": DIST_HUMAN_PUSH,
            "single_human_push": DIST_HUMAN_PUSH,
            "double_human_push": DIST_DOUBLE_HUMAN_PUSH,
            "payload": DIST_PAYLOAD,
            "payload_push": DIST_PAYLOAD_PUSH,
            "slope": DIST_SLOPE,
            "sine_diagnostic": DIST_SINE_DIAGNOSTIC,
        }.get(self.cfg.benchmark_disturbance_kind)
        if forced_kind is not None:
            self._disturbance._clear(env_ids_t)
            self._disturbance.kind[env_ids_t] = forced_kind
            if forced_kind == DIST_DOUBLE_HUMAN_PUSH:
                self._disturbance._sample_push(env_ids_t, first=True, dt=self.step_dt)
                self._disturbance._sample_push(env_ids_t, first=False, dt=self.step_dt)
            elif forced_kind == DIST_HUMAN_PUSH:
                self._disturbance._sample_push(env_ids_t, first=True, dt=self.step_dt)
            elif forced_kind == DIST_PAYLOAD:
                self._disturbance._sample_payload(env_ids_t, dt=self.step_dt)
            elif forced_kind == DIST_PAYLOAD_PUSH:
                self._disturbance._sample_payload(env_ids_t, dt=self.step_dt)
                self._disturbance._sample_push(env_ids_t, first=True, dt=self.step_dt)
            elif forced_kind == DIST_SLOPE:
                self._disturbance.slope_force[env_ids_t, 0] = torch.empty(n, device=self.device).uniform_(
                    *self.cfg.slope_fx_n_range
                )
            elif forced_kind == DIST_SINE_DIAGNOSTIC:
                self._disturbance.sine_amp[env_ids_t] = torch.empty(n, device=self.device).uniform_(
                    *self.cfg.sine_force_n_range
                )
                self._disturbance.sine_freq[env_ids_t] = torch.empty(n, device=self.device).uniform_(
                    *self.cfg.sine_frequency_hz_range
                )
