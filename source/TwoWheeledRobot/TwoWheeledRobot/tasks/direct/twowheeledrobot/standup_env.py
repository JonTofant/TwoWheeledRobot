"""
Standup RL Environment for the Two-Wheeled Leg Robot.

The policy learns to self-right from any fallen position to the upright
balancing stance.  Unlike the walk/drive environments this task:

  - Spawns the robot in a fallen orientation each episode (side, forward,
    backward, upside down, or partially fallen).
  - Has NO fall termination — the robot starts fallen, so terminating on
    falls would immediately end every episode.
  - Terminates on SUCCESS (upright for N consecutive steps) or timeout.
  - Gives no velocity commands — the only goal is reaching upright posture.

Designed for deployment on STM32F446RE (Cortex-M4, FPU).  The [32, 32]
network fits in 7.2 KB of Flash and runs in < 1 ms at 50 Hz.

See standup_env_cfg.py for all tunable parameters.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
import isaaclab.utils.math as math_utils
from isaaclab.sensors import Imu
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.sim.spawners.materials import RigidBodyMaterialCfg

from .standup_env_cfg import StandupEnvCfg
from .sim_params import (
    BATTERY_COM_OFFSET_X,
    BATTERY_COM_OFFSET_Y,
    DDSM115_KT,
    GROUND_DYNAMIC_FRICTION,
    GROUND_RESTITUTION,
    GROUND_STATIC_FRICTION,
    WHEEL_INTERNAL_DAMPING,
)


class StandupEnv(DirectRLEnv):
    cfg: StandupEnvCfg

    def __init__(self, cfg: StandupEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # ── Joint indices ─────────────────────────────────────────────────────
        self._left_wheel_ids,  _ = self.robot.find_joints("DDSM115_Levi")
        self._right_wheel_ids, _ = self.robot.find_joints("DDSM115_Desni")
        self._wheel_ids = torch.tensor(
            [self._left_wheel_ids[0], self._right_wheel_ids[0]],
            device=self.device, dtype=torch.long,
        )

        self._cg_fl_ids, _ = self.robot.find_joints("front_left")
        self._cg_fr_ids, _ = self.robot.find_joints("front_right")
        self._cg_bl_ids, _ = self.robot.find_joints("back_left")
        self._cg_br_ids, _ = self.robot.find_joints("back_right")
        self._cg_ids = torch.tensor(
            [self._cg_fl_ids[0], self._cg_fr_ids[0],
             self._cg_bl_ids[0], self._cg_br_ids[0]],
            device=self.device, dtype=torch.long,
        )

        # ── Timing ────────────────────────────────────────────────────────────
        self._control_dt = self.cfg.sim.dt * self.cfg.decimation   # 0.020 s

        # ── CyberGear joint limits ────────────────────────────────────────────
        # Mechanical limits match real hardware hard stops:
        #   fl/br: [-10°, +90°]    fr/bl: [-90°, +10°]
        # The −10° extension beyond mechanical zero lets the legs push slightly
        # past neutral, which helps during self-righting on hard surfaces.
        lim    = math.pi / 2          # 90°  = π/2 rad
        lo_ext = math.radians(10.0)   # 10°  = π/18 rad
        self._cg_joint_lo = torch.tensor(
            [[-lo_ext, -lim, -lim, -lo_ext]], device=self.device, dtype=torch.float32,
        ).expand(self.num_envs, -1).clone()
        self._cg_joint_hi = torch.tensor(
            [[lim, lo_ext, lo_ext, lim]], device=self.device, dtype=torch.float32,
        ).expand(self.num_envs, -1).clone()
        # Sign that converts raw joint angle to positive extension fraction.
        self._cg_ext_sign = torch.tensor(
            [[1.0, -1.0, -1.0, 1.0]], device=self.device, dtype=torch.float32,
        )
        # Scalar used in observation normalisation: total joint range = lo_ext + lim.
        self._cg_norm_span: float = lo_ext + lim   # 100° in radians = 5π/9
        self._final_pose_rad_threshold: float = math.radians(self.cfg.final_pose_deg_threshold)

        self._set_cybergear_physx_limits(log_success=True)

        # ── Action buffers ────────────────────────────────────────────────────
        self._prev_actions = torch.zeros(self.num_envs, self.cfg.action_space, device=self.device)
        self._cur_actions  = torch.zeros(self.num_envs, self.cfg.action_space, device=self.device)

        # ── Pre-allocated hot-path buffers ────────────────────────────────────
        self._efforts_buf = torch.zeros(self.num_envs, 2, device=self.device)
        self._zero_rew    = torch.zeros(self.num_envs,    device=self.device)

        # ── Standup-specific state buffers ────────────────────────────────────
        # XY position at episode spawn — used for displacement penalty.
        self._spawn_pos_xy = torch.zeros(self.num_envs, 2, device=self.device)

        # Reward debug logging: printed every _DEBUG_LOG_INTERVAL global steps.
        # Set to 0 to disable.  Useful for verifying proj_grav_z, displacement, etc.
        self._DEBUG_LOG_INTERVAL: int = 2000
        self._debug_step: int = 0

        # Consecutive steps within the upright success cone.
        self._success_counter = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.long,
        )

        # Tracks whether the one-time success bonus has been paid this episode.
        self._success_bonus_paid = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.bool,
        )

        # ── Domain randomisation baselines ────────────────────────────────────
        self._default_masses          = self.robot.data.default_mass.cpu().clone()
        self._default_joint_stiffness = self.robot.data.default_joint_stiffness.cpu().clone()
        self._default_joint_damping   = self.robot.data.default_joint_damping.cpu().clone()

        # ── Fallen pose sampler — precompute cumulative weights ───────────────
        weights = [
            cfg.spawn_weight_right_side,
            cfg.spawn_weight_left_side,
            cfg.spawn_weight_forward,
            cfg.spawn_weight_backward,
            cfg.spawn_weight_upside_down,
            cfg.spawn_weight_partial,
        ]
        cumw = []
        running = 0.0
        for w in weights:
            running += w
            cumw.append(running)
        self._spawn_cum_weights = torch.tensor(cumw, device=self.device)

        print(
            f"[StandupEnv] num_envs={self.num_envs}  "
            f"obs={cfg.observation_space}  act={cfg.action_space}  "
            f"episode={cfg.episode_length_s:.0f} s  "
            f"success_required={cfg.success_steps_required} steps  "
            f"cg_gains={'fixed' if cfg.cg_use_fixed_gains else 'randomized'}"
        )

    # ── Joint limit helpers ──────────────────────────────────────────────────

    def _set_cybergear_physx_limits(self, log_success: bool = False) -> None:
        """Write mechanical hard stops into PhysX for all CyberGear DOFs."""
        try:
            limits = self.robot.root_physx_view.get_dof_limits().clone()
            cg_cols = [
                self._cg_fl_ids[0], self._cg_fr_ids[0],
                self._cg_bl_ids[0], self._cg_br_ids[0],
            ]
            limits[:, cg_cols, 0] = self._cg_joint_lo[0].cpu()
            limits[:, cg_cols, 1] = self._cg_joint_hi[0].cpu()
            _lim_max = math.pi * 2.0 - 1e-4
            limits.clamp_(-_lim_max, _lim_max)
            all_ids = torch.arange(self.num_envs, dtype=torch.int32)
            self.robot.root_physx_view.set_dof_limits(limits, all_ids)
            if log_success:
                lo0, hi0 = self._cg_joint_lo[0, 0].item(), self._cg_joint_hi[0, 0].item()
                lo1, hi1 = self._cg_joint_lo[0, 1].item(), self._cg_joint_hi[0, 1].item()
                print(
                    f"[StandupEnv] CyberGear PhysX limits set — "
                    f"left [{math.degrees(lo0):.1f}°, {math.degrees(hi0):.1f}°]  "
                    f"right [{math.degrees(lo1):.1f}°, {math.degrees(hi1):.1f}°]"
                )
        except Exception as exc:
            print(f"[StandupEnv] Warning: could not set CyberGear PhysX limits: {exc}")

    def _enforce_cybergear_joint_state_limits(self) -> None:
        """Clamp escaped CyberGear joint state back to the mechanical range."""
        cg_pos = self.robot.data.joint_pos[:, self._cg_ids]
        cg_pos_clamped = torch.max(self._cg_joint_lo, torch.min(self._cg_joint_hi, cg_pos))
        violated = (cg_pos_clamped - cg_pos).abs() > 1.0e-6
        if not torch.any(violated):
            return
        joint_pos = self.robot.data.joint_pos.clone()
        joint_vel = self.robot.data.joint_vel.clone()
        joint_pos[:, self._cg_ids] = cg_pos_clamped
        joint_vel[:, self._cg_ids] = torch.where(
            violated,
            torch.zeros_like(cg_pos_clamped),
            joint_vel[:, self._cg_ids],
        )
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel)

    # ── Scene ─────────────────────────────────────────────────────────────────

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        self.scene.articulations["robot"] = self.robot

        self.bno080 = Imu(self.cfg.bno080)
        self.scene.sensors["bno080"] = self.bno080

        # Flat ground only — no terrain variation for the standup task.
        spawn_ground_plane(
            prim_path="/World/ground",
            cfg=GroundPlaneCfg(
                physics_material=RigidBodyMaterialCfg(
                    static_friction=GROUND_STATIC_FRICTION,
                    dynamic_friction=GROUND_DYNAMIC_FRICTION,
                    restitution=GROUND_RESTITUTION,
                )
            ),
        )

        # ALL body collisions remain enabled — the torso, leg links, and CyberGear
        # housings must physically rest on the ground when the robot is fallen.
        # StandupEnv needs full contact geometry for the self-righting maneuver.
        print("[StandupEnv] Full body collision ENABLED — robot rests on all surfaces when fallen")

        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=["/World/ground"])

        # Battery CoM offset — matches the physical robot layout.
        if BATTERY_COM_OFFSET_X != 0.0 or BATTERY_COM_OFFSET_Y != 0.0:
            try:
                import omni.usd
                from pxr import Gf, UsdPhysics
                stage = omni.usd.get_context().get_stage()
                platform_path = (
                    "/World/envs/env_0/Robot"
                    "/SimplifiedBipedMainAssembly"
                    "/SimplifiedBipedMainAssembly/Platform_Group"
                )
                prim = stage.GetPrimAtPath(platform_path)
                if prim and prim.IsValid():
                    if not prim.HasAPI(UsdPhysics.MassAPI):
                        UsdPhysics.MassAPI.Apply(prim)
                    UsdPhysics.MassAPI(prim).GetCenterOfMassAttr().Set(
                        Gf.Vec3f(BATTERY_COM_OFFSET_X, BATTERY_COM_OFFSET_Y, 0.0)
                    )
            except Exception:
                pass

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
        light_cfg.func("/World/Light", light_cfg)

    # ── Control step ──────────────────────────────────────────────────────────

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        actions = actions.clamp(-1.0, 1.0)
        self._enforce_cybergear_joint_state_limits()

        self._prev_actions = self._cur_actions.clone()
        self._cur_actions  = actions.clone()

        # ── CyberGear position targets ────────────────────────────────────────
        # action=-1 → retracted (mechanical zero), action=+1 → fully extended.
        # Map normalized actions onto the mechanical joint range.
        act01 = 0.5 * (actions[:, 0:4] + 1.0)
        cg_targets = self._cg_joint_lo + act01 * (self._cg_joint_hi - self._cg_joint_lo)
        self.robot.set_joint_position_target(cg_targets, joint_ids=self._cg_ids)

        # ── Wheel torque targets ───────────────────────────────────────────────
        # Left wheel USD mesh is mirrored → negate left torque (same as control.py).
        I_left  = actions[:, 4] * self.cfg.wheel_current_max
        I_right = actions[:, 5] * self.cfg.wheel_current_max
        self._efforts_buf[:, 0] = -I_left  * DDSM115_KT
        self._efforts_buf[:, 1] =  I_right * DDSM115_KT
        self.robot.set_joint_effort_target(self._efforts_buf, joint_ids=self._wheel_ids)

    def _apply_action(self) -> None:
        self.robot.write_data_to_sim()

    # ── Observations ──────────────────────────────────────────────────────────

    def _get_observations(self) -> dict:
        self._enforce_cybergear_joint_state_limits()

        proj_grav = self.bno080.data.projected_gravity_b   # (N, 3)
        ang_vel_b = self.bno080.data.ang_vel_b             # (N, 3)
        cg_pos    = self.robot.data.joint_pos[:, self._cg_ids]   # (N, 4)

        # ── Sensor noise ──────────────────────────────────────────────────────
        proj_grav = proj_grav + torch.randn_like(proj_grav) * self.cfg.noise_proj_grav_std
        ang_vel_b = ang_vel_b + torch.randn_like(ang_vel_b) * self.cfg.noise_ang_vel_std
        cg_pos    = cg_pos    + torch.randn_like(cg_pos)    * self.cfg.noise_cg_pos_std

        # ── Normalise ─────────────────────────────────────────────────────────
        # All 3 gravity components: inherently in [-1, 1] as a unit vector.
        # proj_grav_z = -1 when upright, ≈ 0 when on side, +1 when upside down.
        # The z-component gives unambiguous uprightness signal from any orientation.
        grav_3d = proj_grav                                  # (N, 3) ∈ [-1, 1]

        # Angular velocity: divide by 10 so typical values land in [-1, 1].
        ang_vel_norm = ang_vel_b / 10.0                      # (N, 3)

        # CyberGear: map [-10°, +90°] → [-1, +1].
        # Shift so that -10° → 0, then scale by total span (100°), then remap.
        # span = lo_ext + lim = π/18 + π/2 = 5π/9  (stored as self._cg_norm_span)
        cg_lo_ext = self._cg_norm_span - math.pi / 2    # lo_ext = π/18
        cg_shifted = cg_pos * self._cg_ext_sign + cg_lo_ext   # 0 at −10°, span at +90°
        cg_norm = cg_shifted / self._cg_norm_span * 2.0 - 1.0  # (N, 4) ∈ [-1, 1]

        # Previous 6 actions: already in [-1, 1] by construction.
        prev_act = self._cur_actions                         # (N, 6)

        obs = torch.cat([
            grav_3d,      # [0–2]   full gravity vector in body frame
            ang_vel_norm, # [3–5]   angular velocities / 10 rad/s
            cg_norm,      # [6–9]   CyberGear extension (zero-centred)
            prev_act,     # [10–15] previous 6 actions
        ], dim=1)   # (N, 16)

        return {
            "policy": torch.nan_to_num(
                obs, nan=0.0, posinf=10.0, neginf=-10.0,
            ).clamp(-10.0, 10.0)
        }

    # ── Rewards ───────────────────────────────────────────────────────────────

    def _get_rewards(self) -> torch.Tensor:
        proj_grav = torch.nan_to_num(
            self.bno080.data.projected_gravity_b,
            nan=0.0, posinf=1.0, neginf=-1.0,
        )   # (N, 3)
        cg_pos = self.robot.data.joint_pos[:, self._cg_ids]

        # ── 1. Uprighting reward (primary objective) ──────────────────────────
        # Uses proj_grav_z (body-Z component of gravity vector):
        #   upright:     proj_grav_z = -1  → err = 0 → exp(0)  = 1.0
        #   on side:     proj_grav_z ≈ 0   → err = 1 → exp(-1/0.3) ≈ 0.04
        #   upside down: proj_grav_z = +1  → err = 4 → exp(-4/0.3) ≈ 0.0
        proj_grav_z = proj_grav[:, 2]
        upright_err  = (proj_grav_z + 1.0).pow(2)
        r_uprighting = self.cfg.rew_uprighting_scale * torch.exp(
            -upright_err / self.cfg.rew_uprighting_sigma
        )

        # ── 2. Success bonus (sparse, one-time per episode) ───────────────────
        # Count consecutive steps within the upright cone.
        pitch_ind = proj_grav[:, 1].abs()
        roll_ind  = proj_grav[:, 0].abs()
        # proj_grav_z = -1 when upright, 0 when on side, +1 when upside down.
        # Without this guard an inverted robot (proj_grav = [0, 0, +1]) has
        # pitch_ind = 0 and roll_ind = 0 — falsely satisfying the upright cone.
        is_upright = (
            (pitch_ind < self.cfg.success_pitch_threshold) &
            (roll_ind  < self.cfg.success_roll_threshold) &
            (proj_grav[:, 2] < -0.5)
        )

        zero_pose_err = cg_pos.abs().amax(dim=1)
        is_zero_pose = zero_pose_err < self._final_pose_rad_threshold
        is_final_pose = is_upright & is_zero_pose

        self._success_counter = torch.where(
            is_final_pose,
            self._success_counter + 1,
            torch.zeros_like(self._success_counter),
        )
        just_succeeded = (
            (self._success_counter >= self.cfg.success_steps_required) &
            (~self._success_bonus_paid)
        )
        remaining_frac = (
            (self.max_episode_length - self.episode_length_buf).float() /
            float(self.max_episode_length)
        ).clamp(0.0, 1.0)
        r_success = just_succeeded.float() * (
            self.cfg.rew_success_bonus +
            self.cfg.rew_fast_success_scale * remaining_frac
        )
        self._success_bonus_paid = self._success_bonus_paid | just_succeeded

        # ── 2b. Upright-only final-pose shaping ──────────────────────────────
        # Do not constrain the recovery maneuver itself.  Once upright, reward
        # the policy for bringing all CyberGear joints back near 0° so it ends
        # in a balanced standing stance rather than a mirrored extreme pose.
        zero_pose_err_norm = zero_pose_err / max(self._final_pose_rad_threshold, 1.0e-6)
        r_final_pose = self.cfg.rew_final_pose_scale * is_upright.float() * torch.exp(
            -2.0 * zero_pose_err_norm.pow(2)
        )

        # ── 3. Displacement penalty ───────────────────────────────────────────
        root_pos_xy = self.robot.data.root_pos_w[:, :2]
        delta_xy    = root_pos_xy - self._spawn_pos_xy
        r_disp = self.cfg.rew_displacement_scale * delta_xy.pow(2).sum(dim=1)

        # ── 4. Wheel energy penalty ───────────────────────────────────────────
        I_left  = self._cur_actions[:, 4] * self.cfg.wheel_current_max
        I_right = self._cur_actions[:, 5] * self.cfg.wheel_current_max
        r_wheel_energy = self.cfg.rew_wheel_energy_scale * (I_left.pow(2) + I_right.pow(2))

        # ── 5. CyberGear energy penalty ───────────────────────────────────────
        r_cg_energy = self.cfg.rew_cg_energy_scale * self._cur_actions[:, 0:4].pow(2).sum(dim=1)

        # ── 6. Action rate penalty ────────────────────────────────────────────
        action_rate = (self._cur_actions - self._prev_actions).pow(2).sum(dim=1)
        r_action_rate = self.cfg.rew_action_rate_scale * action_rate

        # ── 7. Time penalty (encourages fast recovery) ────────────────────────
        r_timeout = self.cfg.rew_timeout_penalty   # scalar applied every step

        total = (
            r_uprighting + r_success + r_disp
            + r_final_pose + r_wheel_energy + r_cg_energy + r_action_rate
            + r_timeout
        )

        # ── Debug logging ─────────────────────────────────────────────────────
        if self._DEBUG_LOG_INTERVAL > 0:
            self._debug_step += 1
            if self._debug_step % self._DEBUG_LOG_INTERVAL == 0:
                e = 0   # log env 0 as a representative sample
                pgz = proj_grav[e, 2].item()
                print(
                    f"[StandupEnv step={self._debug_step}]"
                    f"  proj_grav[{e}]={[round(proj_grav[e, i].item(), 3) for i in range(3)]}"
                    f"  r_upright={r_uprighting[e]:.4f}"
                    f"  r_success={r_success[e]:.4f}"
                    f"  r_pose={r_final_pose[e]:.4f}"
                    f"  r_disp={r_disp[e]:.4f}"
                    f"  zero_pose_deg_max={math.degrees(zero_pose_err[e].item()):.1f}"
                    f"  disp_m={delta_xy[e].norm().item():.3f}"
                    f"  total={total[e]:.4f}"
                    f"  pgz_mean={proj_grav[:, 2].mean().item():.3f}"
                    f"  upright_frac={is_upright.float().mean().item():.3f}"
                    f"  final_pose_frac={is_final_pose.float().mean().item():.3f}"
                )

        return torch.nan_to_num(total, nan=0.0, posinf=2.0, neginf=-10.0).clamp(-10.0, 2.0)

    # ── Termination ───────────────────────────────────────────────────────────

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        self._enforce_cybergear_joint_state_limits()

        # Success: robot has been upright for the required number of steps.
        succeeded = self._success_counter >= self.cfg.success_steps_required

        # Physics guard: terminate environments where the robot fell through the
        # ground (body_z < -1.0) — this indicates a NaN / inf physics state.
        body_z = torch.nan_to_num(
            self.robot.data.root_pos_w[:, 2], nan=-999.0,
        )
        physics_broken = body_z < -1.0

        terminated = succeeded | physics_broken
        timeout    = self.episode_length_buf >= self.max_episode_length - 1

        return terminated, timeout

    # ── Reset ─────────────────────────────────────────────────────────────────

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)

        n = len(env_ids)
        ids_t = (
            env_ids if isinstance(env_ids, torch.Tensor)
            else torch.tensor(env_ids, device=self.device, dtype=torch.long)
        )

        # ── Reset per-env buffers ─────────────────────────────────────────────
        self._prev_actions[env_ids]       = 0.0
        self._cur_actions[env_ids]        = 0.0
        self._success_counter[ids_t]      = 0
        self._success_bonus_paid[ids_t]   = False

        # ── Sample fallen spawn orientations ──────────────────────────────────
        fallen_quat, spawn_z = self._sample_fallen_poses(n)

        # ── Build root state ──────────────────────────────────────────────────
        root_state = self.robot.data.default_root_state[env_ids].clone()
        root_state[:, :3] += self.scene.env_origins[env_ids]
        root_state[:, 2]   = self.scene.env_origins[env_ids, 2] + spawn_z
        root_state[:, 3:7] = fallen_quat
        root_state[:, 7:]  = 0.0   # zero all velocities — start from rest

        # Store spawn XY for displacement penalty.
        self._spawn_pos_xy[env_ids] = root_state[:, :2]

        self.robot.write_root_pose_to_sim(root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(root_state[:, 7:], env_ids)

        # ── CyberGear initialisation — all joints at mechanical zero ──────────
        joint_pos = self.robot.data.default_joint_pos[env_ids].clone()
        joint_vel = self.robot.data.default_joint_vel[env_ids].clone()
        joint_pos[:, self._cg_ids] = 0.0
        joint_vel[:, self._cg_ids] = 0.0
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        self.robot.set_joint_position_target(joint_pos, env_ids=env_ids)
        self._set_cybergear_physx_limits()

        # ── Domain randomisation ───────────────────────────────────────────────
        env_ids_cpu = (
            env_ids.cpu()
            if isinstance(env_ids, torch.Tensor)
            else torch.tensor(env_ids)
        )

        n_joints = self._default_joint_stiffness.shape[1]
        cg_cols  = [self._cg_fl_ids[0], self._cg_fr_ids[0],
                    self._cg_bl_ids[0], self._cg_br_ids[0]]
        wheel_cols = [self._left_wheel_ids[0], self._right_wheel_ids[0]]

        # 1. Body mass ±20 %
        lo_m, hi_m = self.cfg.mass_scale_range
        mass_scale = torch.empty(n, 1).uniform_(lo_m, hi_m)
        all_masses = self.robot.root_physx_view.get_masses()
        all_masses[env_ids_cpu] = self._default_masses[env_ids_cpu] * mass_scale
        self.robot.root_physx_view.set_masses(all_masses, env_ids_cpu)

        # 2. CyberGear stiffness / damping
        if self.cfg.cg_use_fixed_gains:
            joint_stiffness = self._default_joint_stiffness[env_ids_cpu].clone()
            joint_damping = self._default_joint_damping[env_ids_cpu].clone()
            joint_stiffness[:, cg_cols] = self.cfg.cg_fixed_kp
            joint_damping[:, cg_cols] = self.cfg.cg_fixed_kd
            self.robot.write_joint_stiffness_to_sim(
                joint_stiffness.to(self.device),
                env_ids=env_ids_cpu,
            )
            self.robot.write_joint_damping_to_sim(
                joint_damping.to(self.device),
                env_ids=env_ids_cpu,
            )
        else:
            lo_kp, hi_kp = self.cfg.cg_kp_scale_range
            kp_scale = torch.ones(n, n_joints)
            kp_scale[:, cg_cols] = torch.empty(n, 4).uniform_(lo_kp, hi_kp)
            self.robot.write_joint_stiffness_to_sim(
                (self._default_joint_stiffness[env_ids_cpu] * kp_scale).to(self.device),
                env_ids=env_ids_cpu,
            )

            lo_kd, hi_kd = self.cfg.cg_kd_scale_range
            kd_scale = torch.ones(n, n_joints)
            kd_scale[:, cg_cols] = torch.empty(n, 4).uniform_(lo_kd, hi_kd)
            self.robot.write_joint_damping_to_sim(
                (self._default_joint_damping[env_ids_cpu] * kd_scale).to(self.device),
                env_ids=env_ids_cpu,
            )

        # 4. Wheel rotational friction ±50 %
        lo_wd, hi_wd = self.cfg.wheel_damping_scale_range
        fric_default = torch.zeros(n, n_joints)
        fric_default[:, wheel_cols] = WHEEL_INTERNAL_DAMPING
        fric_scale = torch.ones(n, n_joints)
        fric_scale[:, wheel_cols] = torch.empty(n, 2).uniform_(lo_wd, hi_wd)
        self.robot.write_joint_friction_coefficient_to_sim(
            (fric_default * fric_scale).to(self.device),
            env_ids=env_ids_cpu,
        )

    # ── Fallen pose sampler ───────────────────────────────────────────────────

    def _sample_fallen_poses(self, n: int) -> tuple[torch.Tensor, torch.Tensor]:
        """Sample fallen spawn orientations for `n` environments.

        Returns:
            fallen_quat: (n, 4) quaternions [w, x, y, z] for fallen poses.
            spawn_z:     (n,)   heights above env origin (m).

        Scenarios (roll and pitch in body-frame Euler XYZ):
            0 right_side   roll = -π/2,  pitch = 0       (tilted one way)
            1 left_side    roll = +π/2,  pitch = 0       (tilted other way)
            2 forward      roll =  0,    pitch = +π/2    (fallen nose-down)
            3 backward     roll =  0,    pitch = -π/2    (fallen tail-down)
            4 upside_down  roll =  π,    pitch = 0       (completely inverted)
            5 partial      random angle ∈ [30°, 180°] on random roll/pitch blend
        """
        # ── Categorical scenario assignment via searchsorted ──────────────────
        u = torch.rand(n, device=self.device)
        scenario = torch.searchsorted(self._spawn_cum_weights, u).clamp(0, 5)

        right_mask = (scenario == 0)
        left_mask  = (scenario == 1)
        fwd_mask   = (scenario == 2)
        back_mask  = (scenario == 3)
        up_mask    = (scenario == 4)
        part_mask  = (scenario == 5)

        # ── Base Euler angles for canonical scenarios ─────────────────────────
        base_roll  = torch.zeros(n, device=self.device)
        base_pitch = torch.zeros(n, device=self.device)

        base_roll  = torch.where(right_mask, torch.full((n,), -math.pi / 2, device=self.device), base_roll)
        base_roll  = torch.where(left_mask,  torch.full((n,),  math.pi / 2, device=self.device), base_roll)
        base_pitch = torch.where(fwd_mask,   torch.full((n,),  math.pi / 2, device=self.device), base_pitch)
        base_pitch = torch.where(back_mask,  torch.full((n,), -math.pi / 2, device=self.device), base_pitch)
        base_roll  = torch.where(up_mask,    torch.full((n,),  math.pi,     device=self.device), base_roll)

        # ── Partial falls: random angle on a random blend of roll and pitch ───
        partial_angle = torch.empty(n, device=self.device).uniform_(
            math.radians(30.0), math.pi,
        )
        blend = torch.rand(n, device=self.device)   # 0 = pure roll, 1 = pure pitch
        roll_sign  = torch.where(
            torch.rand(n, device=self.device) > 0.5,
            torch.ones(n, device=self.device), -torch.ones(n, device=self.device),
        )
        pitch_sign = torch.where(
            torch.rand(n, device=self.device) > 0.5,
            torch.ones(n, device=self.device), -torch.ones(n, device=self.device),
        )
        partial_roll  = roll_sign  * torch.sqrt((1.0 - blend).clamp(min=0.0)) * partial_angle
        partial_pitch = pitch_sign * torch.sqrt(blend.clamp(min=0.0))         * partial_angle

        base_roll  = torch.where(part_mask, partial_roll,  base_roll)
        base_pitch = torch.where(part_mask, partial_pitch, base_pitch)

        # ── Gaussian noise on canonical scenarios (not on partial — already random) ──
        noise_std   = self.cfg.spawn_tilt_noise_std
        roll_noise  = torch.randn(n, device=self.device) * noise_std
        pitch_noise = torch.randn(n, device=self.device) * noise_std
        roll_noise  = torch.where(part_mask, torch.zeros(n, device=self.device), roll_noise)
        pitch_noise = torch.where(part_mask, torch.zeros(n, device=self.device), pitch_noise)

        final_roll  = base_roll  + roll_noise
        final_pitch = base_pitch + pitch_noise
        final_yaw   = torch.empty(n, device=self.device).uniform_(-math.pi, math.pi)

        # ── Build quaternions ─────────────────────────────────────────────────
        fallen_quat = math_utils.quat_from_euler_xyz(final_roll, final_pitch, final_yaw)

        # ── Geometric spawn height ────────────────────────────────────────────
        # Estimate CoM height above ground given the tilt angle from vertical.
        # For a box-like body: z ≈ R·|cos(tilt)| + L·|sin(tilt)| + clearance
        # where R = upright CoM height, L = lateral half-extent at CoM level.
        #
        # Examples:
        #   tilt =  0° (upright)   → 0.069 + 0     + 0.04 = 0.11 m
        #   tilt = 90° (on side)   → 0     + 0.13  + 0.04 = 0.17 m
        #   tilt = 180° (inverted) → 0.069 + 0     + 0.04 = 0.11 m
        #
        # Much better than a universal 0.20 m which causes a 2 m/s impact,
        # 10 chaotic control steps, and inflated displacement penalties.
        tilt_cos = (torch.cos(final_roll) * torch.cos(final_pitch)).abs()
        tilt_sin = torch.sqrt((1.0 - tilt_cos.pow(2)).clamp(min=0.0))
        spawn_z = (
            self.cfg.spawn_upright_z * tilt_cos
            + self.cfg.spawn_lateral_h * tilt_sin
            + self.cfg.spawn_clearance
        ).clamp(min=self.cfg.spawn_upright_z)

        return fallen_quat, spawn_z
