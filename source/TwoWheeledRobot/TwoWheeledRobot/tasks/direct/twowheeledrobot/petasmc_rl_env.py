"""
PETASMC Gain-Scheduling RL Environment.

The RL policy observes robot state and PETASMC diagnostics, and outputs
the four most impactful controller gains each step:

    action[0] → T_min_bal   periodic floor for balance trigger  [s]
    action[1] → T_min_yaw   periodic floor for heading trigger  [s]
    action[2] → K_max_bal   adaptive gain ceiling, balance      [A]
    action[3] → K_max_yaw   adaptive gain ceiling, heading      [A]

The underlying PETASMC controller runs unchanged at 50 Hz.  The RL policy
also runs at 50 Hz and updates the four gains every control step.

Observation (10 dims):
    [0]  θ           tilt angle                      rad
    [1]  θ̇           tilt rate                       rad/s
    [2]  ψ̇           yaw rate                        rad/s
    [3]  v           forward wheel velocity           m/s
    [4]  v_cmd       commanded forward velocity       m/s
    [5]  s_bal       balance sliding surface          rad
    [6]  s_yaw       heading sliding surface          rad
    [7]  IET_bal / T_min_bal   normalised IET (≥1 = at floor or sparser)
    [8]  IET_yaw / T_min_yaw
    [9]  T_min_bal normalised to [0, 1] over action range

Reward (all positive terms gated on |θ| < 15° — no reward while fallen):
    +2.0 · exp(−θ² / 0.01)        balance quality
    +1.0 · exp(−(v − v_cmd)²/0.25) velocity tracking  [upright only]
    +0.1 · (1 − trig_bal)         sparsity bonus       [upright only]
    +0.5                           alive bonus          [upright only, else −1.0]
    −200 on fall (immediate — episode ends the step tilt > max_pitch_deg)

Domain randomisation per episode:
    Robot mass ±15 %
    Random initial tilt ± 2.3°
    Random forward velocity command ± 0.3 m/s
    Random push impulses every ~4 s
    IMU-level observation noise

Training command (from TwoWheeledRobot/ directory):
    python scripts/rsl_rl/train.py \\
        --task Template-Twowheeledrobot-Petasmc-v0 \\
        --headless --num_envs 2048
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sensors import Imu
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.sim.spawners.materials import RigidBodyMaterialCfg

from .petasmc_rl_env_cfg import PetasmcRlEnvCfg
from .tb_logger import TBLogger
from .control import (
    RobotController,
    IMU_TILT_GRAVITY_AXIS, IMU_TILT_RATE_AXIS, IMU_YAW_RATE_AXIS,
    WHEEL_RADIUS,
    ET_MIN_IET_BAL, ET_MIN_IET_YAW,
    ET_K_MAX_BAL, ET_K_MAX_YAW,
)
from .kinematics import set_leg_foot_position, L1_C, BASE_TARGET_Y
from .sim_params import (
    GROUND_STATIC_FRICTION, GROUND_DYNAMIC_FRICTION, GROUND_RESTITUTION,
    BATTERY_COM_OFFSET_X, BATTERY_COM_OFFSET_Y,
)


def _map_action(a: torch.Tensor, lo: float, hi: float) -> torch.Tensor:
    """Linearly map action from [-1, 1] → [lo, hi]."""
    return lo + (a + 1.0) * 0.5 * (hi - lo)


class PetasmcRlEnv(DirectRLEnv):
    cfg: PetasmcRlEnvCfg

    def __init__(self, cfg: PetasmcRlEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # ── Joint indices ────────────────────────────────────────────────────
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

        # ── Timing ───────────────────────────────────────────────────────────
        self._control_dt = self.cfg.sim.dt * self.cfg.decimation   # 20 ms

        # ── Controller ───────────────────────────────────────────────────────
        self._controller = RobotController(
            num_envs=self.num_envs,
            device=self.device,
            dt=self._control_dt,
        )

        # ── Tilt threshold for termination ───────────────────────────────────
        self._max_tilt_sin = math.sin(math.radians(self.cfg.max_pitch_deg))

        # ── Per-env velocity commands ────────────────────────────────────────
        self._vel_cmd      = torch.zeros(self.num_envs, device=self.device)
        self._yaw_rate_cmd = torch.zeros(self.num_envs, device=self.device)

        # ── Current gain settings (needed for obs normalisation) ─────────────
        self._t_min_bal_cur = torch.full(
            (self.num_envs,), ET_MIN_IET_BAL, device=self.device)
        self._t_min_yaw_cur = torch.full(
            (self.num_envs,), ET_MIN_IET_YAW, device=self.device)

        # ── Domain randomisation ─────────────────────────────────────────────
        # default_mass is a CPU tensor (physx view); keep it on CPU for set_masses()
        self._default_masses = self.robot.data.default_mass.cpu().clone()  # (N, num_bodies)

        # Push disturbance timers
        self._push_timer      = torch.zeros(self.num_envs, device=self.device)
        self._next_push_time  = (
            torch.rand(self.num_envs, device=self.device)
            * self.cfg.push_interval_s + self.cfg.push_interval_s * 0.5
        )

        # ── TensorBoard logger ───────────────────────────────────────────────
        # Logs mean controller diagnostics across all envs every 200 steps.
        # View with: tensorboard --logdir /workspace/TwoWheeledRobot/logs/tensorboard
        self._tb = TBLogger(enabled=True, log_every_n=200)
        self._tb_step = 0

        print(f"[PETASMC-RL] num_envs={self.num_envs}  "
              f"control_dt={self._control_dt*1000:.1f} ms  "
              f"obs={cfg.observation_space}  act={cfg.action_space}")

    # ── Scene ────────────────────────────────────────────────────────────────

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        self.scene.articulations["robot"] = self.robot

        self.bno080 = Imu(self.cfg.bno080)
        self.scene.sensors["bno080"] = self.bno080

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

        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=["/World/ground"])

        # Battery CoM offset (mirrors base env)
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

    # ── Control step ─────────────────────────────────────────────────────────

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        actions = actions.clamp(-1.0, 1.0)

        # ── 1. Map actions → physical gain values ────────────────────────────
        t_min_bal = _map_action(actions[:, 0], self.cfg.t_min_bal_lo, self.cfg.t_min_bal_hi)
        t_min_yaw = _map_action(actions[:, 1], self.cfg.t_min_yaw_lo, self.cfg.t_min_yaw_hi)
        k_max_bal = _map_action(actions[:, 2], self.cfg.k_max_bal_lo, self.cfg.k_max_bal_hi)
        k_max_yaw = _map_action(actions[:, 3], self.cfg.k_max_yaw_lo, self.cfg.k_max_yaw_hi)

        # Cache for observations
        self._t_min_bal_cur = t_min_bal
        self._t_min_yaw_cur = t_min_yaw

        # ── 2. Inject gains and commands into controller ──────────────────────
        # Per-env tensors broadcast correctly with the vectorised controller ops.
        self._controller.et_min_iet_bal = t_min_bal
        self._controller.et_min_iet_yaw = t_min_yaw
        self._controller.et_k_max_bal   = k_max_bal
        self._controller.et_k_max_yaw   = k_max_yaw
        self._controller.vel_cmd        = self._vel_cmd
        self._controller.yaw_rate_cmd   = self._yaw_rate_cmd

        # ── 3. Random push disturbances ──────────────────────────────────────
        self._push_timer += self._control_dt
        push_mask = self._push_timer >= self._next_push_time
        n_push = int(push_mask.sum().item())
        if n_push > 0:
            push_ids = push_mask.nonzero(as_tuple=False).squeeze(-1)
            mag = (torch.rand(n_push, device=self.device)
                   * (self.cfg.push_vel_max - self.cfg.push_vel_min)
                   + self.cfg.push_vel_min)
            angle = torch.rand(n_push, device=self.device) * 2.0 * math.pi
            root_vel = self.robot.data.root_vel_w[push_ids].clone()
            root_vel[:, 0] += mag * torch.cos(angle)
            root_vel[:, 1] += mag * torch.sin(angle)
            self.robot.write_root_velocity_to_sim(root_vel, push_ids)
            self._push_timer[push_ids] = 0.0
            self._next_push_time[push_ids] = (
                torch.rand(n_push, device=self.device)
                * self.cfg.push_interval_s + self.cfg.push_interval_s * 0.5
            )

        # ── 4. Run PETASMC controller ─────────────────────────────────────────
        proj_grav   = self.bno080.data.projected_gravity_b
        ang_vel_b   = self.bno080.data.ang_vel_b
        wheel_vel   = self.robot.data.joint_vel[:, self._wheel_ids]
        wheel_pos   = self.robot.data.joint_pos[:, self._wheel_ids]

        torque_left, torque_right, cg_pos, cg_vel, cg_ff = self._controller.compute(
            projected_gravity_b=proj_grav,
            ang_vel_b=ang_vel_b,
            omega_left=wheel_vel[:, 0],
            omega_right=wheel_vel[:, 1],
            phi_left=wheel_pos[:, 0],
            phi_right=wheel_pos[:, 1],
        )

        # ── 5. Log diagnostics ────────────────────────────────────────────────
        self._tb_step += 1
        if self._tb.enabled:
            ctrl = self._controller
            proj_grav_log = self.bno080.data.projected_gravity_b
            theta_log = torch.asin(proj_grav_log[:, IMU_TILT_GRAVITY_AXIS].clamp(-1.0, 1.0))
            self._tb.push({
                # Mean absolute tilt across all envs
                "tilt":      float(theta_log.abs().mean()) * 57.296,
                "tilt_rate": float(ang_vel_b[:, IMU_TILT_RATE_AXIS].abs().mean()),
                "yaw_rate":  float(ang_vel_b[:, IMU_YAW_RATE_AXIS].abs().mean()),
                "vel_fwd":   float((-wheel_vel[:, 0] + wheel_vel[:, 1]).mul(0.5 * WHEEL_RADIUS).mean()),
                "vel_cmd":   float(self._vel_cmd.mean()),
                # SMC surfaces — mean abs across all envs
                "s_bal":     float(ctrl._s_bal_cur.abs().mean()),
                "s_yaw":     float(ctrl._s_yaw_cur.abs().mean()),
                # Event rates — fraction of envs that triggered this step
                "trig_bal":  float(ctrl._trig_bal_cur.float().mean()),
                "trig_yaw":  float(ctrl._trig_yaw_cur.float().mean()),
                # Inter-event times — mean across envs
                "iet_bal":   float(ctrl._iet_bal.mean()),
                "iet_yaw":   float(ctrl._iet_yaw.mean()),
                # RL-tuned gains — mean across envs
                "torque_L":  float(torque_left.abs().mean()),
                "torque_R":  float(torque_right.abs().mean()),
                "u_bal":     float(ctrl._u_bal_hold.abs().mean()),
                "u_yaw":     float(ctrl._u_yaw_hold.abs().mean()),
                "k_bal":     float(k_max_bal.mean()),
            })

        # ── 6. Apply to simulation ────────────────────────────────────────────
        efforts = torch.stack([torque_left, torque_right], dim=1)
        self.robot.set_joint_effort_target(efforts, joint_ids=self._wheel_ids)
        self.robot.set_joint_position_target(cg_pos, joint_ids=self._cg_ids)
        self.robot.set_joint_velocity_target(cg_vel, joint_ids=self._cg_ids)
        self.robot.set_joint_effort_target   (cg_ff,  joint_ids=self._cg_ids)

    def _apply_action(self) -> None:
        self.robot.write_data_to_sim()

    # ── Observations ─────────────────────────────────────────────────────────

    def _get_observations(self) -> dict:
        proj_grav = self.bno080.data.projected_gravity_b   # (N, 3)
        ang_vel_b = self.bno080.data.ang_vel_b             # (N, 3)
        wheel_vel = self.robot.data.joint_vel[:, self._wheel_ids]

        theta     = torch.asin(proj_grav[:, IMU_TILT_GRAVITY_AXIS].clamp(-1.0, 1.0))
        theta_dot = ang_vel_b[:, IMU_TILT_RATE_AXIS]
        psi_dot   = ang_vel_b[:, IMU_YAW_RATE_AXIS]
        v         = (-wheel_vel[:, 0] + wheel_vel[:, 1]) * 0.5 * WHEEL_RADIUS

        # IMU-level observation noise for sim-to-real robustness
        theta     = theta     + torch.randn_like(theta)     * self.cfg.noise_theta
        theta_dot = theta_dot + torch.randn_like(theta_dot) * self.cfg.noise_theta_dot
        psi_dot   = psi_dot   + torch.randn_like(psi_dot)   * self.cfg.noise_psi_dot
        v         = v         + torch.randn_like(v)         * self.cfg.noise_vel

        # IET normalised: 1.0 = triggering at the periodic floor, >1 = sparser
        iet_bal_norm = (
            self._controller._iet_bal / self._t_min_bal_cur.clamp(min=1e-6)
        ).clamp(0.0, 5.0)
        iet_yaw_norm = (
            self._controller._iet_yaw / self._t_min_yaw_cur.clamp(min=1e-6)
        ).clamp(0.0, 5.0)

        # Current T_min_bal setting normalised to [0, 1] over the action range
        t_min_bal_norm = (
            (self._t_min_bal_cur - self.cfg.t_min_bal_lo)
            / (self.cfg.t_min_bal_hi - self.cfg.t_min_bal_lo)
        ).clamp(0.0, 1.0)

        obs = torch.stack([
            theta,                            # [0] tilt (rad)
            theta_dot,                        # [1] tilt rate (rad/s)
            psi_dot,                          # [2] yaw rate (rad/s)
            v,                                # [3] forward velocity (m/s)
            self._vel_cmd,                    # [4] commanded velocity (m/s)
            self._controller._s_bal_cur,      # [5] balance surface (rad)
            self._controller._s_yaw_cur,      # [6] heading surface (rad)
            iet_bal_norm,                     # [7] IET_bal / T_min_bal
            iet_yaw_norm,                     # [8] IET_yaw / T_min_yaw
            t_min_bal_norm,                   # [9] current T_min_bal setting
        ], dim=1)   # (N, 10)

        return {"policy": obs.clamp(-10.0, 10.0)}

    # ── Rewards ──────────────────────────────────────────────────────────────

    def _get_rewards(self) -> torch.Tensor:
        proj_grav = self.bno080.data.projected_gravity_b
        tilt_sin  = proj_grav[:, IMU_TILT_GRAVITY_AXIS]
        theta     = torch.asin(tilt_sin.clamp(-1.0, 1.0))

        wheel_vel = self.robot.data.joint_vel[:, self._wheel_ids]
        v = (-wheel_vel[:, 0] + wheel_vel[:, 1]) * 0.5 * WHEEL_RADIUS

        # Upright mask: robot is meaningfully balanced (tilt < 15°).
        # All positive rewards are gated on this — the RL must not be able to
        # accumulate reward while lying on the floor.
        upright = tilt_sin.abs() < math.sin(math.radians(15.0))   # (N,) bool

        # Balance quality — Gaussian centred at θ=0 (σ ≈ 5.7°).
        # Naturally falls to ~0 beyond ≈15°, so no extra masking needed.
        r_balance = 2.0 * torch.exp(-(theta.pow(2)) / 0.01)

        # Velocity tracking — only meaningful when upright.
        r_vel = 1.0 * torch.exp(-((v - self._vel_cmd).pow(2)) / 0.25) * upright.float()

        # Sparsity bonus — ZOH hold while fallen is not a virtue.
        r_sparse = 0.1 * (~self._controller._trig_bal_cur).float() * upright.float()

        # Alive bonus becomes a tilt penalty when the robot is significantly off
        # vertical.  +0.5 / step when balanced, −1.0 / step when tilted ≥ 15°.
        # This forces the RL to learn that tilted time costs, not just terminal falls.
        r_alive = torch.where(upright,
                              torch.full_like(theta, 0.5),
                              torch.full_like(theta, -1.0))

        # Terminal fall penalty — episode terminates this same step (_get_dones),
        # so missed future rewards already punish falls. A large one-shot penalty
        # (e.g. -200) adds variance to mean_reward without meaningfully changing
        # the learned policy. Kept small so the spike doesn't dominate the graph.
        r_fall = -10.0 * (tilt_sin.abs() > self._max_tilt_sin).float()

        return (r_balance + r_vel + r_sparse + r_alive + r_fall).clamp(-20.0, 10.0)

    # ── Termination ──────────────────────────────────────────────────────────

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        # Terminate immediately on fall — the robot cannot recover from the ground.
        tilt   = self.bno080.data.projected_gravity_b[:, IMU_TILT_GRAVITY_AXIS]
        fallen  = tilt.abs() > self._max_tilt_sin
        timeout = self.episode_length_buf >= self.max_episode_length - 1
        return fallen, timeout

    # ── Reset ────────────────────────────────────────────────────────────────

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)

        n = len(env_ids)

        # ── Timers ───────────────────────────────────────────────────────────
        self._push_timer[env_ids]     = 0.0
        self._next_push_time[env_ids] = (
            torch.rand(n, device=self.device) * self.cfg.push_interval_s
            + self.cfg.push_interval_s * 0.5
        )
        self._controller.reset(env_ids)

        # ── Domain randomisation — mass ───────────────────────────────────────
        # physx view API operates on CPU tensors
        env_ids_cpu = env_ids.cpu() if isinstance(env_ids, torch.Tensor) else torch.tensor(env_ids)
        mass_scale = (
            torch.empty(n, 1)
            .uniform_(self.cfg.mass_scale_min, self.cfg.mass_scale_max)
        )   # CPU
        all_masses = self.robot.root_physx_view.get_masses()   # (N, num_bodies) CPU
        all_masses[env_ids_cpu] = self._default_masses[env_ids_cpu] * mass_scale
        self.robot.root_physx_view.set_masses(all_masses, env_ids_cpu)

        # ── Velocity commands — resample each episode ─────────────────────────
        self._vel_cmd[env_ids] = (
            (torch.rand(n, device=self.device) * 2.0 - 1.0) * self.cfg.vel_cmd_max
        )
        self._yaw_rate_cmd[env_ids] = 0.0

        # ── Initial robot pose ────────────────────────────────────────────────
        root_state = self.robot.data.default_root_state[env_ids].clone()
        root_state[:, :3] += self.scene.env_origins[env_ids]

        # Small random tilt around the pitch axis (Y rotation: [cos, 0, sin, 0])
        tilt = torch.randn(n, device=self.device) * self.cfg.init_tilt_std
        half = tilt * 0.5
        cos_h = torch.cos(half)
        sin_h = torch.sin(half)
        q_tilt = torch.stack([cos_h,
                               torch.zeros_like(cos_h),
                               sin_h,
                               torch.zeros_like(cos_h)], dim=1)
        # Quaternion multiply: q_new = q_orig ⊗ q_tilt  (wxyz convention)
        w1, x1, y1, z1 = (root_state[:, 3], root_state[:, 4],
                           root_state[:, 5], root_state[:, 6])
        w2, x2, y2, z2 = q_tilt[:, 0], q_tilt[:, 1], q_tilt[:, 2], q_tilt[:, 3]
        q_new = torch.stack([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ], dim=1)
        root_state[:, 3:7] = torch.nn.functional.normalize(q_new, dim=1)

        self.robot.write_root_pose_to_sim(root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(root_state[:, 7:], env_ids)

        # ── CyberGear upright-stance initialisation ───────────────────────────
        joint_pos = self.robot.data.default_joint_pos[env_ids].clone()
        joint_vel = self.robot.data.default_joint_vel[env_ids].clone()
        ik_eq = set_leg_foot_position(L1_C * 0.5, BASE_TARGET_Y)
        if ik_eq is not None:
            lf0, lb0 = ik_eq
            cg_eq = torch.tensor(
                [[-lf0, lf0, -lb0, lb0]],
                device=self.device, dtype=torch.float32,
            ).expand(n, -1)
            joint_pos[:, self._cg_ids] = cg_eq
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        self.robot.set_joint_position_target(joint_pos, env_ids=env_ids)
