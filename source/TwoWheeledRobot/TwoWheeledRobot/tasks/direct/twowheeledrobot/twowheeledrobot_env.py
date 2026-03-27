"""
Two-Wheeled Inverted Pendulum — simulation environment.

Scene setup, sensor wiring, and the physics step are handled here.
All control logic and gain tuning lives in control.py.

Usage (pure control, no RL):
    python scripts/zero_agent.py --task Template-Twowheeledrobot-Direct-v0
"""

from __future__ import annotations

import math
import torch
from collections.abc import Sequence

import carb.input
import omni.appwindow

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sensors import Imu
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.sim.spawners.materials import RigidBodyMaterialCfg

from .twowheeledrobot_env_cfg import TwowheeledrobotEnvCfg
from .control import (
    RobotController,
    FWD_VEL_MAX, FWD_VEL_RAMP, FWD_VEL_DECAY, YAW_RATE_MAX,
    IMU_TILT_GRAVITY_AXIS, IMU_TILT_RATE_AXIS, IMU_YAW_RATE_AXIS,
    WHEEL_RADIUS,
    set_leg_foot_position, L1_C, BASE_TARGET_Y,
)
from .sim_params import GROUND_STATIC_FRICTION, GROUND_DYNAMIC_FRICTION, GROUND_RESTITUTION


class TwowheeledrobotEnv(DirectRLEnv):
    cfg: TwowheeledrobotEnvCfg

    def __init__(self, cfg: TwowheeledrobotEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # ---- Print all joints (useful for verification) ------------------ #
        _, all_joint_names = self.robot.find_joints(".*")
        print(f"[TwoWheeledRobot] All joints in USD: {all_joint_names}")

        # ---- Wheel joint indices ----------------------------------------- #
        self._left_wheel_ids,  _ = self.robot.find_joints("DDSM115_Levi")
        self._right_wheel_ids, _ = self.robot.find_joints("DDSM115_Desni")
        self._wheel_ids = torch.tensor(
            [self._left_wheel_ids[0], self._right_wheel_ids[0]],
            device=self.device, dtype=torch.long,
        )

        # ---- CyberGear joint indices (one per motor) --------------------- #
        # Mapping: LF/LB = left leg front/back, RF/RB = right leg front/back
        self._cg_fl_ids, _ = self.robot.find_joints("front_left")
        self._cg_fr_ids, _ = self.robot.find_joints("front_right")
        self._cg_bl_ids, _ = self.robot.find_joints("back_left")
        self._cg_br_ids, _ = self.robot.find_joints("back_right")
        # Packed in the same order as cg_angles returned by controller: [fl, fr, bl, br]
        self._cg_ids = torch.tensor(
            [self._cg_fl_ids[0], self._cg_fr_ids[0],
             self._cg_bl_ids[0], self._cg_br_ids[0]],
            device=self.device, dtype=torch.long,
        )

        # ---- Control timestep -------------------------------------------- #
        dt = self.cfg.sim.dt * self.cfg.decimation

        # ---- Controller -------------------------------------------------- #
        self._controller = RobotController(
            num_envs=self.num_envs,
            device=self.device,
            dt=dt,
        )

        # ---- Tilt limit -------------------------------------------------- #
        self._max_pitch = math.radians(self.cfg.max_pitch_deg)

        # ---- Debug print throttle ---------------------------------------- #
        self._debug_print_interval: int = 50   # set to 0 to disable
        self._debug_step_count: int = 0

        # ---- Keyboard (WASD teleop) --------------------------------------- #
        self._input    = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()

        print(f"[TwoWheeledRobot] Wheel IDs  — Left: {self._left_wheel_ids}, "
              f"Right: {self._right_wheel_ids}")
        print(f"[TwoWheeledRobot] CyberGear IDs — FL:{self._cg_fl_ids} FR:{self._cg_fr_ids} "
              f"BL:{self._cg_bl_ids} BR:{self._cg_br_ids}")
        print(f"[TwoWheeledRobot] Control dt = {dt:.4f} s")
        print("[TwoWheeledRobot] WASD teleop — W/S = forward/back, A/D = turn")

        # ---- Mass audit (printed once at startup) ------------------------- #
        body_masses = self.robot.data.default_mass[0]   # (num_bodies,)
        total_mass  = float(body_masses.sum())
        _, body_names = self.robot.find_bodies(".*")
        print("[TwoWheeledRobot] ── Body mass audit ──────────────────────────")
        for name, m in zip(body_names, body_masses.tolist()):
            print(f"  {name:<25s} {m:.4f} kg")
        print(f"  {'TOTAL':<25s} {total_mass:.4f} kg  "
              f"→ weight = {total_mass * 9.81:.2f} N  "
              f"→ max static traction = {total_mass * 9.81 * GROUND_STATIC_FRICTION:.2f} N  "
              f"→ slip above τ = {total_mass * 9.81 * GROUND_STATIC_FRICTION / 2 * WHEEL_RADIUS:.3f} Nm/wheel")
        print("[TwoWheeledRobot] ─────────────────────────────────────────────")

    # ---------------------------------------------------------------------- #
    # Scene setup                                                             #
    # ---------------------------------------------------------------------- #
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        self.scene.articulations["robot"] = self.robot

        self.imu = Imu(self.cfg.imu)
        self.scene.sensors["imu"] = self.imu

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

        # ---- Disable ground collision on all non-wheel bodies ---------------- #
        # The USD (Onshape export) has collision enabled on EVERY part:
        # leg links, CyberGear motors, holders, battery, cover, etc.
        # With self-collisions disabled these only collide with the GROUND,
        # creating enormous hidden friction that caps wheel velocity at ~0.15 m/s.
        # Keep collision ONLY on DDSM115 wheel bodies; disable everything else.
        # Collision fix applied directly to USD sub-files (see docs/ColectedUSD_v2/SubUSDs/):
        # - DDSM115_A_JFT.usd: approximation → boundingCylinder (smooth rolling, no tread bumping)
        # - All non-wheel bodies: collisionEnabled = False (removes hidden ground-drag friction)

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
        light_cfg.func("/World/Light", light_cfg)

    # ---------------------------------------------------------------------- #
    # Keyboard teleop                                                         #
    # ---------------------------------------------------------------------- #
    def _read_keyboard(self) -> None:
        Ki   = carb.input.KeyboardInput
        held = lambda k: self._input.get_keyboard_value(self._keyboard, k) > 0.0
        dt   = self.cfg.sim.dt * self.cfg.decimation

        w = held(Ki.W);  s = held(Ki.S)
        a = held(Ki.A);  d = held(Ki.D)

        v = self._controller.vel_cmd
        if w and not s:
            v = min(v + FWD_VEL_RAMP * dt,  FWD_VEL_MAX)
        elif s and not w:
            v = max(v - FWD_VEL_RAMP * dt, -FWD_VEL_MAX)
        else:
            v = max(v - FWD_VEL_DECAY * dt, 0.0) if v > 0 else \
                min(v + FWD_VEL_DECAY * dt, 0.0)
        self._controller.vel_cmd = v

        if a and not d:
            self._controller.yaw_rate_cmd =  YAW_RATE_MAX
        elif d and not a:
            self._controller.yaw_rate_cmd = -YAW_RATE_MAX
        else:
            self._controller.yaw_rate_cmd = 0.0

    # ---------------------------------------------------------------------- #
    # Control step                                                            #
    # ---------------------------------------------------------------------- #
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._read_keyboard()

        proj_grav = self.imu.data.projected_gravity_b    # (N, 3)
        ang_vel_b = self.imu.data.ang_vel_b              # (N, 3)
        wheel_vel = self.robot.data.joint_vel[:, self._wheel_ids]  # (N, 2)
        omega_left  = wheel_vel[:, 0]   # Revolute_13
        omega_right = wheel_vel[:, 1]   # Revolute_6

        torque_left, torque_right, cg_angles = self._controller.compute(
            projected_gravity_b=proj_grav,
            ang_vel_b=ang_vel_b,
            omega_left=omega_left,
            omega_right=omega_right,
        )

        # Wheel torques
        efforts = torch.stack([torque_left, torque_right], dim=1)
        self.robot.set_joint_effort_target(efforts, joint_ids=self._wheel_ids)

        # CyberGear position targets — shape (N, 4) [fl, fr, bl, br]
        self.robot.set_joint_position_target(cg_angles, joint_ids=self._cg_ids)

        # ---- Throttled debug print --------------------------------------- #
        if self._debug_print_interval > 0:
            self._debug_step_count += 1
            if self._debug_step_count >= self._debug_print_interval:
                self._debug_step_count = 0

                grav = proj_grav[0]
                av   = ang_vel_b[0]

                tilt_rad  = math.asin(float(grav[IMU_TILT_GRAVITY_AXIS].clamp(-1.0, 1.0)))
                tilt_deg  = math.degrees(tilt_rad)
                tilt_rate = float(av[IMU_TILT_RATE_AXIS])
                yaw_rate  = float(av[IMU_YAW_RATE_AXIS])

                q = self.robot.data.root_quat_w[0]
                yaw_deg = math.degrees(math.atan2(
                    2.0 * (float(q[0]) * float(q[3]) + float(q[1]) * float(q[2])),
                    1.0 - 2.0 * (float(q[2])**2 + float(q[3])**2),
                ))

                fwd_vel = float((-omega_left[0] + omega_right[0]) * 0.5 * WHEEL_RADIUS)
                body_x  = float(self.robot.data.root_pos_w[0, 0])
                body_y  = float(self.robot.data.root_pos_w[0, 1])
                body_z  = float(self.robot.data.root_pos_w[0, 2])
                total_mass = float(self.robot.data.default_mass[0].sum())
                N_est      = total_mass * 9.81   # N — static equilibrium estimate
                print(
                    f"[IMU]  tilt={tilt_deg:+7.2f}°  yaw={yaw_deg:+7.2f}°  "
                    f"tilt_rate={tilt_rate:+6.3f} rad/s  yaw_rate={yaw_rate:+6.3f} rad/s  "
                    f"fwd_vel={fwd_vel:+6.3f} m/s  vel_cmd={self._controller.vel_cmd:+6.3f} m/s  "
                    f"τ_L={float(torque_left[0]):+5.2f} Nm  τ_R={float(torque_right[0]):+5.2f} Nm  "
                    f"body_x={body_x:+8.4f} m  body_y={body_y:+8.4f} m  body_z={body_z:+7.4f} m  "
                    f"mass={total_mass:.2f} kg  N≈{N_est:.1f} N  "
                    f"μN≈{N_est*0.25:.1f} N  "
                    f"CG=[{float(cg_angles[0,0]):+.2f} {float(cg_angles[0,1]):+.2f} "
                    f"{float(cg_angles[0,2]):+.2f} {float(cg_angles[0,3]):+.2f}] rad"
                )

    def _apply_action(self) -> None:
        self.robot.write_data_to_sim()

    # ---------------------------------------------------------------------- #
    # Observations                                                            #
    # ---------------------------------------------------------------------- #
    def _get_observations(self) -> dict:
        proj_grav = self.imu.data.projected_gravity_b
        ang_vel_b = self.imu.data.ang_vel_b
        wheel_vel = self.robot.data.joint_vel[:, self._wheel_ids]

        tilt        = proj_grav[:, IMU_TILT_GRAVITY_AXIS].unsqueeze(1)
        tilt_rate   = ang_vel_b[:, IMU_TILT_RATE_AXIS].unsqueeze(1)
        yaw_rate    = ang_vel_b[:, IMU_YAW_RATE_AXIS].unsqueeze(1)
        forward_vel = ((wheel_vel[:, 0] + wheel_vel[:, 1]) * 0.5
                       * WHEEL_RADIUS).unsqueeze(1)

        obs = torch.cat([tilt, tilt_rate, forward_vel, yaw_rate, wheel_vel], dim=-1)
        return {"policy": obs}  # (N, 6)

    # ---------------------------------------------------------------------- #
    # Rewards / termination                                                   #
    # ---------------------------------------------------------------------- #
    def _get_rewards(self) -> torch.Tensor:
        return -self.imu.data.projected_gravity_b[:, IMU_TILT_GRAVITY_AXIS].abs()

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        # Tilt-based termination disabled for floor-friction diagnostic.
        # Re-enable by un-commenting the lines below.
        # tilt    = self.imu.data.projected_gravity_b[:, IMU_TILT_GRAVITY_AXIS]
        # fallen  = tilt.abs() > math.sin(self._max_pitch)
        fallen  = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        timeout = self.episode_length_buf >= self.max_episode_length - 1
        return fallen, timeout

    # ---------------------------------------------------------------------- #
    # Reset                                                                   #
    # ---------------------------------------------------------------------- #
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)

        self._controller.reset()

        root_state = self.robot.data.default_root_state[env_ids].clone()
        root_state[:, :3] += self.scene.env_origins[env_ids]
        self.robot.write_root_pose_to_sim(root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(root_state[:, 7:], env_ids)

        joint_pos = self.robot.data.default_joint_pos[env_ids].clone()
        joint_vel = self.robot.data.default_joint_vel[env_ids].clone()

        # Initialize CyberGear joints to their upright-stance IK equilibrium.
        # USD defaults are 0; without this, back motors jump from 0 → ±6.08 rad
        # on the first control step, violently tipping the robot before the
        # balance controller can react.
        ik_eq = set_leg_foot_position(L1_C * 0.5, BASE_TARGET_Y)
        if ik_eq is not None:
            lf0, lb0 = ik_eq   # (front_angle, back_angle) for centered foot
            # left side is physically mirrored → negate; right side not negated
            cg_eq = torch.tensor(
                [[-lf0, lf0, -lb0, lb0]],
                device=self.device, dtype=torch.float32,
            ).expand(joint_pos.shape[0], -1)
            joint_pos[:, self._cg_ids] = cg_eq

        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        self.robot.set_joint_position_target(joint_pos)