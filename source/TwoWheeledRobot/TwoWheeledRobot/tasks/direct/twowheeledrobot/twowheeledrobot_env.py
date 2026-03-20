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

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sensors import Imu
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.sim.spawners.materials import RigidBodyMaterialCfg

from .twowheeledrobot_env_cfg import TwowheeledrobotEnvCfg
from .control import RobotController


class TwowheeledrobotEnv(DirectRLEnv):
    cfg: TwowheeledrobotEnvCfg

    def __init__(self, cfg: TwowheeledrobotEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # ---- Print all joints present in the USD (useful for verification) #
        _, all_joint_names = self.robot.find_joints(".*")
        print(f"[TwoWheeledRobot] All joints in USD: {all_joint_names}")

        # ---- Resolve wheel joint indices --------------------------------- #
        self._left_wheel_ids,  _ = self.robot.find_joints("Revolute_13")
        self._right_wheel_ids, _ = self.robot.find_joints("Revolute_6")
        self._wheel_ids = torch.tensor(
            [self._left_wheel_ids[0], self._right_wheel_ids[0]],
            device=self.device, dtype=torch.long,
        )

        # ---- Control timestep -------------------------------------------- #
        dt = self.cfg.sim.dt * self.cfg.decimation   # seconds per control step

        # ---- Instantiate controller (all tuning is inside control.py) ---- #
        self._controller = RobotController(
            num_envs=self.num_envs,
            device=self.device,
            dt=dt,
        )

        # ---- Tilt limit -------------------------------------------------- #
        self._max_pitch = math.radians(self.cfg.max_pitch_deg)

        # ---- Debug print throttle (print every N control steps) ---------- #
        self._debug_print_interval: int = 50   # change to 0 to disable
        self._debug_step_count: int = 0

        print(f"[TwoWheeledRobot] Wheel joint IDs — Left: {self._left_wheel_ids}, "
              f"Right: {self._right_wheel_ids}")
        print(f"[TwoWheeledRobot] Control dt = {dt:.4f} s")

    # ---------------------------------------------------------------------- #
    # Scene setup                                                             #
    # ---------------------------------------------------------------------- #
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        self.scene.articulations["robot"] = self.robot

        # IMU mounted on Platform_Group inside the robot assembly
        self.imu = Imu(self.cfg.imu)
        self.scene.sensors["imu"] = self.imu

        # Ground plane — high friction for DDSM115 rubber wheels on hard floor
        spawn_ground_plane(
            prim_path="/World/ground",
            cfg=GroundPlaneCfg(
                physics_material=RigidBodyMaterialCfg(
                    static_friction=0.8,
                    dynamic_friction=0.7,
                    restitution=0.0,
                )
            ),
        )

        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=["/World/ground"])

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
        light_cfg.func("/World/Light", light_cfg)

    # ---------------------------------------------------------------------- #
    # Control step                                                            #
    # ---------------------------------------------------------------------- #
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        """Read sensors, call control.py, apply wheel torques."""

        # Raw sensor data
        proj_grav = self.imu.data.projected_gravity_b   # (N, 3)
        ang_vel_b = self.imu.data.ang_vel_b              # (N, 3)

        wheel_vel    = self.robot.data.joint_vel[:, self._wheel_ids]  # (N, 2) rad/s
        omega_left   = wheel_vel[:, 0]   # Revolute_13
        omega_right  = wheel_vel[:, 1]   # Revolute_6

        # Delegate to the controller in control.py
        torque_left, torque_right = self._controller.compute(
            projected_gravity_b=proj_grav,
            ang_vel_b=ang_vel_b,
            omega_left=omega_left,
            omega_right=omega_right,
        )

        # Write effort targets: shape (N, 2) — [Revolute_13, Revolute_6]
        efforts = torch.stack([torque_left, torque_right], dim=1)
        self.robot.set_joint_effort_target(efforts, joint_ids=self._wheel_ids)

        # ---- Debug printout (env 0 only, throttled) ---------------------- #
        if self._debug_print_interval > 0:
            self._debug_step_count += 1
            if self._debug_step_count >= self._debug_print_interval:
                self._debug_step_count = 0
                from .control import IMU_PITCH_AXIS, WHEEL_RADIUS
                grav   = proj_grav[0]           # (3,)
                av     = ang_vel_b[0]           # (3,)
                # projected_gravity_b is a unit vector pointing toward gravity
                # in body frame.  pitch ≈ arcsin(grav[0]) for small angles.
                pitch_rad  = math.asin(float(grav[0].clamp(-1.0, 1.0)))
                pitch_deg  = math.degrees(pitch_rad)
                roll_rad   = math.asin(float(grav[1].clamp(-1.0, 1.0)))
                roll_deg   = math.degrees(roll_rad)
                pitch_rate = float(av[IMU_PITCH_AXIS])
                yaw_rate   = float(av[2])
                fwd_vel    = float((omega_left[0] + omega_right[0]) * 0.5 * WHEEL_RADIUS)
                print(
                    f"[IMU]  pitch={pitch_deg:+7.2f}°  roll={roll_deg:+7.2f}°  "
                    f"pitch_rate={pitch_rate:+6.3f} rad/s  yaw_rate={yaw_rate:+6.3f} rad/s  "
                    f"fwd_vel={fwd_vel:+6.3f} m/s  "
                    f"τ_L={float(torque_left[0]):+5.2f} Nm  τ_R={float(torque_right[0]):+5.2f} Nm"
                )

    def _apply_action(self) -> None:
        self.robot.write_data_to_sim()

    # ---------------------------------------------------------------------- #
    # Observations                                                            #
    # ---------------------------------------------------------------------- #
    def _get_observations(self) -> dict:
        proj_grav = self.imu.data.projected_gravity_b    # (N, 3)
        ang_vel_b = self.imu.data.ang_vel_b              # (N, 3)
        wheel_vel = self.robot.data.joint_vel[:, self._wheel_ids]  # (N, 2)

        from .control import IMU_PITCH_AXIS, WHEEL_RADIUS

        pitch       = proj_grav[:, 0].unsqueeze(1)                              # (N,1)
        pitch_rate  = ang_vel_b[:, IMU_PITCH_AXIS].unsqueeze(1)                 # (N,1)
        yaw_rate    = ang_vel_b[:, 2].unsqueeze(1)                              # (N,1)
        forward_vel = ((wheel_vel[:, 0] + wheel_vel[:, 1]) * 0.5
                       * WHEEL_RADIUS).unsqueeze(1)                             # (N,1)
        position    = self._controller._pos_estimate.unsqueeze(1)               # (N,1)

        obs = torch.cat(
            [pitch, pitch_rate, forward_vel, position, yaw_rate, wheel_vel],
            dim=-1,
        )  # (N, 7)  [pitch, pitch_rate, fwd_vel, pos, yaw_rate, ω_L, ω_R]

        return {"policy": obs}

    # ---------------------------------------------------------------------- #
    # Rewards (minimal — this env is a controller, not a trainer)            #
    # ---------------------------------------------------------------------- #
    def _get_rewards(self) -> torch.Tensor:
        pitch = self.imu.data.projected_gravity_b[:, 0]
        return -pitch.abs()

    # ---------------------------------------------------------------------- #
    # Episode termination                                                     #
    # ---------------------------------------------------------------------- #
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        pitch   = self.imu.data.projected_gravity_b[:, 0]
        fallen  = pitch.abs() > math.sin(self._max_pitch)
        timeout = self.episode_length_buf >= self.max_episode_length - 1
        return fallen, timeout

    # ---------------------------------------------------------------------- #
    # Reset                                                                   #
    # ---------------------------------------------------------------------- #
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)

        # Reset odometry integrator inside the controller
        self._controller.reset(torch.tensor(env_ids, device=self.device)
                               if not isinstance(env_ids, torch.Tensor)
                               else env_ids)

        # Restore robot to its default root pose and joint state
        root_state = self.robot.data.default_root_state[env_ids].clone()
        root_state[:, :3] += self.scene.env_origins[env_ids]
        self.robot.write_root_pose_to_sim(root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(root_state[:, 7:], env_ids)

        joint_pos = self.robot.data.default_joint_pos[env_ids].clone()
        joint_vel = self.robot.data.default_joint_vel[env_ids].clone()
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        self.robot.set_joint_position_target(joint_pos)
