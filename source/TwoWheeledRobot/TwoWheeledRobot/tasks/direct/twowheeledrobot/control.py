"""
------------------------------------------------------------
DDSM115 wheels
    Set current_left / current_right in Amperes [A].
    Same as calling DDSM115setCurrent() in the C firmware.
    Range: -8 A … +8 A.  Positive = forward.

CyberGear leg motors (MIT mode)
    Set desired_angle in radians [rad], 0 = upright stance.
    Same as setting motor->desired_angle in the C firmware after setMechanicalZero().
    Set desired_velocity in rad/s and ff_torque in Nm (leave at 0 for now).
    kp / kd are the same numbers as in cybergear.c (hardware range 0–500 / 0–5).
"""

from __future__ import annotations

import torch

from .kinematics import cybergear_stance_angles

# ──────────────────────────────────────────────────────────────────────────────
#  Hardware constants  (do not change — these match the C firmware)
# ──────────────────────────────────────────────────────────────────────────────

DDSM115_KT:   float = 0.75    # Nm/A   — torque constant  (DDSM115.c)
WHEEL_RADIUS: float = 0.0505  # m      — calibrated wheel radius

# IMU axis mapping  (confirmed from live debug)
IMU_TILT_GRAVITY_AXIS: int = 1   # projected_gravity_b[:, i] — forward tilt
IMU_TILT_RATE_AXIS:    int = 1   # ang_vel_b[:, i]           — tilt rate
IMU_YAW_RATE_AXIS:     int = 2   # ang_vel_b[:, i]           — yaw rate

# ──────────────────────────────────────────────────────────────────────────────
#  Isaac actuator gains  (written to PhysX at sim start via robot_cfg.py)
# ──────────────────────────────────────────────────────────────────────────────

WHEEL_DRIVE_STIFFNESS: float = 0.0    # Nm/rad  — pure torque control

# CyberGear MIT gains — same values as cybergear.c CyberGearMotorList init
# kp range 0–500 Nm/rad,  kd range 0–5 Nm·s/rad
CYBERGEAR_STIFFNESS: float = 5.0   # Nm/rad    (kp)
CYBERGEAR_DAMPING:   float = 0.5    # Nm·s/rad  (kd)

# ──────────────────────────────────────────────────────────────────────────────
#  WASD teleop limits
# ──────────────────────────────────────────────────────────────────────────────

FWD_VEL_MAX:   float = 1.0   # m/s
FWD_VEL_RAMP:  float = 0.4   # m/s²
FWD_VEL_DECAY: float = 1.0   # m/s²
YAW_RATE_MAX:  float = 1.0   # rad/s


# ──────────────────────────────────────────────────────────────────────────────
#  Controller
# ──────────────────────────────────────────────────────────────────────────────

class RobotController:

    def __init__(self, num_envs: int, device: str, dt: float) -> None:
        self.num_envs = num_envs
        self.device   = device
        self.dt       = dt

        # Written by the env from keyboard input each step
        self.vel_cmd:      float = 0.0   # m/s
        self.yaw_rate_cmd: float = 0.0   # rad/s

        # Read by the plotter each step
        self.last: dict = {}

        # Equilibrium offsets — keeps desired_angle=0 meaning "upright stance"
        # in Isaac just like it does on hardware after setMechanicalZero().
        self._cg_stance = cybergear_stance_angles(num_envs, device)  # (N, 4)

    def reset(self, env_ids=None) -> None:
        pass   # add integrator resets here when you port algorithms

    def compute(
        self,
        projected_gravity_b: torch.Tensor,   # (N, 3) — gravity vector in body frame
        ang_vel_b:           torch.Tensor,   # (N, 3) — body angular velocity
        omega_left:          torch.Tensor,   # (N,)   — left  wheel speed  [rad/s]
        omega_right:         torch.Tensor,   # (N,)   — right wheel speed  [rad/s]
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:

        # ┌─────────────────────────────────────────────────────────────────┐
        # │                YOUR CONTROL ALGORITHM HERE                      │
        # └─────────────────────────────────────────────────────────────────┘

        # DDSM115 — set current [A], same as DDSM115setCurrent() in C
        current_left:  float = 0.0   # A
        current_right: float = 0.0   # A

        # CyberGear MIT — set desired state, same as Motor_SendMITCommand() in C
        desired_angle:    float = 0.0   # rad   (0 = upright stance)
        desired_velocity: float = 0.0   # rad/s
        ff_torque:        float = 0.0   # Nm

        # ┌─────────────────────────────────────────────────────────────────┐
        # │               TRANSLATION LAYER — do not edit                   │
        # └─────────────────────────────────────────────────────────────────┘

        # DDSM115: I [A] × Kt [Nm/A] = τ [Nm]
        # Left wheel USD mesh is mirrored — negate so positive current = forward.
        tl = torch.full((self.num_envs,),  current_left  * DDSM115_KT, device=self.device, dtype=torch.float32)
        tr = torch.full((self.num_envs,), -current_right * DDSM115_KT, device=self.device, dtype=torch.float32)

        # CyberGear: add stance offset so desired_angle=0 means equilibrium in Isaac.
        # PhysX applies: τ = kp·(cg_pos − θ) + kd·(cg_vel − ω) + cg_ff
        _z4    = torch.zeros((self.num_envs, 4), device=self.device, dtype=torch.float32)
        cg_pos = self._cg_stance + desired_angle
        cg_vel = _z4 + desired_velocity
        cg_ff  = _z4 + ff_torque

        self.last = {
            "torque_L": float(tl[0]),
            "torque_R": float(tr[0]),
            "vel_cmd":  self.vel_cmd,
        }

        return tl, tr, cg_pos, cg_vel, cg_ff
