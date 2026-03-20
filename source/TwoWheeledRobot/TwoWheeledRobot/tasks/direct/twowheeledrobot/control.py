"""
Control theory for the Two-Wheeled Inverted Pendulum robot.
===========================================================

This is the single place to tune everything:
  - Drive gains applied to PhysX joints at simulation start
  - Physical constants (wheel geometry, sensor axis mapping)
  - Controller parameters (LQR gains, PD gains, yaw, torque limits)

The ``RobotController`` class is stateless (no integrators).
It is instantiated once inside the environment.  Each control step you call
``compute()`` with raw sensor data and get back left/right wheel torques.

Sensor inputs available in ``compute()``:
  - ``projected_gravity_b`` — IMU gravity projection (pitch angle proxy)
  - ``ang_vel_b``           — IMU angular velocity (pitch rate, yaw rate)
  - ``omega_left``          — Revolute_13 angular velocity (rad/s)
  - ``omega_right``         — Revolute_6  angular velocity (rad/s)

To run the pure-control sim (no RL):
    python scripts/zero_agent.py --task Template-Twowheeledrobot-Direct-v0
"""

from __future__ import annotations

import torch

# =========================================================================== #
#  Drive gains — applied to PhysX wheel joints at simulation start            #
#  The USD keeps these at 0; Python overrides them here.                      #
# =========================================================================== #

# Wheel joints (Revolute_13 = left, Revolute_6 = right)
# stiffness = 0  → no position spring (pure torque / velocity control)
# damping   > 0  → viscous term that mimics hub-motor back-EMF / friction
WHEEL_DRIVE_STIFFNESS: float = 0.0    # Nm/rad
WHEEL_DRIVE_DAMPING:   float = 0.1    # Nm·s/rad

# CyberGear joints (front_left, front_right, back_left, back_right)
# USD keeps these at 0; Python sets them here at simulation start.
# stiffness > 0  → position spring (position control mode)
# damping   > 0  → viscous damping
CYBERGEAR_STIFFNESS: float = 10    # Nm/rad  — tune for your CyberGear spec
CYBERGEAR_DAMPING:   float = 1   # Nm·s/rad — tune for your CyberGear spec

# =========================================================================== #
#  Physical constants — match your real robot                                 #
# =========================================================================== #

WHEEL_RADIUS: float = 0.0575   # m  — DDSM115: 115 mm diameter → 57.5 mm radius
WHEEL_BASE:   float = 0.30     # m  — lateral distance between wheel contact points

# Which index of projected_gravity_b and ang_vel_b is the forward/backward tilt axis?
# Confirmed from live debug: projected_gravity_b[1] (Y) tracks the robot leaning
# forward/backward (what the BNO080 reports as "roll").
# 0 = X,  1 = Y,  2 = Z
IMU_TILT_GRAVITY_AXIS: int = 1   # index into projected_gravity_b for forward lean
IMU_TILT_RATE_AXIS:    int = 1   # index into ang_vel_b for forward lean rate
IMU_YAW_RATE_AXIS:     int = 2   # index into ang_vel_b for yaw rate

# =========================================================================== #
#  Controller parameters                                                       #
# =========================================================================== #

# Switch between LQR (True) and simple PD fallback (False)
USE_LQR: bool = True

# Maximum torque sent to each wheel (matches DDSM115 peak torque)
TORQUE_LIMIT: float = 2.0   # Nm

# --- Inner loop: 2-state LQR [k_θ_err, k_θ̇] ------------------------------
# State: [tilt_error (rad), tilt_rate (rad/s)]
# tilt_error = actual_tilt − tilt_setpoint (from outer velocity loop)
LQR_K: list[float] = [-55.0, -10.5]

# --- Outer loop: velocity → tilt setpoint (cascade PD) -------------------
# vel_error  = forward_vel − vel_cmd
# tilt_setpoint = KP_VEL * vel_error + KD_VEL * d(vel_error)/dt
# Positive vel_error (too slow) → positive tilt_setpoint (lean forward)
KP_VEL: float = 0.25   # rad/(m/s)  — how much to lean per m/s of error
KD_VEL: float = 0.05   # rad/(m/s²) — damping on velocity change

# --- Fallback PD gains (used when USE_LQR = False) ------------------------
KP_PITCH: float = 60.0   # Nm/rad       — tilt proportional gain
KD_PITCH: float = 8.0    # Nm/(rad/s)   — tilt rate derivative gain

# --- Yaw (turning) controller — differential torque between wheels --------
KP_YAW:       float = 2.0   # Nm/(rad/s) — yaw-rate damping

# --- WASD teleop limits ---------------------------------------------------
FWD_VEL_MAX:   float = 2.0    # m/s    — maximum forward/backward speed
FWD_VEL_RAMP:  float = 2.0    # m/s²   — how fast vel_cmd ramps up while key held
FWD_VEL_DECAY: float = 3.0    # m/s²   — how fast vel_cmd ramps back to 0 on release
YAW_RATE_MAX:  float = 1.0    # rad/s  — A/D yaw rate command


# =========================================================================== #
#  Controller class                                                            #
# =========================================================================== #

class RobotController:
    """
    Stateful balancing controller for the two-wheeled inverted pendulum.

    Instantiate once in the environment ``__init__``, then call
    ``compute()`` every control step.

    Parameters
    ----------
    num_envs : int
        Number of parallel simulation environments.
    device : str
        Torch device string (``"cuda:0"``, ``"cpu"``, …).
    dt : float
        Control timestep in seconds (``sim.dt * decimation``).
    """

    def __init__(self, num_envs: int, device: str, dt: float) -> None:
        self.num_envs = num_envs
        self.device   = device
        self.dt       = dt

        # LQR gain row-vector  (1, 2)
        self._K = torch.tensor(
            LQR_K, device=device, dtype=torch.float32
        ).unsqueeze(0)

        # Previous velocity error — for KD_VEL derivative term
        self._prev_vel_error = torch.zeros(num_envs, device=device)

        # WASD teleop setpoints — updated each step by the env from keyboard input
        self.vel_cmd:      float = 0.0   # m/s    — W/S
        self.yaw_rate_cmd: float = 0.0   # rad/s  — A/D

    # ---------------------------------------------------------------------- #
    def reset(self, env_ids=None) -> None:
        """Reset velocity derivative state."""
        if env_ids is None:
            self._prev_vel_error[:] = 0.0
        else:
            self._prev_vel_error[env_ids] = 0.0

    # ---------------------------------------------------------------------- #
    def compute(
        self,
        projected_gravity_b: torch.Tensor,  # (N, 3)  — from IMU
        ang_vel_b:           torch.Tensor,  # (N, 3)  — from IMU
        omega_left:          torch.Tensor,  # (N,)    — Revolute_13 rad/s
        omega_right:         torch.Tensor,  # (N,)    — Revolute_6  rad/s
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """
        Compute left and right wheel torques for this control step.

        Returns
        -------
        torque_left  : (N,) Nm  — command for Revolute_13
        torque_right : (N,) Nm  — command for Revolute_6
        """
        # ------------------------------------------------------------------ #
        # Extract states from sensors                                         #
        # ------------------------------------------------------------------ #
        # tilt ≈ sin(lean_angle) from gravity projection — confirmed axis 1 (Y)
        pitch      = projected_gravity_b[:, IMU_TILT_GRAVITY_AXIS]  # (N,)
        pitch_rate = ang_vel_b[:, IMU_TILT_RATE_AXIS]               # (N,)  rad/s
        yaw_rate   = ang_vel_b[:, IMU_YAW_RATE_AXIS]                # (N,)  rad/s

        # Forward linear velocity from average wheel angular speed
        forward_vel = (omega_left + omega_right) * 0.5 * WHEEL_RADIUS  # (N,) m/s

        # ------------------------------------------------------------------ #
        # Outer loop — velocity PD → tilt setpoint                          #
        # ------------------------------------------------------------------ #
        vel_error       = forward_vel - self.vel_cmd                 # (N,)
        vel_error_dot   = (vel_error - self._prev_vel_error) / self.dt  # (N,)
        self._prev_vel_error = vel_error.clone()

        tilt_setpoint = KP_VEL * vel_error + KD_VEL * vel_error_dot  # (N,) rad

        # ------------------------------------------------------------------ #
        # Inner loop — 2-state balance                                       #
        # ------------------------------------------------------------------ #
        tilt_error = pitch - tilt_setpoint                           # (N,)

        if USE_LQR:
            state = torch.stack([tilt_error, pitch_rate], dim=1)     # (N, 2)
            base_torque = -(state * self._K).sum(dim=1)              # (N,)
        else:
            base_torque = (
                -KP_PITCH * tilt_error
                - KD_PITCH * pitch_rate
            )  # (N,)

        # ------------------------------------------------------------------ #
        # Yaw correction — A/D sets desired yaw rate                        #
        # ------------------------------------------------------------------ #
        yaw_torque = -KP_YAW * (yaw_rate - self.yaw_rate_cmd)  # (N,)

        # ------------------------------------------------------------------ #
        # Compose, saturate, and return                                       #
        # ------------------------------------------------------------------ #
        # Revolute_13 (left) and Revolute_6 (right) are mirrored in the USD —
        # negate the right wheel so both push the robot in the same direction.
        torque_left  = -(base_torque + yaw_torque).clamp(-TORQUE_LIMIT, TORQUE_LIMIT)
        torque_right = (base_torque - yaw_torque).clamp(-TORQUE_LIMIT, TORQUE_LIMIT)

        return torque_left, torque_right
