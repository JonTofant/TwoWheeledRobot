"""
Control theory for the Two-Wheeled Inverted Pendulum robot.
===========================================================

This is the single place to tune everything:
  - Drive gains applied to PhysX joints at simulation start
  - Physical constants (wheel geometry, sensor axis mapping)
  - Controller parameters (LQR gains, PD gains, yaw, torque limits)

The ``RobotController`` class is stateful (holds the odometry integrator)
and is instantiated once inside the environment.  Each control step you call
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
CYBERGEAR_STIFFNESS: float = 0.0    # Nm/rad  — tune for your CyberGear spec
CYBERGEAR_DAMPING:   float = 0.0    # Nm·s/rad — tune for your CyberGear spec

# =========================================================================== #
#  Physical constants — match your real robot                                 #
# =========================================================================== #

WHEEL_RADIUS: float = 0.0575   # m  — DDSM115: 115 mm diameter → 57.5 mm radius
WHEEL_BASE:   float = 0.30     # m  — lateral distance between wheel contact points

# Which index of ang_vel_b is the pitch-rate axis?
# 0 = X,  1 = Y,  2 = Z
# For a robot whose wheels roll along the body X-axis, pitch rotates about
# body Y → use index 1.
IMU_PITCH_AXIS: int = 1

# =========================================================================== #
#  Controller parameters                                                       #
# =========================================================================== #

# Switch between LQR (True) and simple PD fallback (False)
USE_LQR: bool = True

# Maximum torque sent to each wheel (matches DDSM115 peak torque)
TORQUE_LIMIT: float = 2.0   # Nm

# --- LQR gains [k_θ, k_θ̇, k_v, k_pos] -----------------------------------
# State: [pitch_angle (rad), pitch_rate (rad/s), forward_vel (m/s), position (m)]
# Computed offline by running: python compute_lqr_gains.py
# Tune these for your specific robot mass / CoM height.
LQR_K: list[float] = [-55.0, -10.5, -4.2, -3.0]

# --- Fallback PD gains (used when USE_LQR = False) ------------------------
KP_PITCH: float = 60.0   # Nm/rad       — pitch angle proportional gain
KD_PITCH: float = 8.0    # Nm/(rad/s)   — pitch rate derivative gain
KP_VEL:   float = 5.0    # Nm/(m/s)     — forward velocity feedback

# --- Yaw (turning) controller — differential torque between wheels --------
KP_YAW:       float = 2.0   # Nm/(rad/s) — yaw-rate damping
YAW_SETPOINT: float = 0.0   # rad/s      — commanded yaw rate (0 = drive straight)


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

        # Wheel-odometry position integrator — one scalar per environment
        self._pos_estimate = torch.zeros(num_envs, device=device)

        # LQR gain row-vector  (1, 4)
        self._K = torch.tensor(
            LQR_K, device=device, dtype=torch.float32
        ).unsqueeze(0)

    # ---------------------------------------------------------------------- #
    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        """Zero the odometry integrator for the given environments."""
        if env_ids is None:
            self._pos_estimate[:] = 0.0
        else:
            self._pos_estimate[env_ids] = 0.0

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
        # pitch ≈ sin(pitch_angle) from gravity projection (X component)
        pitch      = projected_gravity_b[:, 0]           # (N,)
        pitch_rate = ang_vel_b[:, IMU_PITCH_AXIS]         # (N,)  rad/s
        yaw_rate   = ang_vel_b[:, 2]                      # (N,)  rad/s

        # Forward linear velocity from average wheel angular speed
        forward_vel = (omega_left + omega_right) * 0.5 * WHEEL_RADIUS  # (N,) m/s

        # Integrate position (wheel odometry)
        self._pos_estimate = self._pos_estimate + forward_vel * self.dt  # (N,)

        # ------------------------------------------------------------------ #
        # Balancing torque                                                    #
        # ------------------------------------------------------------------ #
        if USE_LQR:
            state = torch.stack(
                [pitch, pitch_rate, forward_vel, self._pos_estimate], dim=1
            )  # (N, 4)
            # u = -K @ x  →  element-wise multiply then sum over state dim
            base_torque = -(state * self._K).sum(dim=1)   # (N,)
        else:
            base_torque = (
                -KP_PITCH * pitch
                - KD_PITCH * pitch_rate
                - KP_VEL   * forward_vel
            )  # (N,)

        # ------------------------------------------------------------------ #
        # Yaw correction — differential torque for steering                  #
        # ------------------------------------------------------------------ #
        yaw_torque = -KP_YAW * (yaw_rate - YAW_SETPOINT)  # (N,)

        # ------------------------------------------------------------------ #
        # Compose, saturate, and return                                       #
        # ------------------------------------------------------------------ #
        torque_left  = (base_torque + yaw_torque).clamp(-TORQUE_LIMIT, TORQUE_LIMIT)
        torque_right = (base_torque - yaw_torque).clamp(-TORQUE_LIMIT, TORQUE_LIMIT)

        return torque_left, torque_right
