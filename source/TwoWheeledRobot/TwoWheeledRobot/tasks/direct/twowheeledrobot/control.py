"""
Control theory for the Two-Wheeled Inverted Pendulum robot.
===========================================================

Direct port of the real STM32 implementation (controler.c / kinematics.c).

Architecture — two cascaded loops, both running at 50 Hz (20 ms):

  Outer loop (velocity → desired tilt angle):
      x_err        = wheel_velocity - desired_velocity
      theta_des    = -(Kp_pos*x_err + Kd_pos*x_ddot + Ki_pos*integral)
      |theta_des|  ≤ MAX_THETA_DES (≈ 14°)

  Inner loop LQI (tilt error → wheel torque):
      theta_error  = (tilt + delta_varphi) - theta_des
      force        = -(K_THETA*theta_error + K_THETA_DOT*tilt_rate
                       + K_I_THETA*theta_integral)
      torque       = force * WHEEL_RADIUS

  Posture controller (CyberGear leg IK — runs in parallel):
      xc_des  ← velocity PID per side
      IK      → physical motor angles for front/back motors
      delta_varphi ← virtual leg angle (tilt compensation)

Tune everything in the constant blocks below.
Run:  python scripts/zero_agent.py --task Template-Twowheeledrobot-Direct-v0
"""

from __future__ import annotations

import math
import torch

# =========================================================================== #
#  Drive gains — applied to PhysX joints at simulation start                  #
# =========================================================================== #
WHEEL_DRIVE_STIFFNESS: float = 0.0    # Nm/rad  — pure torque control
WHEEL_DRIVE_DAMPING:   float = 0.01  # Nm·s/rad — minimum to keep PhysX effort target active; was 0.02 which created 15.7v N of drag

CYBERGEAR_STIFFNESS: float = 10.0    # Nm/rad  — position spring
CYBERGEAR_DAMPING:   float = 1.0     # Nm·s/rad

# =========================================================================== #
#  Physical constants                                                          #
# =========================================================================== #
WHEEL_RADIUS: float = 0.0505   # m  — real DDSM115 (115 mm ⌀ → 57.5 mm r, but
                                #      real calibrated value is 50.5 mm)
WHEEL_BASE:   float = 0.30     # m  — lateral distance between wheel contacts

# IMU axis mapping (confirmed from live debug: axis 1 = forward/backward tilt)
IMU_TILT_GRAVITY_AXIS: int = 1   # projected_gravity_b index for forward lean
IMU_TILT_RATE_AXIS:    int = 1   # ang_vel_b index for tilt rate
IMU_YAW_RATE_AXIS:     int = 2   # ang_vel_b index for yaw rate

# IMU calibration offset (real robot: 0.0280328498 rad ≈ 1.6°)
# Simulation is perfect so default is 0. Tune if the robot has a lean bias.
ROLL_OFFSET: float = 0.0   # rad

# =========================================================================== #
#  Leg / IK geometry  (units: cm — same as real kinematics.c)                 #
# =========================================================================== #
L1_C: float = 10.8    # base link width (distance between pivot points)
L2_C: float = 10.0    # upper leg link length
L3_C: float = 20.0    # lower leg link length
BASE_TARGET_Y: float  = 16.0   # cm — default foot height below body
                                 # (negative = below robot)

# =========================================================================== #
#  Inner-loop gains (LQI balance — from controler.c K_GAINS)                  #
# =========================================================================== #
K_THETA:     float = 100.0   # tilt error gain       [Nm/rad  × 1/r]
K_THETA_DOT: float = 10.0    # tilt rate gain        [Nm·s/rad × 1/r]
K_I_THETA:   float = -18.0     # tilt integral gain — enable once balance confirmed (-18.0)
MAX_THETA_I: float = 0.2     # integral anti-windup  [rad·s]

# =========================================================================== #
#  Outer-loop gains (velocity → desired tilt — from controler.c)              #
# =========================================================================== #
KP_POS:        float = 0.1        # rad / (m/s)   — real=0.13, reduced for sim
KD_POS:        float = 0.0          # rad / (m/s²)  — real=-0.015 (tune after balance ok)
KI_POS:        float = 0.1            # outer integral — enable once balance confirmed (0.25)
MAX_POS_I:     float = 0.2            # anti-windup  [m/s · s]
MAX_THETA_DES: float = 0.244           # ~6° max lean  — real=0.244 (14°), softer for sim

# =========================================================================== #
#  Posture controller gains (CyberGear foot placement — from controler.c)     #
# =========================================================================== #
KP_CHASIS: float = 0.0     # real=1.2; reduced for sim
KD_CHASIS: float = 0.0
KI_CHASIS: float = 0.0
MAX_CHASIS_I:   float = 0.2
MAX_CHASIS_DES: float = 1.0   # cm — foot x-offset clamp

# =========================================================================== #
#  Actuator limits                                                             #
# =========================================================================== #
TORQUE_LIMIT: float = 6.0   # Nm — DDSM115 peak

# =========================================================================== #
#  WASD teleop                                                                 #
# =========================================================================== #
FWD_VEL_MAX:   float = 1.0   # m/s
FWD_VEL_RAMP:  float = 0.4   # m/s²  — ramp-up rate while key held
FWD_VEL_DECAY: float = 1.0   # m/s²  — ramp-down when key released
YAW_RATE_MAX:  float = 1.0   # rad/s


# =========================================================================== #
#  Inverse kinematics (ported from kinematics.c)                              #
# =========================================================================== #

def _solve_ik_two_solutions(
    base_x: float, base_y: float,
    foot_x: float, foot_y: float,
    L_upper: float, L_lower: float,
) -> tuple[tuple[float, float], tuple[float, float]] | None:
    """Two-link IK: returns two (x, y) knee positions, or None if unreachable."""
    dx   = foot_x - base_x
    dy   = foot_y - base_y
    d_sq = dx * dx + dy * dy
    d    = math.sqrt(d_sq)

    if d > (L_upper + L_lower) or d < abs(L_upper - L_lower):
        return None

    a    = (L_upper * L_upper - L_lower * L_lower + d_sq) / (2.0 * d)
    h    = math.sqrt(max(L_upper * L_upper - a * a, 0.0))
    mx   = base_x + a * dx / d
    my   = base_y + a * dy / d
    rx   = -dy * (h / d)
    ry   =  dx * (h / d)

    return (mx + rx, my + ry), (mx - rx, my - ry)


def set_leg_foot_position(xf: float, yf: float) -> tuple[float, float] | None:
    """
    IK for one leg pair given desired foot position (xf, yf) in cm.

    Returns (physical_angle_front, physical_angle_back):
      physical_angle_front → front motor of this leg (LF or RF)
      physical_angle_back  → back  motor of this leg (LB or RB)

    Returns None if the position is unreachable.
    """
    # Pivot points: O0 = left pivot (origin), A = right pivot
    O0_x, O0_y = 0.0, 0.0
    A_x,  A_y  = L1_C, 0.0

    sol_B = _solve_ik_two_solutions(A_x, A_y,   xf, yf, L2_C, L3_C)
    sol_C = _solve_ik_two_solutions(O0_x, O0_y, xf, yf, L2_C, L3_C)

    if sol_B is None or sol_C is None:
        return None

    (B_x1, B_y1), (B_x2, B_y2) = sol_B
    (C_x1, C_y1), (C_x2, C_y2) = sol_C

    # Right knee → larger x (outward elbow), left knee → smaller x (inward)
    if B_x1 >= B_x2:
        cB_x, cB_y = B_x1, B_y1
    else:
        cB_x, cB_y = B_x2, B_y2

    if C_x1 <= C_x2:
        cC_x, cC_y = C_x1, C_y1
    else:
        cC_x, cC_y = C_x2, C_y2

    alpha1 = math.atan2(cB_y - A_y,   cB_x - A_x)
    alpha2 = math.atan2(cC_y - O0_y,  cC_x - O0_x)

    physical_angle_front = -alpha1
    physical_angle_back  = -(alpha2 + math.pi)

    return physical_angle_front, physical_angle_back


def _calculate_leg_theta(xc: float, yc_positive: float) -> float:
    """
    Virtual leg angle from vertical.
    yc_positive = -BASE_TARGET_Y (a positive number, e.g. 15.0 cm).
    Returns theta in radians (used as delta_varphi correction).
    """
    return math.atan((xc - 0.5 * L1_C) / yc_positive)


# =========================================================================== #
#  Helpers                                                                     #
# =========================================================================== #

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# =========================================================================== #
#  Controller                                                                  #
# =========================================================================== #

class RobotController:
    """
    Full cascaded LQI + IK controller, ported from the real STM32 firmware.

    Call ``compute()`` every control step (20 ms).
    All tuning lives in the constant blocks above.
    """

    def __init__(self, num_envs: int, device: str, dt: float) -> None:
        self.num_envs = num_envs
        self.device   = device
        self.dt       = dt

        # Outer-loop (velocity → theta_des) integrators — separate L / R
        self._pos_i_L: float = 0.0
        self._pos_i_R: float = 0.0

        # Inner-loop (LQI tilt) integrators — separate L / R
        self._theta_i_L: float = 0.0
        self._theta_i_R: float = 0.0

        # Posture (chassis / IK) integrators — separate L / R
        self._chasis_i_L: float = 0.0
        self._chasis_i_R: float = 0.0

        # Previous wheel velocities for acceleration estimate
        self._prev_xdot_L: float = 0.0
        self._prev_xdot_R: float = 0.0

        # Low-pass filtered velocities fed to the outer loop.
        # Raw wheel odometry has left/right asymmetry noise that causes theta_des
        # to oscillate, preventing the lean from building.  α = 0.2 gives a
        # ~100 ms time constant at 50 Hz (real robot uses same filter).
        self._vel_filt_L: float = 0.0
        self._vel_filt_R: float = 0.0

        # Virtual leg tilt offsets (updated every step by posture controller)
        self._delta_varphi_l: float = 0.0
        self._delta_varphi_r: float = 0.0

        # WASD setpoints — written by the env each step
        self.vel_cmd:      float = 0.0   # m/s   — W / S
        self.yaw_rate_cmd: float = 0.0   # rad/s — A / D

    # ---------------------------------------------------------------------- #
    def reset(self, env_ids=None) -> None:
        """Reset all integrators (called on episode reset)."""
        self._pos_i_L   = self._pos_i_R   = 0.0
        self._theta_i_L = self._theta_i_R = 0.0
        self._chasis_i_L = self._chasis_i_R = 0.0
        self._prev_xdot_L = self._prev_xdot_R = 0.0
        self._vel_filt_L  = self._vel_filt_R  = 0.0

    # ---------------------------------------------------------------------- #
    def compute(
        self,
        projected_gravity_b: torch.Tensor,  # (N, 3)
        ang_vel_b:           torch.Tensor,  # (N, 3)
        omega_left:          torch.Tensor,  # (N,)   — Revolute_13 rad/s
        omega_right:         torch.Tensor,  # (N,)   — Revolute_6  rad/s
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Returns
        -------
        torque_left  : (N,)   Nm — command for Revolute_13
        torque_right : (N,)   Nm — command for Revolute_6
        cg_angles    : (N, 4) rad — [front_left, front_right, back_left, back_right]
        """
        dt = self.dt

        # ================================================================== #
        # FLOOR-FRICTION DIAGNOSTIC — set DIAG_TORQUE_TEST = True to bypass  #
        # the full controller and apply a constant wheel torque.              #
        # The robot will fall, but it MUST translate first if the wheels work.#
        # Expected: robot slides forward, falls ~1–2 s later.                #
        # If it stays glued and just tips over in pli caace → floor friction bug. #
        # ================================================================== #
        DIAG_TORQUE_TEST: bool = False         # ← set True to activate
        DIAG_TORQUE_NM:   float = 0.5           # start small; increase if no motion
        if DIAG_TORQUE_TEST:
            tl  = torch.full((self.num_envs,),  DIAG_TORQUE_NM,
                             device=self.device, dtype=torch.float32)
            tr  = torch.full((self.num_envs,), -DIAG_TORQUE_NM,
                             device=self.device, dtype=torch.float32)
            # Hold legs at their current equilibrium so the robot doesn't collapse
            # before we can observe wheel motion.
            cg  = torch.tensor([[0.20, -0.20, 6.08, -6.08]],
                               device=self.device, dtype=torch.float32)
            return tl, tr, cg

        # ------------------------------------------------------------------ #
        # Sensor extraction (env 0; only scalars needed — single env)        #
        # ------------------------------------------------------------------ #
        tilt      = float(projected_gravity_b[0, IMU_TILT_GRAVITY_AXIS]) - ROLL_OFFSET
        tilt_rate = float(ang_vel_b[0, IMU_TILT_RATE_AXIS])

        # Convert wheel angular velocity to linear velocity.
        # Revolute_13 (left) is mirrored in the USD: its joint angle convention
        # is inverted relative to the physical forward direction.  Negate it so
        # both xdot_L and xdot_R are positive when the robot moves forward.
        # Without this, the outer loop sees xdot_L ≈ -xdot_R, fwd_vel ≈ 0, and
        # the two independent loops fight each other — preventing all translation.
        xdot_L = -float(omega_left[0])  * WHEEL_RADIUS   # m/s (left, mirror-corrected)
        xdot_R =  float(omega_right[0]) * WHEEL_RADIUS   # m/s (right)

        # Finite-difference wheel acceleration
        xddot_L = (xdot_L - self._prev_xdot_L) / dt
        xddot_R = (xdot_R - self._prev_xdot_R) / dt
        self._prev_xdot_L = xdot_L
        self._prev_xdot_R = xdot_R

        # Low-pass filter velocity for outer loop (α=0.2 → ~100 ms time constant).
        # Raw odometry has L/R asymmetry noise; filtering prevents theta_des from
        # thrashing and allows the lean to build to the actual target angle.
        _VEL_ALPHA = 0.2
        self._vel_filt_L = _VEL_ALPHA * xdot_L + (1.0 - _VEL_ALPHA) * self._vel_filt_L
        self._vel_filt_R = _VEL_ALPHA * xdot_R + (1.0 - _VEL_ALPHA) * self._vel_filt_R

        # Desired wheel velocities from WASD (differential drive)
        v_left  = self.vel_cmd - self.yaw_rate_cmd * WHEEL_BASE * 0.5
        v_right = self.vel_cmd + self.yaw_rate_cmd * WHEEL_BASE * 0.5

        # ================================================================== #
        # POSTURE CONTROLLER — CyberGear foot placement + delta_varphi       #
        # (matches posture_controler() in controler.c)                       #
        # ================================================================== #
        perr_L = xdot_L - v_left
        perr_R = xdot_R - v_right

        self._chasis_i_L = _clamp(self._chasis_i_L + perr_L * dt,
                                  -MAX_CHASIS_I, MAX_CHASIS_I)
        self._chasis_i_R = _clamp(self._chasis_i_R + perr_R * dt,
                                  -MAX_CHASIS_I, MAX_CHASIS_I)

        # Note: real code has L/R ddot cross-swapped; replicated faithfully
        xc_des_l = -(KP_CHASIS * perr_L + KD_CHASIS * xddot_R
                     + KI_CHASIS * self._chasis_i_L)
        xc_des_r =  (KP_CHASIS * perr_R + KD_CHASIS * xddot_L
                     + KI_CHASIS * self._chasis_i_R)

        xc_des_l = _clamp(xc_des_l, -MAX_CHASIS_DES, MAX_CHASIS_DES)
        xc_des_r = _clamp(xc_des_r, -MAX_CHASIS_DES, MAX_CHASIS_DES)

        body_disp_cm = 0.0   # CyberGear targets are body-relative; no correction needed

        xc_l = xc_des_l + L1_C * 0.5 - body_disp_cm
        xc_r = xc_des_r + L1_C * 0.5 - body_disp_cm

        # IK for left leg (LF = front motor, LB = back motor)
        ik_l = set_leg_foot_position(xc_l, BASE_TARGET_Y)
        lf_angle = ik_l[0] if ik_l else 0.0
        lb_angle = ik_l[1] if ik_l else 0.0

        # IK for right leg (RF = front motor, RB = back motor)
        ik_r = set_leg_foot_position(xc_r, BASE_TARGET_Y)
        rf_angle = ik_r[0] if ik_r else 0.0
        rb_angle = ik_r[1] if ik_r else 0.0

        # Virtual leg angle → tilt compensation (delta_varphi)
        yc_pos = -BASE_TARGET_Y   # 15.0 cm
        theta_l = _calculate_leg_theta(xc_l, yc_pos)
        theta_r = _calculate_leg_theta(xc_r, yc_pos)
        self._delta_varphi_l = -theta_l
        self._delta_varphi_r =  theta_r

        # ================================================================== #
        # OUTER LOOP — velocity error → desired tilt angle                   #
        # (matches calculate_cascaded_motor_currents outer loop)             #
        # ================================================================== #
        x_err_L = self._vel_filt_L - v_left
        x_err_R = self._vel_filt_R - v_right

        self._pos_i_L = _clamp(self._pos_i_L + x_err_L * dt,
                               -MAX_POS_I, MAX_POS_I)
        self._pos_i_R = _clamp(self._pos_i_R + x_err_R * dt,
                               -MAX_POS_I, MAX_POS_I)

        theta_des_L = -(KP_POS * x_err_L + KD_POS * xddot_L
                        + KI_POS * self._pos_i_L)
        theta_des_R = -(KP_POS * x_err_R + KD_POS * xddot_R
                        + KI_POS * self._pos_i_R)

        theta_des_L = _clamp(theta_des_L, -MAX_THETA_DES, MAX_THETA_DES)
        theta_des_R = _clamp(theta_des_R, -MAX_THETA_DES, MAX_THETA_DES)

        # ================================================================== #
        # INNER LOOP — LQI balance                                           #
        # (matches calculate_cascaded_motor_currents inner loop)             #
        # ================================================================== #
        theta_err_L = (tilt + self._delta_varphi_l) - theta_des_L
        theta_err_R = (tilt + self._delta_varphi_r) - theta_des_R

        self._theta_i_L = _clamp(self._theta_i_L + theta_err_L * dt,
                                  -MAX_THETA_I, MAX_THETA_I)
        self._theta_i_R = _clamp(self._theta_i_R + theta_err_R * dt,
                                  -MAX_THETA_I, MAX_THETA_I)

        force_L = -(K_THETA * theta_err_L
                    + K_THETA_DOT * tilt_rate
                    + K_I_THETA   * self._theta_i_L)
        force_R = -(K_THETA * theta_err_R
                    + K_THETA_DOT * tilt_rate
                    + K_I_THETA   * self._theta_i_R)

        # In sim we command torque directly (no motor current conversion)
        torque_L = force_L * WHEEL_RADIUS
        torque_R = force_R * WHEEL_RADIUS

        # ================================================================== #
        # Sign convention                                                     #
        # Real C code negates the right motor output.                        #
        # In this USD the LEFT wheel (Revolute_13) is the mirrored one —     #
        # confirmed from the old working simulation — so we negate left.     #
        # ================================================================== #
        tl = torch.full((self.num_envs,),  torque_L,
                        device=self.device, dtype=torch.float32)
        tr = torch.full((self.num_envs,), -torque_R,
                        device=self.device, dtype=torch.float32)

        tl = tl.clamp(-TORQUE_LIMIT, TORQUE_LIMIT)
        tr = tr.clamp(-TORQUE_LIMIT, TORQUE_LIMIT)

        # CyberGear angles: (N, 4) — [front_left, front_right, back_left, back_right]
        # Left-side motors are physically mirrored → negate their angles
        cg = torch.tensor(
            [[-lf_angle, rf_angle, -lb_angle, rb_angle]],
            device=self.device, dtype=torch.float32,
        ).expand(self.num_envs, -1)

        return tl, tr, cg