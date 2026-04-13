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

Control algorithm: Event-Triggered Adaptive Sliding Mode Control (ETASMC)
------------------------------------------------------------
Two coupled SMC surfaces with event-triggered zero-order hold (ZOH) and
adaptive gain scheduling.  The ZOH holds the last computed control output
between events; a new output is computed only when a surface-based event
fires.  Near equilibrium events space out naturally (self-quiescence) and
the gain scales down, eliminating chattering without sacrificing recovery
authority during large disturbances.

  Balance + velocity + position surface (common mode — equal current both wheels):
    x_odom  = 0.5·R·(−φ_L + φ_R)           (wheel odometry, direct encoder read)
    _x_cmd advances at vel_cmd·dt when moving, freezes when vel_cmd=0
      → position-hold engages automatically when motion stops (mirrors heading-hold)
    s_bal = θ + SMC_ALPHA·θ̇ + SMC_BETA·(v_cmd − v) + ET_GAMMA·(x_cmd − x_odom)
    K_bal = ET_K_MIN_BAL + (ET_K_MAX_BAL − ET_K_MIN_BAL)·|s|/(|s| + ET_K_SIGMA_BAL)
    u_bal = K_bal · tanh(s_bal / SMC_PHI)   [A]  (computed only on trigger)

  Heading surface (differential mode — yaw torque via wheel current difference):
    ψ̂  = ∫ ψ̇ dt           (integrated yaw ≈ compass heading)
    s_yaw = e_heading + SMC_YAW_LAMBDA·(ψ̇_cmd − ψ̇)
    K_yaw = ET_K_MIN_YAW + (ET_K_MAX_YAW − ET_K_MIN_YAW)·|s|/(|s| + ET_K_SIGMA_YAW)
    u_yaw = K_yaw · tanh(s_yaw / SMC_YAW_PHI)   [A]  (computed only on trigger)

  Event trigger (Zeno-free, relative + absolute threshold):
    trig_bal = |s_bal(t) − s̃_bal| > ET_ETA_BAL·|s̃_bal| + ET_EPS_BAL
    trig_yaw = |s_yaw(t) − s̃_yaw| > ET_ETA_YAW·|s̃_yaw| + ET_EPS_YAW
    where s̃ is the surface value at the last trigger time (ZOH reference).
    The ε term guarantees a positive minimum inter-event time (Zeno-free):
      T_min ≥ ε / max|ds/dt|  >  0

  Wheel currents (ZOH held between events):
    current_left  = u_bal − u_yaw
    current_right = u_bal + u_yaw

Heading-hold / position-hold logic:
  Both heading and position use the same implicit hold pattern:
    · While commanding motion the set-point advances → error ≈ 0 (rate/velocity mode)
    · Releasing the command freezes the set-point → hold mode engages automatically
"""

from __future__ import annotations

import math
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
#  Sliding Mode Control — surface shape parameters
# ──────────────────────────────────────────────────────────────────────────────

# Balance + velocity + position — combined sliding surface, common mode.
# s_bal = θ + SMC_ALPHA·θ̇ + SMC_BETA·(v_cmd − v) + ET_GAMMA·(x_cmd − x_odom)
SMC_ALPHA: float = 0.08   # s      — tilt-rate weight (≈ pendulum natural period / 2π)
SMC_BETA:  float = 0.60   # s/m    — velocity-error weight
SMC_PHI:   float = 0.20   # rad    — tanh boundary-layer width (tightened vs 0.25;
#                                     adaptive gain now handles near-eq chattering)

# Heading / yaw — differential current, positive = CCW (left turn).
# s_yaw = e_heading + SMC_YAW_LAMBDA·(ψ̇_cmd − ψ̇)
# SMC_YAW_LAMBDA halved: 0.30 gave 0.30·psi_dot derivative term which alone
# saturated u_yaw at any moderate spin rate, starving the balance loop.
SMC_YAW_LAMBDA: float = 0.15   # s    — yaw-rate weight (halved from 0.30)
SMC_YAW_PHI:   float = 0.20   # rad  — boundary-layer width (widened back to 0.20 for smoother yaw)

# ──────────────────────────────────────────────────────────────────────────────
#  Event-Triggered Adaptive SMC (ETASMC) parameters
# ──────────────────────────────────────────────────────────────────────────────

# Position hold — weight of odometry error term in balance surface.
# Kept small relative to SMC_BETA (0.60) so balance dynamics dominate.
# At ET_GAMMA=0.10, a 0.1 m displacement contributes 0.01 rad to s_bal.
# NOTE: set to 0.0 while tuning balance/yaw stability. Spinning corrupts
# x_odom (wheel odometry is not yaw-invariant), which feeds noise into s_bal.
# Re-enable once the robot holds upright without spinning.
ET_GAMMA: float = 0.00    # m⁻¹  (disabled for initial tuning)

# Adaptive gain — balance surface.
# K_bal(|s|) = K_min + (K_max − K_min)·|s|/(|s| + σ)
#   |s|=0.03 → K≈0.44 A (gentle),  |s|=0.15 → K≈0.83 A,  |s|→∞ → K_max
ET_K_MAX_BAL:   float = 1.6    # A  — max current; raised to give balance more authority
ET_K_MIN_BAL:   float = 0.70   # A  — strong minimum so balance always dominates yaw budget
ET_K_SIGMA_BAL: float = 0.15   # rad — half-saturation point

# Adaptive gain — heading surface.
# Yaw torque must stay well below balance (u_yaw << u_bal) or balance loses.
# Previously 0.50 A caused u_yaw > u_bal, consuming torque budget.
ET_K_MAX_YAW:   float = 0.15   # A  — hard cap; keeps yaw subordinate to balance
ET_K_MIN_YAW:   float = 0.02   # A
ET_K_SIGMA_YAW: float = 0.10   # rad

# Event trigger thresholds.
# trig = |s(t) − s̃| > ETA·|s̃| + EPS
# EPS > 0 is the Zeno-free guarantee: T_min ≥ EPS/max|ṡ| > 0.
# EPS_BAL=0.008 rad ≈ 5× IMU noise floor (~0.001–0.003 rad).
ET_ETA_BAL: float = 0.20    # relative threshold (20 % deviation triggers update)
ET_EPS_BAL: float = 0.008   # rad — absolute floor

# Yaw trigger is much more sensitive (near-continuous) to prevent ZOH overshoot oscillation.
# The yaw channel is fast — holding a stale correction long enough to accumulate heading
# error causes alternating max corrections → spinning.  Low ETA keeps it almost fixed-rate
# while still demonstrating the event-based principle.
ET_ETA_YAW: float = 0.05    # near-continuous: fires whenever s_yaw changes > 5 %
ET_EPS_YAW: float = 0.003   # rad — absolute floor


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

        # Heading state — simulated magnetometer / compass integration.
        self._heading     = torch.zeros(num_envs, device=device, dtype=torch.float32)
        self._heading_cmd = torch.zeros(num_envs, device=device, dtype=torch.float32)

        # Position hold state — mirrors the heading-hold pattern.
        # _x_cmd advances at vel_cmd·dt while moving; freezes when vel_cmd=0.
        self._x_cmd = torch.zeros(num_envs, device=device, dtype=torch.float32)

        # ── Event-trigger ZOH state ──────────────────────────────────────────
        # s̃: surface value at the last trigger (ZOH reference point)
        self._s_bal_last = torch.zeros(num_envs, device=device, dtype=torch.float32)
        self._s_yaw_last = torch.zeros(num_envs, device=device, dtype=torch.float32)

        # Held control output (zero-order hold between events)
        self._u_bal_hold = torch.zeros(num_envs, device=device, dtype=torch.float32)
        self._u_yaw_hold = torch.zeros(num_envs, device=device, dtype=torch.float32)

        # ── Event diagnostics ────────────────────────────────────────────────
        # Last recorded inter-event time (seconds elapsed between consecutive triggers)
        self._iet_bal     = torch.zeros(num_envs, device=device, dtype=torch.float32)
        self._iet_yaw     = torch.zeros(num_envs, device=device, dtype=torch.float32)
        # Accumulator: time elapsed since the last trigger
        self._t_since_bal = torch.zeros(num_envs, device=device, dtype=torch.float32)
        self._t_since_yaw = torch.zeros(num_envs, device=device, dtype=torch.float32)

    def reset(self, env_ids=None) -> None:
        if env_ids is None:
            self._heading.zero_()
            self._heading_cmd.zero_()
            self._x_cmd.zero_()
            self._s_bal_last.zero_()
            self._s_yaw_last.zero_()
            self._u_bal_hold.zero_()
            self._u_yaw_hold.zero_()
            self._iet_bal.zero_()
            self._iet_yaw.zero_()
            self._t_since_bal.zero_()
            self._t_since_yaw.zero_()
        else:
            self._heading[env_ids]     = 0.0
            self._heading_cmd[env_ids] = 0.0
            self._x_cmd[env_ids]       = 0.0
            self._s_bal_last[env_ids]  = 0.0
            self._s_yaw_last[env_ids]  = 0.0
            self._u_bal_hold[env_ids]  = 0.0
            self._u_yaw_hold[env_ids]  = 0.0
            self._iet_bal[env_ids]     = 0.0
            self._iet_yaw[env_ids]     = 0.0
            self._t_since_bal[env_ids] = 0.0
            self._t_since_yaw[env_ids] = 0.0

    def compute(
        self,
        projected_gravity_b: torch.Tensor,   # (N, 3) — gravity vector in body frame
        ang_vel_b:           torch.Tensor,   # (N, 3) — body angular velocity
        omega_left:          torch.Tensor,   # (N,)   — left  wheel speed  [rad/s]
        omega_right:         torch.Tensor,   # (N,)   — right wheel speed  [rad/s]
        phi_left:            torch.Tensor,   # (N,)   — left  wheel angle  [rad]
        phi_right:           torch.Tensor,   # (N,)   — right wheel angle  [rad]
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:

        dt = self.dt

        # ── 1. State estimation ──────────────────────────────────────────────────
        theta     = torch.asin(projected_gravity_b[:, IMU_TILT_GRAVITY_AXIS].clamp(-1.0, 1.0))  # (N,) rad
        theta_dot = ang_vel_b[:, IMU_TILT_RATE_AXIS]   # (N,) rad/s
        psi_dot   = ang_vel_b[:, IMU_YAW_RATE_AXIS]    # (N,) rad/s  positive = CCW

        # Linear velocity — left wheel USD mesh is mirrored: −ω_L + ω_R = forward.
        v = (-omega_left + omega_right) * (0.5 * WHEEL_RADIUS)   # (N,) m/s

        # Wheel odometry: direct read from joint angles (no integration in control law).
        # Same sign convention as velocity: −φ_L + φ_R gives net forward displacement.
        x_odom = 0.5 * WHEEL_RADIUS * (-phi_left + phi_right)    # (N,) m

        # ── 2. Heading integration (simulated compass) ───────────────────────────
        self._heading.add_(psi_dot, alpha=dt)
        # Advance heading set-point at yaw_rate_cmd; heading-hold when cmd=0.
        self._heading_cmd.add_(dt * self.yaw_rate_cmd)

        # ── 3. Position set-point (mirrors heading-hold pattern) ─────────────────
        # While vel_cmd ≠ 0 the position set-point advances → x error ≈ 0 (velocity mode).
        # When vel_cmd = 0 the set-point freezes → position-hold engages automatically.
        self._x_cmd.add_(self.vel_cmd * dt)

        # ── 4. Sliding surfaces ──────────────────────────────────────────────────
        # Balance surface — extended with position-hold term.
        s_bal = (theta
                 + SMC_ALPHA * theta_dot
                 + SMC_BETA  * (self.vel_cmd - v)
                 + ET_GAMMA  * (self._x_cmd - x_odom))   # (N,) rad

        # Heading surface (unchanged).
        e_heading = self._heading_cmd - self._heading
        e_heading = (e_heading + math.pi) % (2.0 * math.pi) - math.pi   # wrap ±π
        s_yaw = e_heading + SMC_YAW_LAMBDA * (self.yaw_rate_cmd - psi_dot)  # (N,) rad

        # ── 5. Event triggers (relative + absolute, Zeno-free) ───────────────────
        trig_bal = (s_bal - self._s_bal_last).abs() > ET_ETA_BAL * self._s_bal_last.abs() + ET_EPS_BAL
        trig_yaw = (s_yaw - self._s_yaw_last).abs() > ET_ETA_YAW * self._s_yaw_last.abs() + ET_EPS_YAW

        # ── 6. Adaptive gain — smooth rational schedule ──────────────────────────
        # K(|s|) = K_min + (K_max − K_min)·|s|/(|s| + σ)
        # Monotone, bounded, continuous — no switched-gain discontinuities.
        s_bal_mag = s_bal.abs()
        k_bal = ET_K_MIN_BAL + (ET_K_MAX_BAL - ET_K_MIN_BAL) * s_bal_mag / (s_bal_mag + ET_K_SIGMA_BAL)

        s_yaw_mag = s_yaw.abs()
        k_yaw = ET_K_MIN_YAW + (ET_K_MAX_YAW - ET_K_MIN_YAW) * s_yaw_mag / (s_yaw_mag + ET_K_SIGMA_YAW)

        # ── 7. New control candidates (evaluated everywhere; applied only on trigger) ─
        u_bal_new = k_bal * torch.tanh(s_bal / SMC_PHI)     # (N,) A
        u_yaw_new = k_yaw * torch.tanh(s_yaw / SMC_YAW_PHI) # (N,) A

        # ── 8. ZOH update — apply new value only where triggered ────────────────
        self._u_bal_hold = torch.where(trig_bal, u_bal_new, self._u_bal_hold)
        self._u_yaw_hold = torch.where(trig_yaw, u_yaw_new, self._u_yaw_hold)
        # Update the ZOH reference surface for triggered environments.
        self._s_bal_last = torch.where(trig_bal, s_bal, self._s_bal_last)
        self._s_yaw_last = torch.where(trig_yaw, s_yaw, self._s_yaw_last)

        # ── 9. Inter-event time diagnostics ─────────────────────────────────────
        self._t_since_bal += dt
        self._t_since_yaw += dt
        # On trigger: record IET, reset accumulator.
        self._iet_bal     = torch.where(trig_bal, self._t_since_bal, self._iet_bal)
        self._iet_yaw     = torch.where(trig_yaw, self._t_since_yaw, self._iet_yaw)
        zeros = torch.zeros(self.num_envs, device=self.device, dtype=torch.float32)
        self._t_since_bal = torch.where(trig_bal, zeros, self._t_since_bal)
        self._t_since_yaw = torch.where(trig_yaw, zeros, self._t_since_yaw)

        # ── 10. Wheel currents — ZOH output ─────────────────────────────────────
        current_left  = self._u_bal_hold - self._u_yaw_hold   # (N,) A
        current_right = self._u_bal_hold + self._u_yaw_hold   # (N,) A

        # ── 11. CyberGear — hold upright stance ──────────────────────────────────
        desired_angle:    float = 0.0
        desired_velocity: float = 0.0
        ff_torque:        float = 0.0

        # ┌─────────────────────────────────────────────────────────────────┐
        # │               TRANSLATION LAYER — do not edit                   │
        # └─────────────────────────────────────────────────────────────────┘

        tl = -current_left  * DDSM115_KT    # (N,)  left  wheel negated (USD mirror)
        tr =  current_right * DDSM115_KT    # (N,)

        _z4    = torch.zeros((self.num_envs, 4), device=self.device, dtype=torch.float32)
        cg_pos = self._cg_stance + desired_angle
        cg_vel = _z4 + desired_velocity
        cg_ff  = _z4 + ff_torque

        self.last = {
            "tilt":        math.degrees(float(theta[0])),
            "tilt_rate":   float(theta_dot[0]),
            "theta_des":   0.0,
            "yaw_rate":    float(psi_dot[0]),
            "torque_L":    float(tl[0]),
            "torque_R":    float(tr[0]),
            "vel_cmd":     self.vel_cmd,
            "vel_fwd":     float(v[0]),
            # ETASMC diagnostics
            "s_bal":       float(s_bal[0]),
            "s_yaw":       float(s_yaw[0]),
            "trig_bal":    float(trig_bal[0]),
            "trig_yaw":    float(trig_yaw[0]),
            "iet_bal":     float(self._iet_bal[0]),
            "iet_yaw":     float(self._iet_yaw[0]),
            "k_bal":       float(k_bal[0]),
            "x_odom":      float(x_odom[0]),
            "u_bal":       float(self._u_bal_hold[0]),
            "u_yaw":       float(self._u_yaw_hold[0]),
        }

        return tl, tr, cg_pos, cg_vel, cg_ff
