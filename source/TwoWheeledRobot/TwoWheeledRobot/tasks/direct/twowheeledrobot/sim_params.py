"""
Simulation world parameters — physics fidelity and robot imperfections.
=======================================================================

Edit this file to introduce realistic imperfections without touching the
controller or the USD asset.  Useful for testing robustness before
deploying to the real robot.

Timing
------
  PHYSICS_DT      — PhysX time step (seconds)
  CONTROL_DECIMATION — physics steps per controller call
  → control / sensor sample time = PHYSICS_DT * CONTROL_DECIMATION  (20 ms)

Ground
------
  GROUND_STATIC_FRICTION   — static  friction coefficient
  GROUND_DYNAMIC_FRICTION  — dynamic friction coefficient
  GROUND_RESTITUTION       — bounciness (0 = none)

Robot body
----------
  LINEAR_DAMPING           — linear  velocity damping on all links
  ANGULAR_DAMPING          — angular velocity damping on all links

Actuators
---------
  DDSM115_*                 — DDSM115 current/torque/speed motor model
  WHEEL_DRIVE_STIFFNESS    — PhysX stiffness for effort-controlled wheel joints
  CYBERGEAR_STIFFNESS      — default CyberGear position gain in simulation
  CYBERGEAR_DAMPING        — default CyberGear damping gain in simulation

Solver
------
  SOLVER_POSITION_ITERS    — PhysX position solver iterations (higher = stiffer)
  SOLVER_VELOCITY_ITERS    — PhysX velocity solver iterations
"""

import math

# =========================================================================== #
#  Timing                                                                      #
# =========================================================================== #
PHYSICS_DT:           float = 1.0 / 1000.0  # s  →  1 ms physics step
CONTROL_DECIMATION:   int   = 20            # control sample time = 1ms × 20 = 20 ms

# =========================================================================== #
#  Ground plane                                                                #
# =========================================================================== #
# Rubber tire on laminate / tile flooring (laboratory environment).
# Measured ranges: μ_s ≈ 0.4–0.6, μ_d ≈ 0.3–0.5.
# Slip torque per wheel = μ_s × (m·g / 2) × r_wheel
# ≈ 0.5 × (3.80 × 9.81 / 2) × 0.0505 ≈ 0.47 Nm.
# Using mid-range values; overly high friction (0.8) combined with locked wheels
# and high-stiffness legs caused large destabilising lateral forces in practice.
GROUND_STATIC_FRICTION:  float = 0.6   # rubber on tile (mid-range)
GROUND_DYNAMIC_FRICTION: float = 0.4   # kinetic (post-slip)
GROUND_RESTITUTION:      float = 0.0   # no bounce

# =========================================================================== #
#  Robot body imperfections                                                    #
# =========================================================================== #

# Passive link-body velocity damping (air drag, structural flex).
LINEAR_DAMPING:  float = 0.0
ANGULAR_DAMPING: float = 0.0

# =========================================================================== #
#  Joint friction                                                              #
# =========================================================================== #

# DDSM115 integrated PMSM hub servo. The useful simulation abstraction is a
# current/torque-controlled direct-drive motor, not a position servo.
DDSM115_KT: float = 0.75                         # Nm/A
DDSM115_I_CONT: float = 1.5                      # A, continuous current
DDSM115_I_PEAK: float = 2.7                      # A, short-term command clamp
DDSM115_TAU_RATED: float = 0.96                  # Nm, conservative training limit
DDSM115_TAU_PEAK: float = 2.0                    # Nm, stall/short-term peak
DDSM115_RATED_SPEED: float = 115.0 * 2.0 * math.pi / 60.0  # rad/s
DDSM115_NO_LOAD_SPEED: float = 200.0 * 2.0 * math.pi / 60.0  # rad/s

WHEEL_DRIVE_STIFFNESS: float = 0.0   # pure effort control for wheel joints
CYBERGEAR_STIFFNESS: float = 30.0    # Nm/rad — default sim kp
CYBERGEAR_DAMPING: float = 3.0       # Nm*s/rad — default sim kd

# The torque-speed curve already includes the losses that define no-load speed.
# Additional wheel damping would double-count those losses and make a free wheel
# settle below the documented 200 rpm endpoint.
WHEEL_INTERNAL_DAMPING: float = 0.0   # Nm·s/rad

# Passive revolute joints (leg parallelogram bearings).
# Rolling-element bearings have very low but non-zero viscous friction.
# 0.005 Nm·s/rad corresponds to ~0.05 Nm at 10 rad/s — reasonable for
# a small deep-groove ball bearing.
BEARING_DAMPING: float = 0.005   # Nm·s/rad

# =========================================================================== #
#  Solver quality                                                              #
# =========================================================================== #
SOLVER_POSITION_ITERS: int = 4   # was 8 — halves constraint solver cost; revert to 6 if joints show tunnelling
SOLVER_VELOCITY_ITERS: int = 1   # was 4
