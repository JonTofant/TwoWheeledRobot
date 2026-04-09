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
  ROBOT_MASS_OFFSET_KG     — add/subtract kg from every rigid body (0 = perfect)
  LINEAR_DAMPING           — linear  velocity damping on all links
  ANGULAR_DAMPING          — angular velocity damping on all links

Solver
------
  SOLVER_POSITION_ITERS    — PhysX position solver iterations (higher = stiffer)
  SOLVER_VELOCITY_ITERS    — PhysX velocity solver iterations
"""

# =========================================================================== #
#  Timing                                                                      #
# =========================================================================== #
PHYSICS_DT:           float = 1.0 / 200.0   # s  →  5 ms physics step
CONTROL_DECIMATION:   int   = 4             # control sample time = 5ms × 4 = 20 ms

# =========================================================================== #
#  Ground plane                                                                #
# =========================================================================== #
# Rubber tire on laminate flooring.  Measured ranges: μ_s ≈ 0.4–0.6,
# μ_d ≈ 0.3–0.5.  Slip torque per wheel = μ_s × (m·g / 2) × r_wheel
# ≈ 0.5 × (3.80 × 9.81 / 2) × 0.0505 ≈ 0.47 Nm.
# PhysX enforces this automatically — no need to manually clamp torque
# to the slip threshold; the joint effort limit handles motor saturation.
GROUND_STATIC_FRICTION:  float = 0.5   # rubber on laminate
GROUND_DYNAMIC_FRICTION: float = 0.4   # kinetic (post-slip)
GROUND_RESTITUTION:      float = 0.0   # no bounce

# =========================================================================== #
#  Robot body imperfections                                                    #
# =========================================================================== #

# Extra mass added to every rigid body (kg).
ROBOT_MASS_OFFSET_KG: float = 0.0

# Passive link-body velocity damping (air drag, structural flex).
LINEAR_DAMPING:  float = 0.0
ANGULAR_DAMPING: float = 0.0

# =========================================================================== #
#  Joint friction                                                              #
# =========================================================================== #

# DDSM115 wheel joints — internal motor friction (back-EMF + planetary gear).
# Measured by spinning the real wheel with no current and fitting a linear
# drag model.  0.05 Nm·s/rad gives ~0.5 Nm drag at 10 rad/s which is
# consistent with the DDSM115 datasheet efficiency figures.
WHEEL_INTERNAL_DAMPING: float = 0.05   # Nm·s/rad

# Passive revolute joints (leg parallelogram bearings).
# Rolling-element bearings have very low but non-zero viscous friction.
# 0.005 Nm·s/rad corresponds to ~0.05 Nm at 10 rad/s — reasonable for
# a small deep-groove ball bearing.
BEARING_DAMPING: float = 0.005   # Nm·s/rad

# =========================================================================== #
#  Battery CoM offset                                                          #
# =========================================================================== #
# The LiPo battery sits slightly off the geometric centre of Platform_Group.
# Applied to the Platform_Group USD prim after scene load (see env __init__).
# Positive X = toward robot front, positive Y = toward robot left.
# Set both to 0.0 to disable.
BATTERY_COM_OFFSET_X: float =  0.02   # m  (2 cm forward — tune to real layout)
BATTERY_COM_OFFSET_Y: float =  0.00   # m

# =========================================================================== #
#  Solver quality                                                              #
# =========================================================================== #
SOLVER_POSITION_ITERS: int = 8
SOLVER_VELOCITY_ITERS: int = 4
