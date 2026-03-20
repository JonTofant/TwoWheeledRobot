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
GROUND_STATIC_FRICTION:  float = 0.35  # laminate — realistic range 0.3–0.4
GROUND_DYNAMIC_FRICTION: float = 0.25  # laminate — slightly lower than static
GROUND_RESTITUTION:      float = 0.0   # no bounce

# =========================================================================== #
#  Robot body imperfections                                                    #
# =========================================================================== #

# Extra mass added to every rigid body (kg).
# 0.0  → perfect model.
# 0.1  → each link is 100 g heavier (shifts CoM, stresses balance controller).
ROBOT_MASS_OFFSET_KG: float = 0.0

# Passive velocity damping applied to all links by PhysX.
# Small values (< 0.05) are physically realistic for a rigid robot.
LINEAR_DAMPING:  float = 0.0   # kg/s
ANGULAR_DAMPING: float = 0.0   # kg·m²/s

# =========================================================================== #
#  Solver quality                                                              #
# =========================================================================== #
SOLVER_POSITION_ITERS: int = 8   # increase if joints vibrate or penetrate
SOLVER_VELOCITY_ITERS: int = 1
