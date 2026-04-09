"""
Robot articulation configuration for the custom two-wheeled robot.

Wheel joints: Revolute_13 (left) and Revolute_6 (right)
These are configured as pure effort-controlled actuators.

Drive gains (stiffness / damping) are imported from control.py so that
the USD can keep them at 0 and Python sets the desired values at
simulation start.
"""

import os
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
import isaaclab.sim as sim_utils

from .control import WHEEL_DRIVE_STIFFNESS, CYBERGEAR_STIFFNESS, CYBERGEAR_DAMPING
from .sim_params import (
    LINEAR_DAMPING, ANGULAR_DAMPING,
    SOLVER_POSITION_ITERS, SOLVER_VELOCITY_ITERS,
    WHEEL_INTERNAL_DAMPING, BEARING_DAMPING,
)

# USD path — ColectedUSD_v2/World0.usd lives inside docs/
_USD_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),    # .../TwoWheeledRobot/tasks/direct/twowheeledrobot/
    "..", "..", "..", "..",        # up to source/TwoWheeledRobot/
    "docs", "ColectedUSD_v2", "World0.usd"
))

TWO_WHEELED_ROBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_USD_PATH,
        activate_contact_sensors=False,
        # USD PhysicsMassAPI tokens exist but mass VALUES are broken (zero or unit-mismatch
        # from OnShape export). Override uniformly: 4.5 kg / 11 bodies ≈ 0.41 kg each.
        # The per-body breakdown isn't exact but the total mass is correct, which is what
        # matters for wheel-contact dynamics.
        mass_props=sim_utils.MassPropertiesCfg(mass=0.41),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=LINEAR_DAMPING,
            angular_damping=ANGULAR_DAMPING,
            max_linear_velocity=10.0,
            max_angular_velocity=50.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=SOLVER_POSITION_ITERS,
            solver_velocity_iteration_count=SOLVER_VELOCITY_ITERS,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.06859),
        rot=(1.0, 0.0, 0.0, 0.0),
        # NOTE: Add initial joint positions here if the new USD requires
        # specific rest poses for linkage joints.  Wheel joints (Revolute_13,
        # Revolute_6) start at 0 by default.
        joint_pos={},
    ),
    actuators={
        # ------------------------------------------------------------------ #
        # Wheel drive joints — current-controlled (DDSM115).                 #
        #   Kt  = 0.75 Nm/A,  I_max = 8 A  →  τ_peak = 6 Nm (motor)        #
        #   User-measured peak at wheel: 2 Nm                                #
        #   Rated continuous: 0.96 Nm                                        #
        #   Deadzone: ±0.04 A × 0.75 = ±0.03 Nm (applied in control.py)    #
        #   Internal friction: back-EMF + gear train  (WHEEL_INTERNAL_DAMPING)#
        # ------------------------------------------------------------------ #
        "wheel_joints": ImplicitActuatorCfg(
            joint_names_expr=["DDSM115_Levi", "DDSM115_Desni"],
            effort_limit_sim=2.0,          # Nm — real measured peak
            velocity_limit=15.7,           # rad/s — ~150 RPM no-load at 24 V
            stiffness=WHEEL_DRIVE_STIFFNESS,
            damping=WHEEL_INTERNAL_DAMPING,  # 0.05 Nm·s/rad — back-EMF + gear friction
        ),
        # ------------------------------------------------------------------ #
        # CyberGear leg motors — MIT control (PD + torque feedforward).      #
        #   Gains from cybergear.c:  kp = 3.0 Nm/rad,  kd = 0.5 Nm·s/rad   #
        #   (kp range 0–500 Nm/rad, kd range 0–5 Nm·s/rad in firmware)      #
        #   Torque feedforward is zero here; IK sets position target only.   #
        # ------------------------------------------------------------------ #
        "cybergear_joints": ImplicitActuatorCfg(
            joint_names_expr=["front_left", "front_right", "back_left", "back_right"],
            effort_limit_sim=12.0,         # Nm — CyberGear M5 peak
            velocity_limit=30.0,           # rad/s
            stiffness=CYBERGEAR_STIFFNESS,  # Nm/rad — matches kp in cybergear.c
            damping=CYBERGEAR_DAMPING,      # Nm·s/rad — matches kd in cybergear.c
        ),
        # ------------------------------------------------------------------ #
        # Passive revolute joints — leg parallelogram bearings.              #
        #   No motor; only rolling-element bearing friction.                 #
        #                                                                    #
        #   IMPORTANT: Isaac Lab does NOT enforce exclusive joint matching.  #
        #   find_joints() is called independently per actuator group, so    #
        #   ".*" would match ALL joints (including wheels and CyberGear)     #
        #   and overwrite their PhysX stiffness=0 / effort_limit=0 since    #
        #   bearing_joints is processed last.  Use a negative lookahead to   #
        #   explicitly exclude the named actuator joints.                    #
        # ------------------------------------------------------------------ #
        "bearing_joints": ImplicitActuatorCfg(
            joint_names_expr=[
                "(?!DDSM115_Levi|DDSM115_Desni|front_left|front_right|back_left|back_right).+"
            ],
            effort_limit_sim=0.0,          # no motor torque
            velocity_limit=50.0,
            stiffness=0.0,                 # free to rotate
            damping=BEARING_DAMPING,       # 0.005 Nm·s/rad — ball-bearing viscous friction
        ),
    },
)
