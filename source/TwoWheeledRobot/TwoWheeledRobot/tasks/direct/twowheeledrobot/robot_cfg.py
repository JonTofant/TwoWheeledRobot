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

from .control import WHEEL_DRIVE_STIFFNESS, WHEEL_DRIVE_DAMPING, CYBERGEAR_STIFFNESS, CYBERGEAR_DAMPING
from .sim_params import (
    LINEAR_DAMPING, ANGULAR_DAMPING,
    SOLVER_POSITION_ITERS, SOLVER_VELOCITY_ITERS,
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
        # Wheel drive joints — EFFORT (torque) controlled.                   #
        # DDSM115: diameter=115mm, peak=6Nm, no-load=150RPM at 24V          #
        #                                                                     #
        # stiffness / damping come from control.py so the USD keeps them     #
        # at 0 and Python sets the desired values at simulation start.       #
        # ------------------------------------------------------------------ #
        "wheel_joints": ImplicitActuatorCfg(
            joint_names_expr=["DDSM115_Levi", "DDSM115_Desni"],   # left, right
            effort_limit_sim=6.0,                              # Nm — DDSM115 peak
            velocity_limit=15.7,                               # rad/s — no-load at 24V
            stiffness=WHEEL_DRIVE_STIFFNESS,
            damping=WHEEL_DRIVE_DAMPING,
        ),
        # ------------------------------------------------------------------ #
        # CyberGear top motors — position/torque controlled.                 #
        # front_left, front_right, back_left, back_right                     #
        # Tune CYBERGEAR_STIFFNESS / CYBERGEAR_DAMPING in control.py.        #
        # ------------------------------------------------------------------ #
        "cybergear_joints": ImplicitActuatorCfg(
            joint_names_expr=["front_left", "front_right", "back_left", "back_right"],
            effort_limit_sim=12.0,                             # Nm — CyberGear M5 peak
            velocity_limit=30.0,                               # rad/s — adjust to spec
            stiffness=CYBERGEAR_STIFFNESS,
            damping=CYBERGEAR_DAMPING,
        ),
    },
)
