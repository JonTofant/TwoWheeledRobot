"""
Simulation / scene configuration for the two-wheeled inverted-pendulum env.

Control parameters (gains, physical constants, controller logic) live in
control.py, not here.  This file only holds simulation timing, scene setup,
the robot asset reference, and the IMU sensor path.
"""

from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ImuCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from .robot_cfg import TWO_WHEELED_ROBOT_CFG
from .sim_params import (
    PHYSICS_DT, CONTROL_DECIMATION,
    GROUND_STATIC_FRICTION, GROUND_DYNAMIC_FRICTION, GROUND_RESTITUTION,
)


@configclass
class TwowheeledrobotEnvCfg(DirectRLEnvCfg):
    # ------------------------------------------------------------------ #
    # Simulation timing                                                   #
    # ------------------------------------------------------------------ #
    # Control sample time = PHYSICS_DT * CONTROL_DECIMATION = 20 ms
    decimation: int = CONTROL_DECIMATION
    episode_length_s: float = 30.0

    # No external policy — control is computed inside the env via control.py.
    action_space: int = 0
    # Observations: [tilt, tilt_rate, forward_vel, yaw_rate, ω_L, ω_R]
    observation_space: int = 6
    state_space: int = 0

    sim: SimulationCfg = SimulationCfg(dt=PHYSICS_DT, render_interval=CONTROL_DECIMATION)

    # ------------------------------------------------------------------ #
    # Robot                                                               #
    # ------------------------------------------------------------------ #
    robot_cfg = TWO_WHEELED_ROBOT_CFG.replace(
        prim_path="/World/envs/env_.*/Robot"
    )

    # ------------------------------------------------------------------ #
    # Scene                                                               #
    # ------------------------------------------------------------------ #
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=1,            # pure control — single robot instance
        env_spacing=4.0,
        replicate_physics=True,
    )

    # ------------------------------------------------------------------ #
    # IMU sensor                                                          #
    #                                                                     #
    # prim_path must match the Imu_Sensor xform inside World0.usd.       #
    # Run once and print the prim tree if the path needs adjusting:      #
    #   stage = omni.usd.get_context().get_stage()                       #
    #   for p in stage.Traverse(): print(p.GetPath())                    #
    # ------------------------------------------------------------------ #
    imu: ImuCfg = ImuCfg(
        prim_path=(
            "/World/envs/env_.*/Robot"
            "/SimplifiedBipedMainAssembly"
            "/SimplifiedBipedMainAssembly"
            "/Platform_Group/BNO080"
        ),
        visualizer_cfg=None,
    )

    # ------------------------------------------------------------------ #
    # Termination                                                         #
    # ------------------------------------------------------------------ #
    max_pitch_deg: float = 45.0   # episode ends if robot tilts beyond this
