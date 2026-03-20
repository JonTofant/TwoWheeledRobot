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


@configclass
class TwowheeledrobotEnvCfg(DirectRLEnvCfg):
    # ------------------------------------------------------------------ #
    # Simulation timing                                                   #
    # ------------------------------------------------------------------ #
    decimation: int = 4            # physics steps per control step
    episode_length_s: float = 30.0

    # No external policy — control is computed inside the env via control.py.
    action_space: int = 0
    # Observations: [pitch, pitch_rate, forward_vel, position, yaw_rate, ω_L, ω_R]
    observation_space: int = 7
    state_space: int = 0

    sim: SimulationCfg = SimulationCfg(dt=1.0 / 200.0, render_interval=decimation)

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
