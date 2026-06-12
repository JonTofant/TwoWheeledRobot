"""Residual PPO-on-LQR balancing task configuration.

This task keeps the analytical LQR wheel-current controller fully active and
lets PPO learn only a small two-current residual:

    u = u_LQR + u_RL + u_disturbance

The residual action is bounded in amperes before the shared DDSM115 actuator
current saturation and torque-speed model are applied.
"""

from isaaclab.utils import configclass

from .standup_env_cfg import StandupEnvCfg


@configclass
class ResidualLqrEnvCfg(StandupEnvCfg):
    # ── RL interface ──────────────────────────────────────────────────────────
    observation_space: int = 8
    action_space: int = 2

    # ── Residual controller options ──────────────────────────────────────────
    enable_residual_rl: bool = True
    residual_action_limit: float = 0.5
    training_mode: bool = True
    evaluation_mode: bool = False

    # ── Episode setup ────────────────────────────────────────────────────────
    # Residual PPO trains around the existing floor-balancing benchmark, not the
    # self-righting task.  Keep the robot nearly upright with fixed legs.
    episode_length_s: float = 5.0
    floor_initial_pitch_deg: float = 1.0
    floor_stop_pitch_deg: float = 10.0
    success_steps_required: int = 1_000_000_000
    # The residual task does not observe contact forces.  Leave physical floor
    # contact enabled through the robot/ground collision model, but do not create
    # a ContactSensor that requires contact reporter APIs on wheel bodies.
    enable_wheel_contacts: bool = False

    # No domain randomization beyond disturbance timing/amplitude for the first
    # residual implementation.
    wheel_damping_scale_range: tuple = (1.0, 1.0)
    noise_proj_grav_std: float = 0.0
    noise_ang_vel_std: float = 0.0
    noise_cg_pos_std: float = 0.0

    # ── Disturbance randomization ────────────────────────────────────────────
    disturbance_currents_a: tuple = (0.5, 1.0, 1.5, 2.0)
    disturbance_start_range_s: tuple = (1.3, 1.7)
    disturbance_samples: int = 10
