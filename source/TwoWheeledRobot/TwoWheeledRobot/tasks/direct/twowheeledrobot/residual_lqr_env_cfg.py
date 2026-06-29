"""Residual PPO-on-LQR balancing task configuration.

Purpose:
    Override the standup task into a near-upright floor-balancing residual RL
    task where PPO learns bounded wheel-command corrections on top of LQR.

Edit here when:
    You want to change residual action limits, residual observation/action
    sizes, episode pitch limits, or residual-training disturbance distribution.

Avoid changing here without also checking:
    residual_lqr_env.py observation, reward, reset, and disturbance logic, plus
    scripts/lqr_control.py frozen-policy evaluation.

This task keeps the analytical LQR wheel controller fully active and lets PPO
learn only a small two-wheel residual:

    u = u_LQR + u_RL + u_disturbance

With the default MuJoCo actuator model the residual action is bounded in Nm.
If wheel_actuator_model="ddsm115", the same field is interpreted in amperes.
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
    residual_action_limit: float = 0.5  # Nm in MuJoCo mode, A in DDSM115 mode
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
    disturbance_currents_a: tuple = (0.5, 1.0, 1.5, 2.0)  # Nm in MuJoCo mode, A in DDSM115 mode
    disturbance_start_range_s: tuple = (1.3, 1.7)
    disturbance_samples: int = 10
