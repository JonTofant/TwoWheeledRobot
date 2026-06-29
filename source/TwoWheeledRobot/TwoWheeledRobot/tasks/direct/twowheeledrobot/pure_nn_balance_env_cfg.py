"""Pure neural-network 50 Hz balance controller task configuration."""

import math

from isaaclab.utils import configclass

from .standup_env_cfg import StandupEnvCfg


@configclass
class PureNNBalanceEnvCfg(StandupEnvCfg):
    observation_space: int = 8
    action_space: int = 2
    state_space: int = 0

    episode_length_s: float = 8.0
    floor_initial_pitch_deg: float = 3.0
    floor_stop_pitch_deg: float = 25.0
    position_stop_m: float = 1.0
    success_steps_required: int = 1_000_000_000
    enable_wheel_contacts: bool = False
    training_mode: bool = True
    evaluation_mode: bool = False

    # Policy interface. Observation scale order:
    # x_rel, linear_velocity, pitch, pitch_rate, yaw_error, yaw_rate, previous_left_current, previous_right_current.
    include_yaw_rate: bool = True
    observation_scale: tuple = (1.0, 1.0, math.radians(25.0), 4.0, math.pi, 4.0, 2.0, 2.0)
    i_max_a: float = 2.0
    # Default training uses direct NN current commands. Smoothing and hard slew
    # limiting remain configurable for hardware safety tests, but are disabled
    # by default so PPO can learn the necessary current changes at 50 Hz.
    action_smoothing_alpha: float = 0.0
    enable_current_slew_limit: bool = False
    hardware_safe_current_slew_limit: bool = False
    current_slew_limit_a: float = 0.3

    # Progressive training stage: 1..5. Override from CLI, e.g. curriculum_stage=3.
    curriculum_stage: int = 1

    # Reward weights: +alive - weighted quadratic penalties.
    rew_alive: float = 1.0
    rew_pitch: float = 12.0
    rew_pitch_rate: float = 0.8
    rew_velocity: float = 0.15
    rew_position: float = 0.4
    rew_yaw_error: float = 0.1
    rew_current: float = 0.01
    rew_delta_current: float = 0.03

    # Domain randomization. Physical mass/COM/inertia hooks are sampled and logged;
    # apply support is simulator-version dependent, so motor/sensor randomization is the deployed-critical path.
    body_mass_scale_range: tuple = (0.9, 1.1)
    body_com_height_scale_range: tuple = (0.9, 1.1)
    body_pitch_inertia_scale_range: tuple = (0.8, 1.2)
    wheel_radius_scale_range: tuple = (0.98, 1.02)
    ground_static_friction_range: tuple = (0.7, 1.1)
    ground_dynamic_friction_range: tuple = (0.6, 1.0)

    motor_gain_range: tuple = (0.8, 1.2)
    motor_deadzone_a_range: tuple = (0.03, 0.20)
    motor_bias_a_range: tuple = (-0.08, 0.08)
    # Electrical/current-loop response lag, not the robot mechanical time
    # constant. Sampled once per episode and held fixed. A wider robustness
    # setting up to 0.030 s can be used by overriding this range from Hydra.
    motor_tau_s_range: tuple = (0.005, 0.020)
    motor_current_limit_a_range: tuple = (1.6, 2.4)

    pitch_bias_rad_range: tuple = (math.radians(-0.5), math.radians(0.5))
    pitch_noise_std: float = math.radians(0.15)
    pitch_rate_noise_std: float = 0.02
    velocity_noise_std: float = 0.01

    # Physically motivated sim2real disturbance curriculum.
    # Human push: short force pulse on the selected body/platform.
    human_push_start_s_range: tuple = (1.0, 6.0)
    human_push_duration_s_range: tuple = (0.05, 0.20)
    human_push_fx_n_range: tuple = (-6.0, 6.0)
    human_push_fy_n_range: tuple = (-1.5, 1.5)
    human_push_yaw_torque_nm_range: tuple = (-0.15, 0.15)

    # Payload / shifted COM fallback model: persistent pitch-axis torque step.
    payload_start_s_range: tuple = (1.0, 4.0)
    payload_pitch_torque_nm_range: tuple = (-0.25, 0.25)

    # Optional slope-equivalent diagnostic/late-stage bias. This is not part of
    # the default training distribution unless explicitly enabled below.
    enable_slope_disturbance: bool = False
    stage5_slope_probability: float = 0.05
    slope_fx_n_range: tuple = (-1.5, 1.5)

    # Optional sine diagnostic only. Sine is intentionally excluded from the
    # default training curriculum because it is not a primary hardware scenario.
    sine_force_n_range: tuple = (0.5, 2.0)
    sine_frequency_hz_range: tuple = (0.2, 1.0)
    benchmark_disturbance_kind: str = ""  # "", "none", "human_push", "double_human_push", "payload", "payload_push", "slope", "sine_diagnostic"
