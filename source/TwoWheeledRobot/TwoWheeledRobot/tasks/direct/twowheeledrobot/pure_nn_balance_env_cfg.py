"""Pure neural-network ~66.7 Hz (15 ms) balance controller task configuration."""

import math

from isaaclab.utils import configclass

from .standup_env_cfg import StandupEnvCfg


@configclass
class PureNNBalanceEnvCfg(StandupEnvCfg):
    observation_space: int = 8
    action_space: int = 2
    state_space: int = 0

    # Override the shared 20 ms control step: this task's control loop must
    # match the real hardware's 15 ms period. PHYSICS_DT stays 1 ms, so this
    # yields an exact 15-physics-step control decimation (~66.7 Hz control).
    # Standup/ResidualLQR are unaffected — they keep sim_params.CONTROL_DECIMATION.
    decimation: int = 15

    episode_length_s: float = 8.0
    floor_initial_pitch_deg: float = 3.0
    floor_stop_pitch_deg: float = 25.0
    # "tilt" is the historical rule and stays the default for every task that is
    # not NNDrive. NNDriveEnvCfg overrides it to "contact"; see the comment there.
    fall_mode: str = "tilt"
    contact_force_threshold_n: float = 1.0
    fall_pitch_threshold_deg: float = 25.0
    fall_total_tilt_threshold_deg: float = 25.0
    fall_consecutive_steps: int = 5
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
    # by default so PPO can learn the necessary current changes at the 15 ms
    # control rate.
    action_smoothing_alpha: float = 0.0
    enable_current_slew_limit: bool = False
    hardware_safe_current_slew_limit: bool = False
    current_slew_limit_a: float = 0.3

    # Progressive training stage: 1..5. Override from CLI, e.g. curriculum_stage=3.
    curriculum_stage: int = 1

    # Initial state randomization for balance training.
    reset_pitch_range_deg: float = 20.0
    reset_pitch_rate_range_radps: float = 0.8
    reset_velocity_range_mps: float = 0.15

    # Reward weights: +alive - weighted quadratic penalties. Position penalty
    # is station-keeping: it drives the robot back toward its reset point
    # rather than letting it drift at low velocity. At 3.0, a 10 cm drift
    # costs -0.03/step (negligible next to rew_alive=1.0) but a 30 cm drift
    # costs -0.27/step (comparable to a ~10 deg pitch excursion) — small
    # drift is tolerated, larger drift is actively corrected. Previous 0.1
    # was too weak to matter (10 cm drift cost only -0.001/step) and did not
    # meaningfully change behavior. Retune down if this starts fighting pitch
    # recovery under disturbances (oscillating hard trying to re-home).
    rew_alive: float = 1.0
    rew_pitch: float = 10.0
    rew_pitch_rate: float = 0.4
    rew_roll: float = 10.0
    rew_roll_rate: float = 0.4
    rew_velocity: float = 0.15
    rew_position: float = 3.0
    rew_yaw_error: float = 0.20
    rew_yaw_rate: float = 0.30
    rew_current: float = 0.005
    rew_delta_current: float = 0.0
    fall_penalty: float = -100.0
    physics_broken_penalty: float = -100.0
    invalid_state_penalty: float = -100.0

    # Active sim2real randomization. Wheel friction and motor response parameters
    # are randomized around physically plausible/identified nominal values.
    # NOT a Nm frictionloss despite the name — this goes to Isaac Lab's
    # write_joint_friction_coefficient_to_sim, whose argument is a dimensionless
    # coefficient bounding the resisting torque at mu_s * |F_spatial|, i.e. it
    # scales with the transmitted constraint force. It therefore cannot be
    # compared directly to EMB-17's 23-58 mNm measured wheel friction, and it may
    # be modelling the same physics the current deadzone already models. Open
    # question, see the "wheel friction modelled twice" task. Nominal 0.010.
    wheel_frictionloss_range: tuple = (0.007, 0.013)
    wheel_viscous_damping_randomization_active: bool = True
    wheel_viscous_damping_range: tuple = (0.006, 0.014)  # Nm*s/rad, DDSM115 internal damping estimate.
    ground_friction_randomization_mode: str = "per_run"  # Shared ground plane; sampled once at env startup.
    ground_static_friction_range: tuple = (0.5, 1.1)
    ground_dynamic_friction_range: tuple = (0.4, 0.9)

    # Inactive/planned physical randomization. These ranges are not applied by
    # the current pure NN environment and must not be reported as active.
    body_mass_scale_range: tuple = (0.9, 1.1)
    body_com_height_scale_range: tuple = (0.9, 1.1)
    body_pitch_inertia_scale_range: tuple = (0.8, 1.2)
    wheel_radius_scale_range: tuple = (0.98, 1.02)

    randomization_debug_log_resets: int = 3
    randomization_debug_env_count: int = 4

    # Motor gain (current-to-torque scale). ASSUMED, not identified: K_t cannot be
    # reached on a free-running rig, so there is no measured torque-constant
    # spread. The nearest bound is the no-load speed plateau, whose session-immune
    # unit-to-unit spread is 1.21 % of mean (EMB-18, n = 12) — and that already
    # includes friction variation, so it is an upper bound on the K_t spread.
    # +-3 % is that bound with safety margin. The previous +-20 % randomized over
    # motors the measured population does not contain, which would have made the
    # EXP-A (point estimate) vs EXP-B (range) comparison meaningless.
    motor_gain_range: tuple = (0.97, 1.03)
    # Deadzone. IDENTIFIED from EMB-18 (n = 12 units, 5 repeats, 25 V bench supply,
    # free-running): stopping-deadzone mean 53.4 mA, observed 36.0-78.0 mA,
    # between-unit SD 7.6 mA (session-immune) to 11.1 mA (naive), against a 0.8 mA
    # rig noise floor. Sampled independently per wheel, and CurrentActionProcessor
    # applies each draw symmetrically to both directions — POS and NEG deadzone
    # correlate at r = +0.989 across units, so the asymmetry is a per-unit constant
    # and sampling the two directions independently would generate motors that do
    # not exist. Note this is the *stopping* (kinetic) deadzone; breakaway (static,
    # from rest) is 2-3x larger and is not reliably measurable on that rig.
    # Provisional: n = 12 of a ~30-unit target — re-derive from EMB-18 before any
    # EXP-B result is written up.
    motor_deadzone_a_range: tuple = (0.031, 0.078)
    # Constant per-wheel current offset, added before the deadzone is applied, so
    # it is what would make a zero command produce torque. IDENTIFIED AS ZERO
    # (EMB-17, 2026-08-04), measured directly in the stalled region (|command| <=
    # 30 mA, below every unit's deadzone, so there is no back-EMF and the current
    # loop delivers exactly what it is asked for): bias at commanded zero is
    # -0.002 mA with a between-unit SD of 0.030 counts = 0.0074 mA at the measured
    # 4096 counts/A, and speed is exactly 0.00 rpm on every unit and every repeat.
    # That is ~0.0015 mNm of offset torque.
    #
    # Left at zero rather than given a token range: the usual "do not randomize
    # too narrowly" argument applies to parameters whose spread is unmeasured,
    # and this spread *is* measured, at ~7 uA. Randomizing it models nothing.
    # Previous values were +-0.08 A (~10000x the measured SD) and briefly
    # +-0.005 A, set before the direct measurement existed.
    #
    # Note this is an electromagnetic statement. A PM motor still has cogging
    # torque at zero current, but it is position-dependent and averages to zero
    # over a revolution, so it is not a bias term — it is why *static* breakaway
    # is 2-3x the kinetic deadzone modelled here and is not reliably measurable.
    motor_bias_a_range: tuple = (0.0, 0.0)
    # Electrical/current-loop response lag, not the robot mechanical time
    # constant. Sampled once per episode and held fixed. ASSUMED — the
    # free-running rig cannot reach it (a load is required, EMB-18 open item).
    # Narrowed from (0.005, 0.020): the old upper end exceeded the 15 ms control
    # period, so one policy was trained across plants ranging from "responds
    # within the step" to "a full step behind", which is a large unsupported
    # variance. A wider robustness setting can still be applied from Hydra.
    motor_tau_s_range: tuple = (0.005, 0.010)
    # ASSUMED. Saturation is a driver/firmware property rather than a
    # manufacturing one, so it should be near-identical across units. Narrowed
    # from (1.6, 2.4): the old lower end sat below i_max_a = 2.0, so those
    # episodes clipped the policy's own maximum command and taught it that its
    # top-end authority is unreliable.
    motor_current_limit_a_range: tuple = (1.9, 2.1)

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
