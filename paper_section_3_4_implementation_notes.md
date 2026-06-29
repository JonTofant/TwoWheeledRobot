# Implementation Notes for Paper Sections 3 and 4

This report documents the current pure neural-network balance-controller implementation in the project. It is based on the code currently present in `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/` and related training/export scripts.

## Section 3: Proposed Neural Controller

## 3.1 Controller structure

Files:
- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/pure_nn_balance_env.py`
- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/pure_nn_components.py`
- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/pure_nn_balance_env_cfg.py`
- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/agents/rsl_rl_pure_nn_balance_cfg.py`
- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/__init__.py`
- `scripts/export_pure_nn_current_onnx.py`
- `scripts/rsl_rl/play.py`

Classes/functions:
- `PureNNBalanceEnv` in `pure_nn_balance_env.py`
- `PureNNBalanceEnv._pre_physics_step()` in `pure_nn_balance_env.py`
- `PureNNBalanceEnv._get_observations()` in `pure_nn_balance_env.py`
- `CurrentActionProcessor` in `pure_nn_components.py`
- `CurrentActionProcessor.process()` in `pure_nn_components.py`
- `NormalizedObservationBuilder` in `pure_nn_components.py`
- `PureNNBalancePPORunnerCfg` in `agents/rsl_rl_pure_nn_balance_cfg.py`
- `CurrentPolicyWrapper` in `scripts/export_pure_nn_current_onnx.py`

Description:
- The pure NN task is registered as `Template-Twowheeledrobot-PureNNBalance-v0` in `twowheeledrobot/__init__.py`.
- The policy network itself is created by RSL-RL from `PureNNBalancePPORunnerCfg.policy`, not manually defined as a standalone PyTorch class in the task code.
- The controller is called by the Isaac Lab/RSL-RL environment loop. Policy actions are received by `PureNNBalanceEnv._pre_physics_step(actions)`.
- The inherited timing is 1 ms physics step with `CONTROL_DECIMATION = 20`, giving a controller period of `0.02 s` or 50 Hz. The inherited `StandupEnv.__init__()` stores this as `self._control_dt = self.cfg.sim.dt * self.cfg.decimation`.
- The two policy outputs are interpreted as left and right wheel current commands after post-processing in `CurrentActionProcessor.process()`.
- Left and right wheel outputs are independent neural-network outputs because `action_space = 2`, with action index 0 used for left current and action index 1 used for right current.
- Action scaling is applied in `CurrentActionProcessor.process()` as `torch.tanh(raw_actions[:, :2]) * cfg.i_max_a`.
- The default network current limit is `i_max_a = 2.0 A` in `PureNNBalanceEnvCfg`.
- Motor-current saturation is applied in `CurrentActionProcessor.process()` using randomized per-motor current limits sampled from `motor_current_limit_a_range = (1.6, 2.4) A`.
- Optional action smoothing is implemented in `CurrentActionProcessor.process()` using `I_filtered = alpha * I_previous + (1 - alpha) * I_network`; default training uses `action_smoothing_alpha = 0.0`.
- Optional slew-rate limiting can clamp the change in filtered current to `current_slew_limit_a = 0.3 A` per 20 ms sample, but default training has `enable_current_slew_limit = False` and `hardware_safe_current_slew_limit = False`.
- The filtered/current-limited command is converted to torque in `PureNNBalanceEnv._pre_physics_step()` using `DDSM115_KT = 0.75 Nm/A`.
- The final wheel torques are speed-limited using the DDSM115 torque-speed limiter in `PureNNBalanceEnv._pre_physics_step()`.

## 3.2 Observation and action space

Observation dimension: `8`.

Observation normalization:
- Implemented in `NormalizedObservationBuilder.build()`.
- Normalization scales are stored in `PureNNBalanceEnvCfg.observation_scale`.
- Current implemented scale tuple: `(1.0, 1.0, math.radians(25.0), 4.0, math.pi, 4.0, 2.0, 2.0)`.
- Normalized observations are clipped to `[-10, 10]` after replacing NaN/Inf with finite values.

Observation construction source:
- Raw state terms are computed in `PureNNBalanceEnv._state_terms()`.
- Pitch comes from the IMU-like projected gravity vector `self.bno080.data.projected_gravity_b` via `pitch_from_projected_gravity()`.
- Pitch rate comes from the IMU-like body angular velocity `self.bno080.data.ang_vel_b[:, 0]`, negated in `_state_terms()`.
- Yaw is computed from root quaternion using `yaw_from_quat_wxyz(self.robot.data.root_quat_w)` and compared to a reset-time yaw reference.
- Previous currents are the previous processed motor current commands from `self._action_processor.command_current`.

| Index | Observation | Meaning | Unit before normalization | Normalization scale | Source in code |
|---:|---|---|---|---|---|
| 0 | `x_rel` | Relative forward wheel-derived position from wheel joint positions | m | `1.0` | `PureNNBalanceEnv._state_terms()`: `0.5 * wheel_pos.sum() * R_WHEEL` |
| 1 | `linear_velocity` | Forward wheel-derived linear velocity | m/s | `1.0` | `PureNNBalanceEnv._state_terms()`: `0.5 * wheel_vel.sum() * R_WHEEL` |
| 2 | `pitch` | Body pitch estimated from projected gravity, with optional bias/noise in observation | rad | `math.radians(25.0)` | `pitch_from_projected_gravity(self.bno080.data.projected_gravity_b)` |
| 3 | `pitch_rate` | Body pitch angular rate, with optional noise in observation | rad/s | `4.0` | `-self.bno080.data.ang_vel_b[:, 0]` |
| 4 | `yaw_error` | Heading error relative to yaw at reset | rad | `math.pi` | `wrap_angle_rad(yaw_from_quat_wxyz(root_quat) - self._yaw_reference)` |
| 5 | `yaw_rate` | World-frame yaw rate | rad/s | `4.0` | `self.robot.data.root_ang_vel_w[:, 2]` |
| 6 | `previous_left_current` | Previous processed left motor current command | A | `2.0` | `self._action_processor.command_current[:, 0]` |
| 7 | `previous_right_current` | Previous processed right motor current command | A | `2.0` | `self._action_processor.command_current[:, 1]` |

Action dimension: `2`.

Action interpretation:
- The policy outputs two unconstrained/raw action values from the RSL-RL actor.
- In simulation, `CurrentActionProcessor.process()` applies `tanh()` to the raw actions and scales by `cfg.i_max_a`.
- The deployment export wrapper `CurrentPolicyWrapper.forward()` also applies `torch.tanh(actor(obs)) * i_max_a`.
- The training-time default network current envelope is `i_max_a = 2.0 A`.
- The deployed ONNX wrapper default is also `--i-max-a 2.0`.
- After network scaling, simulated motor current is further affected by optional smoothing/slew safety settings, motor gain, bias, deadzone, randomized current saturation, and current-loop response lag.

| Index | Action | Meaning | Unit after scaling | Scaling |
|---:|---|---|---|---|
| 0 | `I_left` | Left wheel current command | A | `tanh(action[0]) * i_max_a`, default `i_max_a = 2.0 A` |
| 1 | `I_right` | Right wheel current command | A | `tanh(action[1]) * i_max_a`, default `i_max_a = 2.0 A` |

## 3.3 Neural network architecture

Files:
- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/agents/rsl_rl_pure_nn_balance_cfg.py`
- `scripts/export_pure_nn_current_onnx.py`
- `scripts/rsl_rl/play.py`

Implemented actor configuration:
- Input neurons: `8`.
- Output neurons: `2`.
- Hidden layers: `[32, 32]`.
- Hidden activation: `relu`.
- Actor observation normalization in RSL-RL: disabled with `actor_obs_normalization=False` because observations are manually normalized by `NormalizedObservationBuilder`.
- Critic observation normalization: disabled with `critic_obs_normalization=False`.
- Actor architecture line: `8 -> 32 -> 32 -> 2`.

Output activation:
- The RSL-RL actor output itself is not documented in this code as having a built-in final `tanh` activation.
- The implemented controller applies `tanh` after the actor output in `CurrentActionProcessor.process()` during simulation.
- The inference-ready ONNX export applies `tanh` in `CurrentPolicyWrapper.forward()`.

Parameter count:
- Actor-only dense-layer parameter count for `8 -> 32 -> 32 -> 2` is `1410` parameters.
- Calculation: `(8 * 32 + 32) + (32 * 32 + 32) + (32 * 2 + 2) = 288 + 1056 + 66 = 1410`.
- RSL-RL also trains a critic network with hidden layers `[32, 32]`; the critic is not part of the deployed current-output actor wrapper.

Export paths and files:
- `scripts/rsl_rl/play.py` exports RSL-RL policy artifacts to `<run_dir>/exported/policy.pt` and `<run_dir>/exported/policy.onnx`.
- `scripts/export_pure_nn_current_onnx.py` exports the inference-ready current-output ONNX model to the path supplied by `--output`.
- The curriculum script uses `logs/rsl_rl/pure_nn_balance_two_wheel/<run>/exported/policy_current.onnx` as the current-output ONNX filename.

ONNX and STM32Cube.AI compatibility:
- The intended deployed actor uses fully connected layers with ReLU activations plus final `Tanh` and scalar multiply by `I_max`.
- These operations are expected to map to simple ONNX operators such as `Gemm`/`MatMul`, `Add`, `Relu`, `Tanh`, and `Mul`.
- No trained `policy_current.onnx` artifact was found in the checked codebase, so the actual exported graph operators were not inspected here.
- ONNX numerical validation is implemented in `scripts/export_pure_nn_current_onnx.py` and `scripts/validate_onnx_policy.py` with default tolerance `1e-4`.

## 3.4 Filtering and safety limits

Implemented filtering and current limits:
- Network current envelope: `I_network = tanh(action) * i_max_a`, default `i_max_a = 2.0 A`.
- Action delay: randomized as 0 or 1 sample in `CurrentActionProcessor.reset()` via `action_delay_samples`.
- Low-pass smoothing: configurable as `target = alpha * filtered_current + (1 - alpha) * delayed_current`; default training uses `alpha = 0.0`.
- Slew-rate limit: optional hardware-safety mode clamps `delta` to `±0.3 A` per control sample; default training leaves `delta` unclamped.
- Motor gain randomization: left and right gains are independently sampled from `(0.8, 1.2)`.
- Motor current bias: sampled from `(-0.08, 0.08) A`.
- Motor deadzone: sampled from `(0.03, 0.20) A`.
- Electrical/current-loop response lag: first-order lag with time constant sampled from `(0.005, 0.020) s`; this is not a measured mechanical time constant.
- Current saturation: randomized per motor from `(1.6, 2.4) A`.
- Torque conversion: `tau = I_cmd * DDSM115_KT`, where `DDSM115_KT = 0.75 Nm/A`.
- Torque-speed limiting: implemented using `DDSM115_TAU_PEAK = 2.0 Nm` and `DDSM115_NO_LOAD_SPEED = 200 rpm` converted to rad/s.

Episode safety and termination:
- Maximum pitch: episode terminates when `abs(pitch) > floor_stop_pitch_deg`, default `25 deg`.
- Maximum position: episode terminates when `abs(x_rel) > position_stop_m`, default `1.0 m`.
- Timeout: episode terminates when `episode_length_buf >= max_episode_length - 1`, with default `episode_length_s = 8.0 s`.
- Physics-broken condition: episode terminates if `body_z < -1.0`.

Emergency stop condition:
- Not currently implemented as a separate hardware emergency-stop routine in the pure NN task.

Hardware safety-specific limits:
- The controller includes current saturation, optional slew-rate limiting, torque-speed limiting, pitch termination, and position termination in simulation.
- A separate hardware emergency stop in firmware is not documented in the pure NN controller implementation inspected here.

## Section 4: Training of the Controller in Simulation

## 4.1 Simulation environment

Simulator:
- Isaac Lab / Isaac Sim, using `DirectRLEnv` through the inherited `StandupEnv` base class.

Robot model:
- USD asset path is defined in `robot_cfg.py` as `source/TwoWheeledRobot/docs/ColectedUSD_v2/World0.usd`.
- The articulation config is `TWO_WHEELED_ROBOT_CFG` in `robot_cfg.py`.

Environment class:
- `PureNNBalanceEnv` in `pure_nn_balance_env.py`.
- It inherits from `StandupEnv` in `standup_env.py`.

Task registration:
- Gym task ID: `Template-Twowheeledrobot-PureNNBalance-v0`.
- Registered in `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/__init__.py`.

Sample time:
- Physics time step: `PHYSICS_DT = 1.0 / 1000.0 s`.
- Control decimation: `CONTROL_DECIMATION = 20`.
- Control sample time: `0.02 s`.
- Control frequency: `50 Hz`.

Episode length:
- `episode_length_s = 8.0 s`.
- At 50 Hz, this corresponds to `8.0 / 0.02 = 400` controller steps per episode.

Termination:
- `abs(pitch) > 25 deg`.
- `abs(x_rel) > 1.0 m`.
- `body_z < -1.0` as a physics-broken guard.
- Timeout after approximately 400 controller steps.

Reset behavior:
- `PureNNBalanceEnv._reset_idx()` starts the robot near upright.
- Root height is set to `spawn_upright_z` inherited from `StandupEnvCfg`, default `0.06859 m`.
- Yaw reference is recorded at reset.
- CyberGear joints are set to zero position and zero velocity.
- Wheel velocities are initialized from sampled forward velocity as `wheel_omega = velocity / R_WHEEL`.
- Previous actions, observation delay buffers, currents, disturbance force, and episode reward are reset to zero.

Initial state randomization:
- Stage 1: pitch sampled uniformly from `±3 deg`, pitch rate from `±0.2 rad/s`, velocity from `±0.05 m/s`.
- Stages 2 through 5: pitch sampled uniformly from `±8 deg`, pitch rate from `±0.8 rad/s`, velocity from `±0.15 m/s`.
- Implemented in `CurriculumSampler.reset_ranges()`.

Motor model:
- The pure NN task uses the DDSM115 current-to-torque abstraction directly in `PureNNBalanceEnv._pre_physics_step()`.
- Torque constant: `DDSM115_KT = 0.75 Nm/A`.
- Peak torque: `DDSM115_TAU_PEAK = 2.0 Nm`.
- No-load speed: `DDSM115_NO_LOAD_SPEED = 200 rpm` converted to rad/s.
- Torque-speed saturation is applied as a linear reduction to zero at no-load speed.

Real motor nonlinearities included:
- Current deadzone: implemented.
- Current bias: implemented.
- Left/right gain asymmetry: implemented.
- First-order motor response lag: implemented.
- Current saturation: implemented.
- Torque-speed limiting: implemented.

Latency modeled:
- Observation delay: implemented as 0 or 1 sample in `PureNNBalanceEnv._get_observations()` using `_obs_delay_samples`.
- Action delay: implemented as 0 or 1 sample in `CurrentActionProcessor.process()` using `action_delay_samples`.

Floor/contact model:
- A flat ground plane is spawned in `StandupEnv._setup_scene()`.
- Ground friction constants used by the ground plane are `GROUND_STATIC_FRICTION = 0.6` and `GROUND_DYNAMIC_FRICTION = 0.4` in `sim_params.py`.

## 4.2 Reward function

Implemented reward source:
- `BalanceReward.compute()` in `pure_nn_components.py`.
- Called from `PureNNBalanceEnv._get_rewards()`.

Mathematical expression:

```latex
r = w_0
- w_\theta \theta^2
- w_{\dot\theta} \dot\theta^2
- w_v v^2
- w_x x^2
- w_\psi \psi^2
- w_I (I_L^2 + I_R^2)
- w_{\Delta I} (\Delta I_L^2 + \Delta I_R^2)
```

Actual implemented equation in code form:

```text
reward = rew_alive
       - rew_pitch * pitch^2
       - rew_pitch_rate * pitch_rate^2
       - rew_velocity * velocity^2
       - rew_position * position^2
       - rew_yaw_error * yaw_error^2
       - rew_current * (I_left^2 + I_right^2)
       - rew_delta_current * (Delta_I_left^2 + Delta_I_right^2)
```

Reward post-processing:
- `torch.nan_to_num(reward, nan=0.0, posinf=1.0, neginf=-100.0)`.
- Reward is clamped to `[-100.0, 1.0]`.

| Term | Symbol | Config field | Value |
|---|---|---|---:|
| Survival/alive reward | `w_0` | `rew_alive` | `1.0` |
| Pitch penalty | `w_theta` | `rew_pitch` | `12.0` |
| Pitch-rate penalty | `w_dtheta` | `rew_pitch_rate` | `0.8` |
| Velocity penalty | `w_v` | `rew_velocity` | `0.15` |
| Position penalty | `w_x` | `rew_position` | `0.4` |
| Yaw-error penalty | `w_psi` | `rew_yaw_error` | `0.1` |
| Current penalty | `w_I` | `rew_current` | `0.01` |
| Current-change penalty | `w_DeltaI` | `rew_delta_current` | `0.03` |

Special reward terms:
- No separate sparse success bonus is used by `PureNNBalanceEnv`.
- No explicit fall penalty is used beyond episode termination and the reward clamp.
- No contact-force reward is used.

## 4.3 Disturbances during training

Disturbance implementation:
- `DisturbanceGenerator` in `pure_nn_components.py`.
- Sampled in `PureNNBalanceEnv._reset_idx()`.
- Applied in `PureNNBalanceEnv._pre_physics_step()` when `curriculum_stage >= 3`.

Force application:
- A body is selected by `_resolve_push_body_ids()` using the first matching pattern from `.*Platform.*`, `.*MainAssembly.*`, then `.*`.
- Force is applied through `self.robot.set_external_force_and_torque()`.
- Direction is `self._body_force[:, 0, 0] = force`, i.e. along the local/world tensor x-axis used by Isaac for the selected body force tensor.
- Torque disturbance is zero.
- Disturbance is applied as a single scalar force to the selected body, not as separate left/right wheel or asymmetric wheel commands.

Curriculum disturbance distributions:

| Stage | Disturbance type | Probability | Magnitude | Duration | Time | Notes |
|---:|---|---:|---|---|---|---|
| 1 | none | 100% | `0 N` | n/a | n/a | No disturbances |
| 2 | none | 100% | `0 N` | n/a | n/a | Larger reset randomization only |
| 3 | none | 50% | `0 N` | n/a | n/a | No external disturbance |
| 3 | single human push | 40% | `Fx [-6,6] N`, `Fy [-1.5,1.5] N`, `Mz [-0.15,0.15] Nm` | `[0.05,0.20] s` | start `[1,6] s` | Short hand bump/push |
| 3 | double human push | 10% | Two independently sampled human pushes | Each `[0.05,0.20] s` | each start `[1,6] s` | Two separate push events |
| 4 | none | 40% | `0 N` | n/a | n/a | No external disturbance |
| 4 | payload / COM-shift | 40% | pitch torque `Mx [-0.25,0.25] Nm` | until episode end | start `[1,4] s` | Persistent book/payload equivalent |
| 4 | payload + human push | 20% | payload torque plus one human push | payload persistent, push `[0.05,0.20] s` | payload start `[1,4] s`, push start `[1,6] s` | Combined realistic event |
| 5 | none | 30% | `0 N` | n/a | n/a | Implemented with `kind = 0` |
| 5 | human push | 30% | `Fx [-6,6] N`, `Fy [-1.5,1.5] N`, `Mz [-0.15,0.15] Nm` | `[0.05,0.20] s` | start `[1,6] s` | Short hand bump/push |
| 5 | payload / COM-shift | 25% | pitch torque `Mx [-0.25,0.25] Nm` | until episode end | start `[1,4] s` | Persistent payload equivalent |
| 5 | payload + human push | 15% | payload torque plus one human push | payload persistent, push `[0.05,0.20] s` | payload start `[1,4] s`, push start `[1,6] s` | Combined final robustness |
| 5 optional | slope-equivalent | low probability only if enabled | `Fx [-1.5,1.5] N` | entire episode | start `0 s` | Disabled by default |

Missing or differing disturbance details:
- Full dynamic payload/mass insertion is not implemented; the payload is represented by a persistent pitch-axis torque step.
- Sine disturbance is not part of the default training curriculum. It remains available as a diagnostic benchmark scenario only.
- Disturbance components are passed directly to Isaac Lab's external force/torque tensor API without frame conversion; comments in `PureNNBalanceEnv._pre_physics_step()` define the task convention.

## 4.4 Domain randomization

Implemented domain randomization:

| Parameter | Range | Applied where | Notes |
|---|---|---|---|
| Initial pitch, Stage 1 | `±3 deg` | `CurriculumSampler.reset_ranges()` and `PureNNBalanceEnv._reset_idx()` | Uniform reset randomization |
| Initial pitch rate, Stage 1 | `±0.2 rad/s` | `CurriculumSampler.reset_ranges()` and `PureNNBalanceEnv._reset_idx()` | Uniform reset randomization |
| Initial velocity, Stage 1 | `±0.05 m/s` | `CurriculumSampler.reset_ranges()` and `PureNNBalanceEnv._reset_idx()` | Uniform reset randomization |
| Initial pitch, Stages 2-5 | `±8 deg` | `CurriculumSampler.reset_ranges()` and `PureNNBalanceEnv._reset_idx()` | Uniform reset randomization |
| Initial pitch rate, Stages 2-5 | `±0.8 rad/s` | `CurriculumSampler.reset_ranges()` and `PureNNBalanceEnv._reset_idx()` | Uniform reset randomization |
| Initial velocity, Stages 2-5 | `±0.15 m/s` | `CurriculumSampler.reset_ranges()` and `PureNNBalanceEnv._reset_idx()` | Uniform reset randomization |
| Left motor gain | `(0.8, 1.2)` | `CurrentActionProcessor.reset()` | Independent left motor gain |
| Right motor gain | `(0.8, 1.2)` | `CurrentActionProcessor.reset()` | Independent right motor gain |
| Motor deadzone | `(0.03, 0.20) A` | `CurrentActionProcessor.reset()` and `.process()` | Per-motor deadzone |
| Current bias | `(-0.08, 0.08) A` | `CurrentActionProcessor.reset()` and `.process()` | Per-motor bias |
| Motor current-loop time constant | `(0.005, 0.020) s` | `CurrentActionProcessor.reset()` and `.process()` | Per-motor electrical/current-loop response lag, sampled once per episode |
| Current limit | `(1.6, 2.4) A` | `CurrentActionProcessor.reset()` and `.process()` | Per-motor current saturation |
| Pitch bias | `±0.5 deg` | `PureNNBalanceEnv._reset_idx()` and `_get_observations()` | Added to observed pitch |
| Pitch noise | `0.15 deg` standard deviation | `PureNNBalanceEnv._get_observations()` | Gaussian observation noise |
| Pitch-rate noise | `0.02 rad/s` standard deviation | `PureNNBalanceEnv._get_observations()` | Gaussian observation noise |
| Velocity noise | `0.01 m/s` standard deviation | `PureNNBalanceEnv._get_observations()` | Gaussian observation noise |
| Observation delay | `0` or `1` sample | `PureNNBalanceEnv._reset_idx()` and `_get_observations()` | Per-environment random delay |
| Action delay | `0` or `1` sample | `CurrentActionProcessor.reset()` and `.process()` | Per-environment random delay |

Configured but not currently applied randomization:

| Parameter | Configured range | Applied where | Notes |
|---|---|---|---|
| Body mass scale | `(0.9, 1.1)` | Not currently implemented | Present in `PureNNBalanceEnvCfg`, but no active application in `PureNNBalanceEnv` |
| Center-of-mass height scale | `(0.9, 1.1)` | Not currently implemented | Present in config only |
| Body pitch inertia scale | `(0.8, 1.2)` | Not currently implemented | Present in config only; previous PhysX hook was disabled due tensor-size errors |
| Wheel radius scale | `(0.98, 1.02)` | Not currently implemented | Present in config only; `R_WHEEL` is fixed from `residual_lqr_env.py` |
| Ground static friction | `(0.7, 1.1)` | Not currently implemented for pure NN randomization | Ground plane uses fixed `GROUND_STATIC_FRICTION = 0.6` |
| Ground dynamic friction | `(0.6, 1.0)` | Not currently implemented for pure NN randomization | Ground plane uses fixed `GROUND_DYNAMIC_FRICTION = 0.4` |

## Training pipeline and export

Training entry points:
- General training script: `scripts/rsl_rl/train.py`.
- Pure NN curriculum wrapper: `scripts/train_pure_nn_curriculum.py`.
- Curriculum command uses task `Template-Twowheeledrobot-PureNNBalance-v0` and passes `env.curriculum_stage=<stage>`.

Default curriculum iterations in `scripts/train_pure_nn_curriculum.py`:
- Stage 1: `1000` iterations.
- Stage 2: `1000` iterations.
- Stage 3: `1500` iterations.
- Stage 4: `1500` iterations.
- Stage 5: `2000` iterations.

PPO training configuration:
- `num_steps_per_env = 64`.
- `max_iterations = 2000` in `PureNNBalancePPORunnerCfg`; curriculum wrapper overrides per stage.
- `save_interval = 100`.
- `experiment_name = "pure_nn_balance_two_wheel"`.
- `init_noise_std = 0.2`.
- `entropy_coef = 0.002`.
- `learning_rate = 5.0e-4`.
- `gamma = 0.99`.
- `lam = 0.95`.
- `clip_param = 0.2`.
- `num_learning_epochs = 4`.
- `num_mini_batches = 4`.
- `desired_kl = 0.01`.
- `max_grad_norm = 0.5`.

Export and validation:
- `scripts/rsl_rl/play.py` exports `policy.pt` and `policy.onnx`.
- `scripts/export_pure_nn_current_onnx.py` exports `policy_current.onnx` with current output in amperes.
- ONNX validation compares PyTorch/TorchScript and ONNX Runtime outputs on identical random observations.
- Validation fails if maximum absolute error is `>= 1e-4`.

Benchmark script:
- `scripts/benchmark_pure_nn_balance.py` evaluates scenarios: no disturbance, single human push, double human push, payload/COM shift, payload plus push, sine diagnostic, slope-equivalent, and randomized motor parameters.
- Metrics reported: survival time, maximum pitch, RMS pitch, RMS pitch rate, RMS current, maximum current, final position error, and recovery time.
- Recovery condition: after a disturbance, `abs(pitch) < 2 deg` and `abs(pitch_rate) < 0.2 rad/s` for at least `0.5 s`.

## Paper-ready summary

- The proposed controller is a compact feed-forward neural policy trained with PPO in Isaac Lab and registered as `Template-Twowheeledrobot-PureNNBalance-v0`.
- The actor uses an `8 -> 32 -> 32 -> 2` fully connected ReLU architecture; the deployed actor has approximately `1410` dense-layer parameters before including framework/export metadata.
- The observation vector contains wheel-derived relative position and velocity, IMU-like pitch and pitch rate, yaw error, yaw rate, and the previous left/right current commands.
- The policy outputs two independent wheel commands that are converted directly to motor currents using `tanh(output) * I_max`, with default `I_max = 2.0 A`.
- The simulated actuator path includes 0/1-sample action delay, optional current smoothing/slew safety settings, motor gain asymmetry, deadzone, bias, current saturation, sampled current-loop response lag, and DDSM115 torque-speed limiting.
- Training uses 50 Hz control, 8 s episodes, near-upright reset randomization, and termination at `25 deg` pitch or `1 m` position error.
- The reward is a quadratic balance objective with alive reward and penalties on pitch, pitch rate, velocity, position, yaw error, current, and current change.
- The controller was trained with physically motivated sim2real disturbances: initial state perturbations, short human-push disturbances, payload/center-of-mass-shift equivalent pitch torques, motor nonlinearities and asymmetry, observation/action delay, and sensor noise.
- Sine disturbance is retained only as an optional diagnostic benchmark, not as part of the default training curriculum; mass/COM/inertia/wheel-radius/floor-friction ranges are configured but not currently applied in the pure NN environment.

## Figures needed for the paper

- Controller block diagram showing normalized observations, neural network, `tanh * I_max`, optional smoothing/slew safety settings, motor nonlinearities, current-loop lag, and wheel torque application.
- Training pipeline diagram: Isaac Lab simulation, PPO training, curriculum stages, checkpoint, TorchScript export, ONNX current wrapper, STM32Cube.AI deployment.
- Disturbance examples over time: single human push, double human push, persistent payload torque, payload plus push, optional slope bias, sine diagnostic.
- Reward component diagram or stacked plot showing pitch, pitch-rate, position, current, and current-change penalties.
- Domain-randomization diagram separating implemented motor/sensor/latency randomization from configured-but-not-applied physical randomization.

## Tables needed for the paper

- Observation and action space table.
- Neural network architecture and parameter-count table.
- Reward weight table.
- Curriculum and disturbance parameter table.
- Domain randomization table.
- Safety and actuator post-processing table.
