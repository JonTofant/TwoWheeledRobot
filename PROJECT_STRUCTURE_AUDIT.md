# Project Structure Audit

## 1. Repository Tree

Relevant source, configuration, documentation, scripts, and firmware files:

```text
.
├── README.md
├── STM32_DEPLOYMENT.md
├── DDSM115_TORQUE_SPEED_LIMITER_AUDIT.md
├── LQR_MODEL_SUSPENDED_TEST_REPORT.md
├── LQR_YAW_DRIFT_TEST_REPORT.md
├── pyproject.toml
├── scripts/
│   ├── rsl_rl/
│   │   ├── train.py
│   │   ├── play.py
│   │   └── cli_args.py
│   ├── lqr_control.py
│   ├── calculate_lqr_gains.py
│   ├── run_lqr_disturbance_sweep.py
│   ├── run_lqr_vs_residual_repeatability.py
│   ├── run_step_disturbance_benchmark.py
│   ├── plot_control_effort_lqr_vs_residual.py
│   ├── plot_single_run_control_effort.py
│   ├── analyze_ddsm115_torque_speed_audit.py
│   ├── analyze_lqr_yaw.py
│   ├── test_policy_angle_sweep.py
│   ├── uart_policy_runner.py
│   ├── list_envs.py
│   └── tools/
│       └── extract_robot_physical_parameters.py
├── source/
│   └── TwoWheeledRobot/
│       ├── setup.py
│       ├── pyproject.toml
│       ├── config/
│       │   └── extension.toml
│       ├── docs/
│       │   ├── CHANGELOG.rst
│       │   ├── ColectedUSD_v2/
│       │   │   ├── World0.usd
│       │   │   └── SubUSDs/*.usd
│       │   └── ColectedUSD_v2_old/
│       │       ├── World0.usd
│       │       └── SubUSDs/*.usd
│       └── TwoWheeledRobot/
│           ├── __init__.py
│           └── tasks/
│               └── direct/
│                   └── twowheeledrobot/
│                       ├── __init__.py
│                       ├── robot_cfg.py
│                       ├── sim_params.py
│                       ├── standup_env_cfg.py
│                       ├── standup_env.py
│                       ├── residual_lqr_env_cfg.py
│                       ├── residual_lqr_env.py
│                       └── agents/
│                           ├── __init__.py
│                           ├── rsl_rl_standup_cfg.py
│                           └── rsl_rl_residual_lqr_cfg.py
└── RealImplementationCode/
    ├── main.c
    ├── controler.c
    ├── DDSM115.c
    ├── cybergear.c
    ├── StateEstimator.c
    ├── state_machine.c
    ├── telemetry.c
    ├── kinematics.c
    └── other STM32 support files
```

Ignored from this tree: generated `logs/`, `outputs/`, checkpoints, Python caches, build artifacts, and `.git`.

## 2. Main Entry Points

### `scripts/rsl_rl/train.py`

- Purpose: train an RSL-RL PPO agent for a registered Isaac Lab task.
- Example command:

```bash
python scripts/rsl_rl/train.py --task Template-Twowheeledrobot-Standup-v0 --headless --num_envs 4096
python scripts/rsl_rl/train.py --task Template-Twowheeledrobot-ResidualLQR-v0 --headless --num_envs 4096
```

- Required arguments: `--task` is effectively required because the Hydra task config is selected from it.
- Optional important arguments: `--num_envs`, `--seed`, `--max_iterations`, `--resume`, `--load_run`, `--checkpoint`, `--experiment_name`, `--run_name`, `--logger`, `--video`, `--headless`, `--device`, `--distributed`.
- Main calls: registers `TwoWheeledRobot.tasks`, loads task/agent config through `hydra_task_config`, creates `gym.make(args_cli.task, cfg=env_cfg)`, wraps with `RslRlVecEnvWrapper`, trains `OnPolicyRunner` or `DistillationRunner`, writes `env.yaml` and `agent.yaml`.

### `scripts/rsl_rl/play.py`

- Purpose: play/evaluate an RSL-RL checkpoint and export the trained policy to TorchScript and ONNX.
- Example command:

```bash
python scripts/rsl_rl/play.py --task Template-Twowheeledrobot-Standup-v0 --num_envs 1
python scripts/rsl_rl/play.py --task Template-Twowheeledrobot-ResidualLQR-v0 --num_envs 1 --checkpoint logs/rsl_rl/residual_lqr_two_wheel/<run>/model_<iter>.pt
```

- Required arguments: `--task`; checkpoint is optional because it can auto-resolve from the experiment log folder.
- Optional important arguments: `--checkpoint`, `--load_run`, `--use_pretrained_checkpoint`, `--num_steps`, `--real-time`, `--video`, `--headless`.
- Main calls: loads RSL-RL runner, gets inference policy, exports `exported/policy.pt` and `exported/policy.onnx`, steps the environment.

### `scripts/lqr_control.py`

- Purpose: fixed-base DDSM115 free-spin diagnostics, suspended LQR model sign tests, floor-contact LQR tests, external disturbance tests, residual-policy evaluation, live plotting, and CSV logging.
- Example commands:

```bash
python scripts/lqr_control.py --task Template-Twowheeledrobot-Standup-v0 --motor-test-current 0.25
python scripts/lqr_control.py --task Template-Twowheeledrobot-Standup-v0 --test-mode lqr-model --artificial-pitch-deg 1.0 --log-csv ''
python scripts/lqr_control.py --task Template-Twowheeledrobot-Standup-v0 --test-mode lqr-floor --floor-initial-pitch-deg 1.0 --no-plot
python scripts/lqr_control.py --task Template-Twowheeledrobot-Standup-v0 --test-mode lqr-floor --actuator-disturbance physical-forward --actuator-disturbance-current-a 1.0 --no-plot
```

- Required arguments: none strictly; defaults to task `Template-Twowheeledrobot-Standup-v0`, one env, and `free-spin`.
- Optional important arguments: `--test-mode {free-spin,lqr-model,lqr-floor}`, `--motor-test-current`, `--left-motor-test-current`, `--right-motor-test-current`, `--floor-initial-pitch-deg`, `--floor-motors-disabled`, `--max-test-time-s`, `--disturbance`, `--actuator-disturbance`, `--enable-residual-rl`, `--residual-policy`, `--residual-action-limit`, `--log-csv`, `--no-plot`, `--list-signals`.
- Main calls/classes/functions: `LqrPhysicalParams`, `LqrWeights`, `calculate_lqr_gains`, `calculate_lqr6_gains`, `compute_action`, `residual_current_from_policy`, `actuator_disturbance_current`, `disturbance_wrench`, `CsvLogger`, `LivePlot`, `sample_to_row`.

### `scripts/calculate_lqr_gains.py`

- Purpose: calculate 4-state pitch LQR gains for manual insertion into `scripts/lqr_control.py`.
- Example command:

```bash
python scripts/calculate_lqr_gains.py
```

- Required arguments: none.
- Optional important arguments: none currently exposed; constants are edited in the script.
- Main calls/classes/functions: `RobotParams`, `LqrWeights`, `build_pitch_model`, `lqr`, `force_gain_to_normalized_action_gain`, `print_results`.

### `scripts/run_lqr_disturbance_sweep.py`

- Purpose: run a pulse-style actuator-current disturbance sweep and summarize recovery metrics.
- Example command:

```bash
python scripts/run_lqr_disturbance_sweep.py --headless --currents 0.5 1.0 1.5 2.0
```

- Required arguments: none.
- Optional important arguments: `--currents`, `--output-dir`, `--enable-residual-rl`, `--residual-policy`, `--residual-action-limit`, `--headless`, `--real-time`.
- Main calls: spawns `scripts/lqr_control.py` with `--test-mode lqr-floor --actuator-disturbance physical-forward --actuator-disturbance-samples 10`, reads CSVs with pandas, writes `summary.csv` and PNG plots.

### `scripts/run_lqr_vs_residual_repeatability.py`

- Purpose: repeat LQR and LQR+residual-PPO pulse disturbance tests and aggregate pitch/current metrics.
- Example command:

```bash
python scripts/run_lqr_vs_residual_repeatability.py --headless --residual-policy logs/rsl_rl/residual_lqr_two_wheel/<run>/exported/policy.pt
```

- Required arguments: none, but `--residual-policy` is needed for a real frozen residual PPO comparison.
- Optional important arguments: `--repeats`, `--currents`, `--output-dir`, `--residual-policy`, `--residual-action-limit`, `--no-plot`, `--headless`.
- Main calls: imports `run_lqr_disturbance_sweep`, calls its `run_simulation` and `compute_metrics`, writes per-run and aggregate CSVs, plots comparisons.

### `scripts/run_step_disturbance_benchmark.py`

- Purpose: compare LQR and residual PPO under step actuator-current disturbances.
- Example command:

```bash
python scripts/run_step_disturbance_benchmark.py --headless --residual-policy logs/rsl_rl/residual_lqr_two_wheel/<run>/exported/policy.pt
```

- Required arguments: none, but `--residual-policy` is needed for residual PPO.
- Optional important arguments: `--repeats`, `--currents`, `--output-dir`, `--actuator-disturbance-start-s`, `--actuator-disturbance-duration-s`, `--selected-current-a`, `--residual-action-limit`, `--no-plot`, `--headless`.
- Main calls: spawns `scripts/lqr_control.py` with `--actuator-disturbance step-forward`, computes during-step and after-step metrics, writes raw CSVs plus summaries and plots.

### Plotting and Analysis Scripts

- `scripts/plot_control_effort_lqr_vs_residual.py`: compares LQR vs residual current effort from benchmark CSVs or a per-run summary. Requires `--output` and either `--summary` or `--lqr-csv` plus `--ppo-csv`.
- `scripts/plot_single_run_control_effort.py`: plots one raw run CSV; requires positional `csv_path`.
- `scripts/analyze_ddsm115_torque_speed_audit.py`: validates torque-speed limiter equations from one or more CSVs; requires one or more CSV paths.
- `scripts/analyze_lqr_yaw.py`: analyzes yaw repeatability from one or more LQR floor-contact CSV logs; requires one or more CSV paths.

### Hardware and Utility Scripts

- `scripts/uart_policy_runner.py`: runs an exported standup `policy.pt` over UART. Requires `--policy` and `--port`; optional `--baud`, `--degrees`, `--rate-hz`, `--safe-on-error`.
- `scripts/test_policy_angle_sweep.py`: host-only synthetic roll/pitch policy sanity check. Requires `--policy`.
- `scripts/tools/extract_robot_physical_parameters.py`: launches Isaac Lab, reads PhysX mass properties, writes `outputs/robot_physical_parameters.csv`. Defaults to `Template-Twowheeledrobot-Standup-v0`.
- `scripts/list_envs.py`: lists registered Isaac Lab environments, optionally filtered by `--keyword`.

## 3. Isaac Sim / Isaac Lab Environment

Two Gym tasks are registered in `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/__init__.py`:

- `Template-Twowheeledrobot-Standup-v0` -> `standup_env:StandupEnv`, configured by `standup_env_cfg:StandupEnvCfg`, trained by `agents/rsl_rl_standup_cfg:StandupPPORunnerCfg`.
- `Template-Twowheeledrobot-ResidualLQR-v0` -> `residual_lqr_env:ResidualLqrEnv`, configured by `residual_lqr_env_cfg:ResidualLqrEnvCfg`, trained by `agents/rsl_rl_residual_lqr_cfg:ResidualLqrPPORunnerCfg`.

### Standup Environment

- Environment class: `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/standup_env.py`, class `StandupEnv(DirectRLEnv)`.
- Config class: `standup_env_cfg.py`, class `StandupEnvCfg(DirectRLEnvCfg)`.
- Observation definition: `_get_observations()` builds 18 values:
  - projected gravity body vector, 3 values;
  - body angular velocity divided by `10.0`, 3 values;
  - CyberGear normalized extension fractions, 4 values;
  - DDSM115 wheel velocities divided by `wheel_velocity_norm`, 2 values;
  - previous action, 6 values.
- Action definition: `_pre_physics_step()` consumes 6 normalized actions:
  - actions `0:4` map to absolute CyberGear joint position targets inside mechanical limits;
  - actions `4:6` map to DDSM115 desired currents through `wheel_current_max`.
- Reset logic: `_reset_idx()` clears buffers, samples fallen poses through `_sample_fallen_poses()`, writes root pose/velocity, zeros CyberGear joints, sets joint stiffness/damping/friction randomization.
- Simulation timestep: `PHYSICS_DT = 0.001 s` from `sim_params.py`; control decimation is `20`, so control timestep is `0.020 s` / `50 Hz`.
- Robot articulation loading: `robot_cfg.py` defines `TWO_WHEELED_ROBOT_CFG` with USD path `source/TwoWheeledRobot/docs/ColectedUSD_v2/World0.usd`; `standup_env_cfg.py` installs it at `/World/envs/env_.*/Robot`.
- Sensors: IMU `bno080` is configured in `standup_env_cfg.py` and read in `standup_env.py`; optional wheel contact sensor exists in config but is disabled by default.
- Termination conditions: `_get_dones()` terminates on success counter or physics-broken body height; timeout at max episode length. The standup task intentionally does not terminate on ordinary falls.

### Residual LQR Environment

- Environment class: `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/residual_lqr_env.py`, class `ResidualLqrEnv(StandupEnv)`.
- Config class: `residual_lqr_env_cfg.py`, class `ResidualLqrEnvCfg(StandupEnvCfg)`.
- Observation definition: `_get_observations()` builds 8 values:
  - pitch, pitch rate, base position, base velocity, two normalized wheel velocities, two previous residual currents normalized by `residual_action_limit`.
- Action definition: action space is 2; if `enable_residual_rl` is true, actions are clamped to `[-1, 1]` and scaled to residual current in amperes by `residual_action_limit`.
- Reset logic: calls `StandupEnv._reset_idx()`, then forces a near-upright floor-balancing spawn at `floor_initial_pitch_deg`, zeros CyberGear joints, resets LQR/residual/disturbance buffers, samples disturbance amplitude/timing.
- Simulation/control timestep: inherited from `StandupEnvCfg`, so `0.001 s` physics and `0.020 s` control by default.
- Robot articulation and sensors: inherited from `StandupEnv`.
- Termination conditions: `_get_dones()` terminates when absolute pitch exceeds `floor_stop_pitch_deg` or body height indicates broken physics; timeout at episode length.

## 4. Robot and Physical Parameters

### Main Runtime Simulation Parameters

| Name | Value | Unit | File path | Duplicated elsewhere |
|---|---:|---|---|---|
| `PHYSICS_DT` | `1.0 / 1000.0` | s | `sim_params.py` | Used by `standup_env_cfg.py` |
| `CONTROL_DECIMATION` | `20` | physics steps | `sim_params.py` | `scripts/lqr_control.py` has CLI default `--control-decimation 20` |
| `GROUND_STATIC_FRICTION` | `0.6` | coefficient | `sim_params.py` | Imported by `standup_env.py`; read in `lqr_control.py` |
| `GROUND_DYNAMIC_FRICTION` | `0.4` | coefficient | `sim_params.py` | Imported by `standup_env.py`; read in `lqr_control.py` |
| `GROUND_RESTITUTION` | `0.0` | coefficient | `sim_params.py` | Imported by `standup_env.py` |
| `LINEAR_DAMPING` | `0.0` | damping | `sim_params.py` | Imported by `robot_cfg.py` |
| `ANGULAR_DAMPING` | `0.0` | damping | `sim_params.py` | Imported by `robot_cfg.py` |
| `WHEEL_DRIVE_STIFFNESS` | `0.0` | stiffness | `sim_params.py` | Imported by `robot_cfg.py` |
| `WHEEL_INTERNAL_DAMPING` | `0.0` | Nm*s/rad | `sim_params.py` | Used in `robot_cfg.py`, `standup_env.py`; LQR scripts compute an analytical no-load damping separately |
| `BEARING_DAMPING` | `0.005` | Nm*s/rad | `sim_params.py` | Imported by `robot_cfg.py` |
| `CYBERGEAR_STIFFNESS` | `30.0` | Nm/rad | `sim_params.py` | `standup_env_cfg.py` also has fixed standup gains `60.0` |
| `CYBERGEAR_DAMPING` | `3.0` | Nm*s/rad | `sim_params.py` | `standup_env_cfg.py` also has fixed standup damping `4.0` |
| `SOLVER_POSITION_ITERS` | `4` | count | `sim_params.py` | Imported by `robot_cfg.py` |
| `SOLVER_VELOCITY_ITERS` | `1` | count | `sim_params.py` | Imported by `robot_cfg.py` |

### DDSM115 Parameters

| Name | Value | Unit | File path | Duplicated elsewhere |
|---|---:|---|---|---|
| `DDSM115_KT` | `0.75` | Nm/A | `sim_params.py` | `scripts/uart_policy_runner.py`, LQR dataclasses, docs |
| `DDSM115_I_CONT` | `1.5` | A | `sim_params.py` | docs mention validation current |
| `DDSM115_I_PEAK` | `2.7` | A | `sim_params.py` | imported by envs and `lqr_control.py`; docs |
| `DDSM115_TAU_RATED` | `0.96` | Nm | `sim_params.py` | `uart_policy_runner.py`, docs |
| `DDSM115_TAU_PEAK` | `2.0` | Nm | `sim_params.py` | `analyze_ddsm115_torque_speed_audit.py`, docs |
| `DDSM115_RATED_SPEED` | `115 rpm` converted to rad/s | rad/s | `sim_params.py` | not widely used |
| `DDSM115_NO_LOAD_SPEED` | `200 rpm` converted to rad/s | rad/s | `sim_params.py` | `uart_policy_runner.py`, analysis script, LQR dataclasses, docs |

### LQR Physical Parameters

| Name | Value | Unit | File path | Duplicated elsewhere |
|---|---:|---|---|---|
| `body_mass_kg` | `2.6` | kg | `residual_lqr_env.py`, `scripts/lqr_control.py`, `scripts/calculate_lqr_gains.py` | Yes, same concept in all three |
| `wheel_cart_mass_kg` | `1.53` | kg | same as above | Yes |
| `wheel_radius_m` | `0.05035` | m | same as above | Yes; extracted script can estimate a USD value |
| `body_com_height_m` | `0.14` | m | same as above | Yes |
| `body_pitch_inertia_kg_m2` | `0.0129596588636` | kg*m^2 | same as above | Yes |
| `wheel_torque_constant_nm_per_a` | `0.75` | Nm/A | same as above | Yes, duplicates `DDSM115_KT` |
| `track_width_m` | `0.382999941707` | m | `residual_lqr_env.py`, `scripts/lqr_control.py` | Yes |
| `body_yaw_inertia_kg_m2` | `0.0315051945189` | kg*m^2 | `residual_lqr_env.py`, `scripts/lqr_control.py` | Yes |
| `no_load_current_a` | `0.25` | A | LQR dataclasses | Yes |
| `gravity_m_s2` | `9.81` | m/s^2 | LQR dataclasses | Yes |

`scripts/tools/extract_robot_physical_parameters.py` is the current source for deriving PhysX mass properties from the USD. It writes link masses, COMs, inertias, combined non-wheel body mass, wheel mass, COM height above wheel axle, pitch inertia, track width, yaw inertia, wheel rotational inertia, and wheel radius to CSV.

### Standup Task Parameters

- Spawn and geometry: `spawn_upright_z = 0.06859 m`, `spawn_lateral_h = 0.13 m`, `spawn_clearance = 0.04 m`, `spawn_tilt_noise_std = 0.175 rad`.
- Spawn probabilities: right side, left side, forward, backward, upside down each `0.15`; partial fall `0.25`.
- Success thresholds: pitch `sin(15 deg)`, roll `sin(8 deg)`, `success_steps_required = 2`.
- CyberGear limits are hardcoded in `standup_env.py`: front-left/back-right `[-10 deg, +90 deg]`; front-right/back-left `[-90 deg, +10 deg]`.
- Standup CyberGear fixed gains: `cg_fixed_kp = 60.0 Nm/rad`, `cg_fixed_kd = 4.0 Nm*s/rad`.

## 5. DDSM115 Motor / Actuator Model

The DDSM115 model is implemented inline in two environment paths:

- `standup_env.py`, `_pre_physics_step()`.
- `residual_lqr_env.py`, `_pre_physics_step()`, duplicating the same current, torque, and torque-speed limiting logic after LQR/residual/disturbance currents are summed.

Inputs:

- Standup: normalized policy actions `actions[:, 4:6]`.
- Residual LQR: physical amperes from `u_LQR + u_RL + u_disturbance`.

Outputs:

- Desired current `_wheel_i_des`.
- Clamped current `_wheel_i_cmd`.
- Current torque `_wheel_tau_current`.
- Torque-speed limit `_wheel_tau_speed_limit`.
- Applied torque `_wheel_torque_cmd`.
- Joint effort target sent to the wheel joints.

Current-to-torque conversion:

```text
current_cmd = clamp(desired_current, -DDSM115_I_PEAK, DDSM115_I_PEAK)
tau_current = current_cmd * DDSM115_KT
```

Torque-speed envelope:

```text
tau_speed_limit = DDSM115_TAU_PEAK * (1 - abs(wheel_velocity) / DDSM115_NO_LOAD_SPEED)
tau_speed_limit = clamp(tau_speed_limit, 0, DDSM115_TAU_PEAK)
tau_actual = clamp(tau_current, -tau_speed_limit, +tau_speed_limit)
```

Saturation/current limit:

- Current is clamped at `DDSM115_I_PEAK = 2.7 A`.
- Low-speed torque is capped by `DDSM115_TAU_PEAK = 2.0 Nm`.
- Standup policy action `+/-1` maps to `wheel_current_max = DDSM115_TAU_RATED / DDSM115_KT = 1.28 A`, but diagnostics may raise `env_cfg.wheel_current_max` to allow peak current testing.

Delay/filtering:

- No delay, command-rate filter, thermal model, locked-rotor fault, or current-loop lag was found.

Deadzone/nonlinearity:

- No deadzone or nonlinear current response was found.
- The only nonlinearity is saturation and the linear torque-speed envelope.

Left/right asymmetry:

- The left wheel effort is negated before applying to the USD joint: `left_effort = -tau_left`, `right_effort = tau_right`.
- This is a sign convention for the mirrored left wheel joint axis, not a modeled hardware asymmetry.
- Torque-speed limiting is applied independently per wheel, so different instantaneous wheel velocities can create asymmetric available torque.

Parameters that should move to config:

- Current limits, torque limits, torque constant, no-load speed, rated speed, rated torque.
- Left/right sign convention.
- Any future deadzone, current-loop lag, thermal/current derating, or left/right calibration offsets.

## 6. LQR Controller

LQR exists in multiple places.

### Runtime Residual LQR Environment

- File: `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/residual_lqr_env.py`.
- Classes/functions: `LqrPhysicalParams`, `LqrWeights`, `_build_pitch_model`, `_calculate_lqr6_gain`, `_compute_lqr_current`.
- State vector: 6-state vector `[wheel_position_m, wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s, yaw_rad, yaw_rate_rad_s]`.
- Gain: `_k6_lqr`, calculated during environment construction using SciPy `solve_continuous_are`.
- Input/output units: output is left/right wheel current in amperes; state uses meters, meters/second, radians, radians/second.
- Conversion: current enters the B matrix through `Kt / wheel_radius`; yaw uses track width and yaw inertia.
- Hardcoded signs: `LQR_CURRENT_SIGN = -1.0`, `LQR_YAW_SIGN = -1.0`.

### Diagnostic and Evaluation LQR

- File: `scripts/lqr_control.py`.
- 4-state option: `LQR_STATE_MODE = "lqr4"` uses `[wheel_position_m, wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s]` and equal wheel currents.
- 6-state option: `LQR_STATE_MODE = "lqr6"` uses `[wheel_position_m, wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s, yaw_rad, yaw_rate_rad_s]` and independent left/right currents.
- Current setting: currently `LQR_STATE_MODE = "lqr6"`.
- Gain modes for 4-state: `GAIN_MODE = "auto_lqr"` or `manual_current`; currently `manual_current`.
- Manual 4-state current gains:
  - `K_WHEEL_POSITION_CURRENT = 0.0`
  - `K_WHEEL_VELOCITY_CURRENT = 0.474502`
  - `K_PITCH_CURRENT = 3.21126`
  - `K_PITCH_RATE_CURRENT = 0.327822`
  - roll gains are exposed but unused.
- 6-state gains are calculated from hardcoded physical parameters and weights in the script.

### Gain Calculation Script

- File: `scripts/calculate_lqr_gains.py`.
- Purpose: print 4-state force, torque, current, and normalized action gains.
- State: 4-state pitch model only.
- It duplicates physical parameters and has different default `q_wheel_velocity = 0.15` from the current 6-state runtime/evaluation value `20.0`.

Multiple LQR variants exist:

- Suspended-air diagnostic (`lqr-model`).
- Floor-contact LQR balancing (`lqr-floor`).
- 4-state pitch-only LQR in `scripts/lqr_control.py`.
- 6-state pitch+yaw LQR in `scripts/lqr_control.py` and `residual_lqr_env.py`.
- Standalone 4-state gain printer in `scripts/calculate_lqr_gains.py`.

## 7. Residual RL Controller

### Training Environment

- File: `residual_lqr_env.py`.
- Config: `residual_lqr_env_cfg.py`.
- Action shape: 2 normalized values.
- Combination:

```text
final_current = lqr_current + residual_current + disturbance_current
residual_current = clamp(action[0:2] * residual_action_limit, +/- residual_action_limit)
```

- Action scaling: `residual_action_limit = 0.5 A` by default.
- Current limits: after summing, final desired current is passed through shared DDSM115 current clamp and torque-speed limiter.
- Residual symmetry: per-wheel residual currents; not forced symmetric.
- Observations: 8 values: pitch, pitch rate, base position, base velocity, left/right wheel velocity normalized by no-load speed, previous left/right residual current normalized by residual limit.
- Network architecture: `agents/rsl_rl_residual_lqr_cfg.py` uses actor and critic hidden dims `[32, 32]`, ELU activation, no actor/critic observation normalization, `init_noise_std = 0.2`.
- Checkpoint loading: training uses RSL-RL checkpoints under `logs/rsl_rl/residual_lqr_two_wheel`; frozen evaluation expects exported TorchScript `policy.pt`.

### Frozen Evaluation Path

- File: `scripts/lqr_control.py`.
- Functions: `load_residual_policy`, `residual_observation`, `residual_current_from_policy`, `residual_reward`.
- Checkpoint loading path: CLI `--residual-policy`; must point to a TorchScript `policy.pt`, not a raw RSL-RL training checkpoint.
- Combination: same conceptual sum as the training environment.
- Observation count: 8 values matching `ResidualLqrEnv`.

## 8. Reward Function

### Standup Reward

File: `standup_env.py`, `_get_rewards()`. Constants live mostly in `standup_env_cfg.py`.

| Term | Formula / behavior | Weight | State dependency |
|---|---|---:|---|
| Uprighting | `rew_uprighting_scale * exp(-((proj_grav_z + 1)^2) / rew_uprighting_sigma)` | scale `1.0`, sigma `0.3` | projected gravity Z |
| Success bonus | one-time bonus once upright and near-zero CyberGear pose for `success_steps_required` steps | `15.0 + 10.0 * remaining_episode_fraction` | projected gravity X/Y/Z, CyberGear pose, episode length |
| Final pose | upright-only exponential reward for CyberGear joints near zero | `0.35` | CyberGear joint positions |
| Displacement | `rew_displacement_scale * ||root_xy - spawn_xy||^2` | `-0.02` | root XY position |
| Wheel energy | `rew_wheel_energy_scale * (I_left^2 + I_right^2)` | `-2.0e-4` | wheel current action |
| CyberGear energy | `rew_cg_energy_scale * sum(cg_actions^2)` | `-2.0e-5` | CyberGear action |
| Action rate | `rew_action_rate_scale * sum((cur_action - prev_action)^2)` | `-0.002` | current and previous action |
| Timeout/time penalty | scalar every step | `-0.03` | episode step |

The total is clamped with `torch.nan_to_num(...).clamp(-10.0, 2.0)`.

Reward constants that should move/remain in reward config:

- All `rew_*` values, success thresholds, `final_pose_deg_threshold`, and terminal clamp bounds.
- The internal constants `2.0` in final-pose exponent and `10.0`/`2.0` clamp bounds are currently embedded in the reward function.

### Residual LQR Reward

File: `residual_lqr_env.py`, `_get_rewards()`.

| Term | Formula / behavior | Weight | State dependency |
|---|---|---:|---|
| Pitch penalty | `-pitch^2` | `-1.0` | pitch |
| Pitch-rate penalty | `-0.1 * pitch_rate^2` | `-0.1` | pitch rate |
| Position penalty | `-0.05 * position^2` | `-0.05` | average wheel-derived base position |
| Residual-current penalty | `-0.01 * sum(rl_current^2)` | `-0.01` | residual current only |

The reward is clamped to `[-10.0, 0.0]` and accumulated in `_episode_reward`. The same residual formula is duplicated in `scripts/lqr_control.py` as `residual_reward()`.

Residual reward constants that should move to config:

- `1.0`, `0.1`, `0.05`, `0.01`, and clamp bounds `-10.0`, `0.0`.

## 9. Disturbances

### Residual Training Random Actuator Disturbance

- File: `residual_lqr_env.py` plus config in `residual_lqr_env_cfg.py`.
- Type: random per-episode actuator-current pulse.
- Parameters:
  - `disturbance_currents_a = (0.5, 1.0, 1.5, 2.0) A`
  - `disturbance_start_range_s = (1.3, 1.7) s`
  - `disturbance_samples = 10` control samples
- Application: left current gets `+amp`, right current gets `-amp`, added after LQR and residual current.
- Command-line arguments: none for training; set in config only.

### LQR Diagnostic External Wrench Disturbance

- File: `scripts/lqr_control.py`.
- Type: external force/torque applied to a named body via `permanent_wrench_composer`.
- Modes: `none`, `forward`, `backward`, `yaw-left`, `yaw-right`, `forward-yaw`.
- Parameters:
  - `--disturbance-start-s`
  - `--disturbance-duration-s`
  - `--disturbance-force-n`
  - `--disturbance-yaw-torque-nm`
  - `--disturbance-body`
- Application: world-frame force along yaw reference and/or world Z torque on the selected body.

### LQR Diagnostic Actuator Disturbances

- File: `scripts/lqr_control.py`.
- Modes: `none`, `physical-forward`, `step-forward`.
- Pulse disturbance:
  - `physical-forward` is pulse-style and uses `--actuator-disturbance-samples`.
  - Current mapping is left `+value`, right `-value`.
- Step disturbance:
  - `step-forward` uses `--actuator-disturbance-duration-s`.
  - Current mapping is also left `+value`, right `-value`.
- Parameters:
  - `--actuator-disturbance-start-s`
  - `--actuator-disturbance-samples`
  - `--actuator-disturbance-duration-s`
  - `--actuator-disturbance-current-a`
- Application: current added after LQR and optional residual policy current.

### Sweep and Benchmark Disturbances

- `scripts/run_lqr_disturbance_sweep.py` runs `physical-forward` pulses at currents `[0.5, 1.0, 1.5, 2.0] A`, start `1.5 s`, `10` samples.
- `scripts/run_lqr_vs_residual_repeatability.py` repeats the same pulse sweep for `lqr` and `residual_ppo`.
- `scripts/run_step_disturbance_benchmark.py` runs `step-forward` for currents `[0.1, 0.2, 0.3, 0.5] A`, start `1.5 s`, duration `2.0 s`.

### Sine Disturbance

No sine disturbance implementation was found in the current Python source scan. If it exists outside the scanned files, it is not currently exposed through the main Isaac Lab scripts listed above.

## 10. Logging and Metrics

### Training Logs

- `scripts/rsl_rl/train.py` writes RSL-RL logs under `logs/rsl_rl/<experiment_name>/<timestamp>`.
- It dumps `params/env.yaml` and `params/agent.yaml`.
- `StandupEnv` prints periodic debug reward information every `_DEBUG_LOG_INTERVAL = 2000` global steps.
- `ResidualLqrEnv` stores scalar metrics in `self.extras["log"]`: reward, episode reward, LQR currents, residual currents, final currents, disturbance current, and disturbance start.

### Play / Export Logs

- `scripts/rsl_rl/play.py` loads a checkpoint from the experiment log folder or explicit `--checkpoint`.
- It exports `exported/policy.pt` and `exported/policy.onnx` next to the checkpoint.

### Diagnostic CSV Logging

- `scripts/lqr_control.py`, class `CsvLogger`, writes a CSV to `--log-csv` unless the value is an empty string.
- Default path is `logs/ddsm115_free_spin.csv`.
- `sample_to_row()` writes pitch, yaw, position, velocity, disturbance fields, LQR/RL/final current fields, reward, root pose/velocity, wheel RPM, desired/saturated/current torque, torque-speed limit, actual torque, contact force, saturation flags, gravity, and angular velocity.

### Sweep and Benchmark Summaries

- `run_lqr_disturbance_sweep.py` writes raw CSVs, `summary.csv`, `pitch_overlay.png`, `command_overlay.png`, and `pitch_rms_bar.png`.
- `run_lqr_vs_residual_repeatability.py` writes raw CSVs under `raw/lqr` and `raw/residual_ppo`, plus `per_run_summary.csv`, `aggregate_summary.csv`, and comparison plots.
- `run_step_disturbance_benchmark.py` writes raw step CSVs, `per_run_summary.csv`, `aggregate_summary.csv`, and step comparison plots.
- Metrics include RMS pitch, peak pitch, peak-to-peak pitch, final pitch, current-command metrics, integrated control effort, survival success, recovery success, and during/after-step metrics.
- `analyze_lqr_yaw.py` writes yaw repeatability CSV and markdown report.
- `analyze_ddsm115_torque_speed_audit.py` writes torque-speed limiter audit plots and `summary.csv`.

## 11. Current Problems Found

- Physical constants are scattered across `sim_params.py`, `standup_env_cfg.py`, `residual_lqr_env.py`, `scripts/lqr_control.py`, `scripts/calculate_lqr_gains.py`, `scripts/uart_policy_runner.py`, and analysis scripts.
- LQR physical parameters are duplicated in at least three Python files.
- DDSM115 constants are mostly centralized in `sim_params.py`, but still duplicated in hardware/export and audit scripts.
- DDSM115 actuator logic is duplicated in `StandupEnv` and `ResidualLqrEnv`.
- Residual reward logic is duplicated between the training environment and `scripts/lqr_control.py`.
- `scripts/lqr_control.py` mixes controller design, policy loading, disturbance generation, plotting, CSV logging, spawn overrides, and Isaac app lifecycle.
- `residual_lqr_env.py` mixes LQR model construction, residual controller logic, disturbances, DDSM motor model, reward, observations, reset, and logging.
- Some names are abbreviated or historical: `cg`, `cfg`, `lqr4`, `lqr6`, `ColectedUSD_v2`, `Template-Twowheeledrobot-*`.
- Common evaluation runs require long argument lists and rely on nested subprocess wrappers.
- Reward constants include hardcoded formula constants inside environment methods rather than named config fields.
- No single config file captures robot physical parameters used by both simulation and analytical controllers.
- No single command/config abstraction captures pulse, step, and future sine disturbance experiments.
- Sine disturbance is referenced as an expected project capability but was not found in the current main source tree.
- Generated CSV column names include multiple aliases for the same concept, for example `u_left_lqr`, `u_left_lqr_a`, `u_left_final`, `u_left_final_a`, `u_left_cmd_a`.
- README says the cleaned project keeps one task, but the code currently registers both standup and residual LQR tasks.

## 12. Proposed New Architecture

Keep the Isaac Lab task registration structure, but split task logic into descriptive modules. Suggested layout:

```text
source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/
├── __init__.py
├── robot_articulation_configuration.py
├── simulation_physics_parameters.py
├── standup_environment_configuration.py
├── residual_lqr_environment_configuration.py
├── two_wheeled_robot_environment.py
├── residual_lqr_environment.py
├── controllers/
│   ├── lqr_controller.py
│   └── residual_rl_controller.py
├── actuators/
│   └── ddsm115_motor_model.py
├── rewards/
│   ├── standup_reward_function.py
│   └── residual_lqr_reward_function.py
├── disturbances/
│   ├── disturbance_generator.py
│   ├── actuator_current_disturbance.py
│   └── external_wrench_disturbance.py
├── observations/
│   ├── standup_observation.py
│   └── residual_lqr_observation.py
└── agents/
    ├── rsl_rl_standup_cfg.py
    └── rsl_rl_residual_lqr_cfg.py
```

Suggested configuration layout:

```text
configuration/
├── robot/
│   ├── robot_physical_parameters.yaml
│   ├── ddsm115_motor_parameters.yaml
│   └── cybergear_joint_parameters.yaml
├── rewards/
│   ├── standup_reward_weights.yaml
│   └── residual_lqr_reward_weights.yaml
├── controllers/
│   ├── lqr_controller.yaml
│   └── residual_rl_controller.yaml
└── experiments/
    ├── train_standup.yaml
    ├── train_residual_rl.yaml
    ├── evaluate_lqr_baseline.yaml
    ├── evaluate_pulse_disturbance.yaml
    ├── evaluate_step_disturbance.yaml
    └── evaluate_sine_disturbance.yaml
```

Suggested script layout:

```text
scripts/
├── train_residual_rl.py
├── train_standup.py
├── evaluate_lqr.py
├── evaluate_controller.py
├── calculate_lqr_gains.py
├── export_policy.py
├── hardware/
│   └── uart_policy_runner.py
├── analysis/
│   ├── plot_control_effort.py
│   ├── analyze_ddsm115_torque_speed.py
│   └── analyze_lqr_yaw.py
└── tools/
    └── extract_robot_physical_parameters.py
```

Responsibility split:

- Isaac environment definition: reset, stepping, scene setup, Isaac-specific buffers and lifecycle only.
- Robot/motor physical model: `ddsm115_motor_model.py` takes desired current and wheel velocity, returns current, torque limit, applied torque, and debug fields.
- Controllers: `lqr_controller.py` owns physical parameters, CARE solve, state construction, sign conventions, and output current; `residual_rl_controller.py` owns observation building, TorchScript loading, action scaling, and current clipping.
- Reward function: pure functions/classes that receive named state/action tensors and return reward terms plus total.
- Disturbances: independent generators for pulse, step, sine, random episode-sampled current disturbances, and external wrenches.
- Configuration files: hold robot physical constants, reward weights, disturbance parameters, controller choices, and experiment defaults.
- Plotting/analysis: consume standardized CSV schemas rather than reaching into control scripts.

## 13. Refactor Plan

### Stage 1: Documentation only

- Keep `PROJECT_STRUCTURE_AUDIT.md` as the current map.
- Add short docstrings/comments only if needed in future work.
- No behavior changes.

### Stage 2: Move constants to config files

- Create `robot_physical_parameters.yaml`, `ddsm115_motor_parameters.yaml`, `reward_weights.yaml`, and controller config files.
- Load values into existing classes while preserving current defaults exactly.
- Add a small validation script that prints old hardcoded values vs loaded config values.

### Stage 3: Rename files/functions descriptively

- Rename gradually using compatibility imports if needed.
- Prefer names such as `two_wheeled_robot_environment.py`, `reward_function.py`, `disturbance_generator.py`, `ddsm115_motor_model.py`, `lqr_controller.py`, `residual_rl_controller.py`.
- Avoid vague names like `env.py`, `cfg.py`, `utils.py`, `common.py`, `helper.py`, `misc.py`, `twr_env.py`.

### Stage 4: Split reward, disturbance, controller, and motor model logic

- Extract DDSM115 model first because it is duplicated and easy to regression-test numerically.
- Extract LQR controller and residual controller next.
- Extract reward functions after establishing term-by-term equivalence tests.
- Extract disturbance generation into reusable pulse, step, sine, random-current, and external-wrench classes.

### Stage 5: Create unified train/evaluate commands

- Replace long command lines with experiment YAML files.
- Keep old scripts as wrappers for one transition period.
- Standardize output directory and CSV schema across LQR, residual PPO, pulse, step, and sine evaluations.

### Stage 6: Add regression tests

- Unit-test DDSM115 torque-speed outputs for fixed current/velocity inputs.
- Unit-test LQR gain/state/current outputs for fixed states.
- Unit-test residual observation construction against current environment and `scripts/lqr_control.py`.
- Unit-test reward term outputs for fixed tensors.
- Add fixed-seed smoke tests that compare old and new CSV outputs for short runs where Isaac Sim is available.

## 14. Suggested Common Commands

Future config-based commands:

```bash
python scripts/train_residual_rl.py --config configuration/experiments/train_residual_rl.yaml
python scripts/evaluate_lqr.py --config configuration/experiments/evaluate_lqr_baseline.yaml
python scripts/evaluate_controller.py --config configuration/experiments/evaluate_pulse_disturbance.yaml
python scripts/evaluate_controller.py --config configuration/experiments/evaluate_step_disturbance.yaml
python scripts/evaluate_controller.py --config configuration/experiments/evaluate_sine_disturbance.yaml
```

Additional useful commands:

```bash
python scripts/train_standup.py --config configuration/experiments/train_standup.yaml
python scripts/export_policy.py --config configuration/experiments/export_residual_policy.yaml
python scripts/tools/extract_robot_physical_parameters.py --config configuration/robot/extract_physical_parameters.yaml
python scripts/analysis/plot_control_effort.py --config configuration/experiments/plot_control_effort.yaml
```
