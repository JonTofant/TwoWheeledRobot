# Architecture Dependency Map

This map is documentation only. It starts from `PROJECT_STRUCTURE_AUDIT.md` and verifies the current dependency edges against the repository source.

## 1. High-Level Execution Flow

### `scripts/rsl_rl/train.py`

```text
scripts/rsl_rl/train.py
  -> adds source/TwoWheeledRobot to sys.path
  -> imports cli_args
  -> launches Isaac Sim through isaaclab.app.AppLauncher
  -> imports TwoWheeledRobot.tasks for Gym registration side effects
  -> hydra_task_config(args_cli.task, args_cli.agent)
  -> gym.make(task, cfg=env_cfg)
     -> Template-Twowheeledrobot-Standup-v0
        -> standup_env.py::StandupEnv
        -> standup_env_cfg.py::StandupEnvCfg
        -> robot_cfg.py::TWO_WHEELED_ROBOT_CFG
        -> sim_params.py constants
        -> standup_env.py::_pre_physics_step()
        -> inline DDSM115 motor model
        -> standup_env.py::_get_rewards()
     -> Template-Twowheeledrobot-ResidualLQR-v0
        -> residual_lqr_env.py::ResidualLqrEnv
        -> residual_lqr_env_cfg.py::ResidualLqrEnvCfg
        -> inherits StandupEnv scene/articulation setup
        -> residual_lqr_env.py::_compute_lqr_current()
        -> residual_lqr_env.py::_pre_physics_step()
        -> inline DDSM115 motor model
        -> residual_lqr_env.py::_get_rewards()
        -> residual_lqr_env.py::_sample_disturbance()
  -> RslRlVecEnvWrapper
  -> rsl_rl.runners.OnPolicyRunner or DistillationRunner
  -> logs/rsl_rl/<experiment_name>/<timestamp>/
     -> params/env.yaml
     -> params/agent.yaml
     -> RSL-RL checkpoints and TensorBoard-style training logs
```

Motor model path:

- Standup task: `standup_env.py::_pre_physics_step()`.
- Residual task: `residual_lqr_env.py::_pre_physics_step()`.

Reward path:

- Standup task: `standup_env.py::_get_rewards()`.
- Residual task: `residual_lqr_env.py::_get_rewards()`.

Disturbance path:

- Standup task: none in the environment.
- Residual task: `residual_lqr_env.py::_sample_disturbance()` and disturbance addition in `_pre_physics_step()`.

Logging/output path:

- RSL-RL runner logs.
- Environment scalar metrics through `env.extras["log"]`, especially in `ResidualLqrEnv`.

### `scripts/rsl_rl/play.py`

```text
scripts/rsl_rl/play.py
  -> adds source/TwoWheeledRobot to sys.path
  -> imports cli_args
  -> launches Isaac Sim through AppLauncher
  -> imports TwoWheeledRobot.tasks for Gym registration side effects
  -> hydra_task_config(args_cli.task, args_cli.agent)
  -> resolves checkpoint
     -> explicit --checkpoint
     -> latest run under logs/rsl_rl/<experiment_name>
     -> optional published pretrained checkpoint path
  -> gym.make(task, cfg=env_cfg)
     -> same task graph as train.py
  -> RslRlVecEnvWrapper
  -> OnPolicyRunner or DistillationRunner
  -> runner.load(resume_path)
  -> runner.get_inference_policy()
  -> export_policy_as_jit(..., exported/policy.pt)
  -> export_policy_as_onnx(..., exported/policy.onnx)
  -> while simulation_app.is_running()
     -> action = policy(obs)
     -> env.step(action)
     -> environment motor/reward/done paths
```

Motor model path:

- Same as selected task environment.

Reward path:

- Same as selected task environment.

Disturbance path:

- Same as selected task environment. For `ResidualLqrEnv`, random actuator-current pulse logic remains active unless config is changed.

Logging/output path:

- Loads from `logs/rsl_rl/<experiment_name>/...`.
- Exports policy artifacts under the checkpoint run directory: `exported/policy.pt`, `exported/policy.onnx`.
- Optional video output under `<run>/videos/play`.

### `scripts/lqr_control.py`

```text
scripts/lqr_control.py
  -> adds source/TwoWheeledRobot to sys.path
  -> launches Isaac Sim through AppLauncher
  -> imports TwoWheeledRobot.tasks for Gym registration side effects
  -> imports sim_params.py::{DDSM115_I_PEAK, GROUND_*}
  -> defines LqrPhysicalParams and LqrWeights
  -> compute_single_wheel_4_state_lqr_current()
  -> compute_split_4_state_lqr_currents()
  -> shared K_SPLIT_4_STATE_LQR_CURRENT gain vector
  -> hydra_task_config(args_cli.task, None)
  -> mutates env_cfg for diagnostic mode
     -> scene.num_envs = 1
     -> decimation = --control-decimation
     -> long episode, no observation noise, fixed wheel damping
     -> wheel_current_max raised to expose current saturation
     -> optional wheel contact sensor activation for lqr-floor
  -> gym.make(args_cli.task, cfg=env_cfg)
     -> normally Template-Twowheeledrobot-Standup-v0
     -> StandupEnv scene, robot, IMU, optional wheel contacts
  -> optional spawn overrides
     -> force_upright_fixed_spawn()
     -> force_floor_pitch_spawn()
  -> optional base hold fixed joint for suspended tests
  -> loop
     -> read_state()
     -> compute_action()
        -> free-spin constant current, or
        -> compute_split_4_state_lqr_currents()
           -> independent left/right 4-state wheel controllers
        -> residual_current_from_policy()
           -> load_residual_policy()
           -> residual_observation()
        -> actuator_disturbance_current()
        -> residual_reward()
     -> action_to_env()
     -> apply_disturbance()
        -> disturbance_wrench()
        -> external force/torque through permanent_wrench_composer
     -> env.step(env_action)
        -> StandupEnv::_pre_physics_step()
        -> inline DDSM115 motor model
        -> StandupEnv::_get_rewards()
     -> read_motor_debug()
     -> sample_to_row()
     -> CsvLogger.write()
     -> optional LivePlot.update()
```

Motor model path:

- Even for LQR and residual evaluation, the actual DDSM115 current clamp and torque-speed limiter are the inline model in `StandupEnv::_pre_physics_step()`.

Reward path:

- `scripts/lqr_control.py::residual_reward()` is used for residual-evaluation metrics.
- `StandupEnv::_get_rewards()` still runs during `env.step()`, but the diagnostic scripts mainly consume the script-level metric row.

Disturbance path:

- Current disturbance: `actuator_disturbance_current()`.
- External wrench disturbance: `disturbance_wrench()` and `apply_disturbance()`.

Logging/output path:

- `CsvLogger` writes `--log-csv`, default `logs/ddsm115_free_spin.csv`.
- `sample_to_row()` defines the large diagnostic CSV schema.
- Optional live matplotlib plots.

### `scripts/calculate_lqr_gains.py`

```text
scripts/calculate_lqr_gains.py
  -> RobotParams
  -> LqrWeights
  -> build_pitch_model()
  -> lqr()
     -> scipy.linalg.solve_continuous_are
  -> force_gain_to_normalized_action_gain()
  -> print_results()
     -> prints force gains
     -> prints normalized action gains
     -> prints per-wheel torque/current gains
     -> prints paste-ready manual_current constants for scripts/lqr_control.py
```

Environment class/controller used:

- No Isaac environment.
- Uses its own 4-state analytical pitch LQR controller model.

Motor model path:

- No runtime motor model. It only converts force gains to current/action gains using duplicated motor constants.

Reward path:

- None.

Disturbance path:

- None.

Logging/output path:

- Console only.

### `scripts/run_lqr_disturbance_sweep.py`

```text
scripts/run_lqr_disturbance_sweep.py
  -> imports pandas, numpy, matplotlib
  -> parse_args()
  -> for each current in --currents
     -> run_simulation()
        -> subprocess.run([
             python,
             scripts/lqr_control.py,
             --task Template-Twowheeledrobot-Standup-v0,
             --test-mode lqr-floor,
             --actuator-disturbance physical-forward,
             --actuator-disturbance-start-s 1.5,
             --actuator-disturbance-samples 10,
             --actuator-disturbance-current-a <current>,
             --log-csv <raw csv>,
             --no-plot
           ])
        -> lqr_control.py execution flow
     -> compute_metrics(raw csv)
        -> time_s()
        -> pitch_deg()
        -> command_columns()
        -> survival/recovery metrics
  -> write_summary(summary.csv)
  -> plot_pitch_overlay()
  -> plot_command_overlay()
  -> plot_pitch_rms_bar()
```

Environment class/controller used:

- Indirectly uses `StandupEnv` through `scripts/lqr_control.py`.
- Controller is LQR in `scripts/lqr_control.py`; optional residual policy if `--enable-residual-rl`.

Motor model path:

- `StandupEnv::_pre_physics_step()` through `lqr_control.py`.

Reward path:

- Diagnostic metrics from CSV; residual reward if residual mode is enabled in `lqr_control.py`.

Disturbance path:

- Pulse current disturbance through `lqr_control.py::actuator_disturbance_current()` with mode `physical-forward`.

Logging/output path:

- Raw CSVs under `--output-dir`.
- `summary.csv` and PNG plots under `--output-dir`.

### `scripts/run_lqr_vs_residual_repeatability.py`

```text
scripts/run_lqr_vs_residual_repeatability.py
  -> sys.path inserts scripts/
  -> imports run_lqr_disturbance_sweep as sweep
  -> for controller in ("lqr", "residual_ppo")
     -> for each current and repeat
        -> sweep.run_simulation()
           -> subprocess.run(scripts/lqr_control.py ...)
           -> if controller == residual_ppo:
              -> add --enable-residual-rl
              -> add --evaluation-mode
              -> add --residual-action-limit
              -> optionally add --residual-policy
        -> sweep.compute_metrics()
        -> compute_control_effort_metrics()
        -> per_run_row()
  -> aggregate_rows()
  -> plot_comparisons()
  -> write per_run_summary.csv
  -> write aggregate_summary.csv
```

Environment class/controller used:

- Indirectly `StandupEnv` through `lqr_control.py`.
- LQR baseline and LQR plus frozen residual PPO through `lqr_control.py`.

Motor model path:

- `StandupEnv::_pre_physics_step()` through `lqr_control.py`.

Reward path:

- Metrics from CSV; `lqr_control.py::residual_reward()` is logged when residual is enabled.

Disturbance path:

- Same pulse current disturbance path as `run_lqr_disturbance_sweep.py`.

Logging/output path:

- Raw CSVs under `<output-dir>/raw/lqr` and `<output-dir>/raw/residual_ppo`.
- `per_run_summary.csv`, `aggregate_summary.csv`, and comparison PNGs.

### `scripts/run_step_disturbance_benchmark.py`

```text
scripts/run_step_disturbance_benchmark.py
  -> sys.path inserts scripts/
  -> imports run_lqr_disturbance_sweep as sweep
  -> for controller in ("lqr", "residual_ppo")
     -> for each current and repeat
        -> run_simulation()
           -> subprocess.run([
                python,
                scripts/lqr_control.py,
                --task Template-Twowheeledrobot-Standup-v0,
                --test-mode lqr-floor,
                --actuator-disturbance step-forward,
                --actuator-disturbance-start-s <start>,
                --actuator-disturbance-duration-s <duration>,
                --actuator-disturbance-current-a <current>,
                --log-csv <raw csv>,
                --no-plot,
                optional residual flags
              ])
        -> compute_metrics()
           -> sweep.time_s()
           -> sweep.pitch_deg()
           -> final_command_columns()
           -> during-step metrics
           -> after-step metrics
           -> control effort metrics
  -> aggregate_rows()
  -> plot_comparisons()
  -> write per_run_summary.csv
  -> write aggregate_summary.csv
```

Environment class/controller used:

- Indirectly `StandupEnv` through `lqr_control.py`.
- LQR baseline and optional LQR plus frozen residual PPO.

Motor model path:

- `StandupEnv::_pre_physics_step()` through `lqr_control.py`.

Reward path:

- Metrics from CSV; `lqr_control.py::residual_reward()` if residual is enabled.

Disturbance path:

- Step current disturbance through `lqr_control.py::actuator_disturbance_current()` with mode `step-forward`.

Logging/output path:

- Raw CSVs under `<output-dir>/raw/lqr` and `<output-dir>/raw/residual_ppo`.
- `per_run_summary.csv`, `aggregate_summary.csv`, and step comparison PNGs.

## 2. Isaac Task Dependency Graph

### `Template-Twowheeledrobot-Standup-v0`

```text
registration
  -> source/.../twowheeledrobot/__init__.py
  -> gym.register(id="Template-Twowheeledrobot-Standup-v0")

environment class
  -> standup_env.py::StandupEnv

environment configuration
  -> standup_env_cfg.py::StandupEnvCfg
  -> imports robot_cfg.py::TWO_WHEELED_ROBOT_CFG
  -> imports sim_params.py::{CONTROL_DECIMATION, PHYSICS_DT, DDSM115_*}

robot articulation configuration
  -> robot_cfg.py::TWO_WHEELED_ROBOT_CFG
  -> USD: source/TwoWheeledRobot/docs/ColectedUSD_v2/World0.usd
  -> actuator groups: wheel_joints, cybergear_joints, bearing_joints
  -> imports sim_params.py for damping, stiffness, limits, solver settings

simulation physics parameter file
  -> sim_params.py

agent configuration
  -> agents/rsl_rl_standup_cfg.py::StandupPPORunnerCfg

observation function
  -> standup_env.py::_get_observations()

reward function
  -> standup_env.py::_get_rewards()

action application function
  -> standup_env.py::_pre_physics_step()
  -> standup_env.py::_apply_action()

reset function
  -> standup_env.py::_reset_idx()
  -> standup_env.py::_sample_fallen_poses()

done/termination function
  -> standup_env.py::_get_dones()
```

### `Template-Twowheeledrobot-ResidualLQR-v0`

```text
registration
  -> source/.../twowheeledrobot/__init__.py
  -> gym.register(id="Template-Twowheeledrobot-ResidualLQR-v0")

environment class
  -> residual_lqr_env.py::ResidualLqrEnv
  -> inherits standup_env.py::StandupEnv

environment configuration
  -> residual_lqr_env_cfg.py::ResidualLqrEnvCfg
  -> inherits standup_env_cfg.py::StandupEnvCfg

robot articulation configuration
  -> inherited through StandupEnvCfg.robot_cfg
  -> robot_cfg.py::TWO_WHEELED_ROBOT_CFG

simulation physics parameter file
  -> sim_params.py

agent configuration
  -> agents/rsl_rl_residual_lqr_cfg.py::ResidualLqrPPORunnerCfg

observation function
  -> residual_lqr_env.py::_get_observations()

reward function
  -> residual_lqr_env.py::_get_rewards()

action application function
  -> residual_lqr_env.py::_pre_physics_step()
  -> StandupEnv helper methods for CyberGear limits and simulation write

reset function
  -> residual_lqr_env.py::_reset_idx()
  -> calls StandupEnv::_reset_idx() first
  -> then overrides root pose and task-specific buffers

done/termination function
  -> residual_lqr_env.py::_get_dones()
```

## 3. Duplicate Logic Map

### DDSM115 Motor Current-to-Torque Logic

Current locations:

- `standup_env.py::_pre_physics_step()`
- `residual_lqr_env.py::_pre_physics_step()`

Risk:

- If current clamping or torque conversion changes in one environment only, training and evaluation can silently use different actuator physics.
- Residual training may learn against one current envelope while standup diagnostics and deployment assumptions use another.

Recommended single owner:

- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/actuators/ddsm115_motor_model.py`

### DDSM115 Torque-Speed Limiter Logic

Current locations:

- `standup_env.py::_pre_physics_step()`
- `residual_lqr_env.py::_pre_physics_step()`
- `scripts/analyze_ddsm115_torque_speed_audit.py` re-implements the expected equation for validation.

Risk:

- The limiter equation is central to free-spin behavior, current effort, and disturbance response. A one-sided update could invalidate existing benchmark comparisons.
- The audit script could pass or fail for the wrong reason if it diverges from the runtime implementation.

Recommended single owner:

- Runtime owner: `actuators/ddsm115_motor_model.py`.
- Analysis scripts should import or explicitly document the same equation and parameters.

### LQR Physical Parameters

Current locations:

- `residual_lqr_env.py::LQR_PHYSICAL_PARAMS`
- `scripts/lqr_control.py::LQR_PHYSICAL_PARAMS`
- `scripts/calculate_lqr_gains.py::RobotParams`
- `scripts/tools/extract_robot_physical_parameters.py` derives related values from the USD and writes CSV.

Risk:

- LQR gains used during residual training can differ from LQR gains used during frozen-policy evaluation.
- The gain calculator can print gains for parameters that are no longer the runtime parameters.

Recommended single owner:

- `configuration/robot/robot_physical_parameters.yaml`
- Future loader module: `controllers/lqr_controller.py` or `configuration/robot_physical_parameters.py`

### LQR Gain Calculation

Current locations:

- `residual_lqr_env.py::_build_pitch_model()` and `_calculate_lqr6_gain()`
- `scripts/lqr_control.py::compute_single_wheel_4_state_lqr_current()` and `compute_split_4_state_lqr_currents()`
- `scripts/calculate_lqr_gains.py::build_pitch_model()` and `lqr()`

Risk:

- The 6-state residual-training controller and split 4-state diagnostic controller are now intentionally different.
- The 4-state gain calculator remains a separate reference and can drift from the diagnostic script if gains are copied manually.

Recommended single owner:

- `controllers/lqr_controller.py`

### Residual RL Observation Creation

Current locations:

- `residual_lqr_env.py::_get_observations()`
- `scripts/lqr_control.py::residual_observation()`

Risk:

- A frozen residual policy can be evaluated on an observation layout different from the training layout.
- Previous residual current normalization and wheel velocity normalization are especially easy to change in one place only.

Recommended single owner:

- `controllers/residual_rl_controller.py` or `observations/residual_lqr_observation.py`

### Residual Reward Calculation

Current locations:

- `residual_lqr_env.py::_get_rewards()`
- `scripts/lqr_control.py::residual_reward()`

Risk:

- Training reward and evaluation-reported reward can diverge.
- Benchmark CSV reward columns may stop meaning what the residual task actually optimized.

Recommended single owner:

- `rewards/residual_lqr_reward_function.py`

### Disturbance Current Generation

Current locations:

- `residual_lqr_env.py::_sample_disturbance()` and current addition in `_pre_physics_step()`
- `scripts/lqr_control.py::actuator_disturbance_current()`
- `scripts/run_lqr_disturbance_sweep.py` hardcodes `physical-forward`, start `1.5 s`, samples `10`, currents `[0.5, 1.0, 1.5, 2.0]`
- `scripts/run_step_disturbance_benchmark.py` hardcodes `step-forward` orchestration defaults.

Risk:

- Training disturbance distribution and evaluation disturbance distribution may be described with similar names but behave differently.
- Adding sine disturbance in only one layer would make it unavailable to other benchmarks.

Recommended single owner:

- `disturbances/actuator_current_disturbance.py`
- Common experiment parameters in `configuration/experiments/*.yaml`

### CSV Metric Naming

Current locations:

- `scripts/lqr_control.py::sample_to_row()`
- `scripts/run_lqr_disturbance_sweep.py::command_columns()`
- `scripts/run_lqr_vs_residual_repeatability.py::FINAL_COMMAND_COLUMN_CANDIDATES`
- `scripts/run_step_disturbance_benchmark.py::final_command_columns()`
- plotting scripts that search for alternate column names.

Risk:

- Adding or renaming a CSV column can break downstream plots or silently select the wrong alias.
- Multiple aliases for final current make it hard to know the canonical metric.

Recommended single owner:

- `analysis/metrics_schema.py` or `logging/controller_csv_schema.py`

### Robot Physical Constants

Current locations:

- `robot_cfg.py` for articulation, USD path, actuator limits.
- `standup_env_cfg.py` for spawn geometry, CyberGear training gains, wheel current max.
- `standup_env.py` for CyberGear mechanical limits and signs.
- `residual_lqr_env.py`, `scripts/lqr_control.py`, and `scripts/calculate_lqr_gains.py` for LQR physical constants.
- `scripts/uart_policy_runner.py` for deployment-side CyberGear limits and DDSM current scaling.

Risk:

- Simulation, LQR, and UART deployment can disagree about the same robot.
- The policy can be exported with one normalization convention and run on hardware with another.

Recommended single owner:

- `configuration/robot/robot_physical_parameters.yaml`
- `configuration/robot/cybergear_joint_parameters.yaml`

### Motor Constants

Current locations:

- `sim_params.py`
- `scripts/uart_policy_runner.py`
- `scripts/analyze_ddsm115_torque_speed_audit.py`
- LQR dataclasses in `residual_lqr_env.py`, `scripts/lqr_control.py`, and `scripts/calculate_lqr_gains.py`
- Documentation files.

Risk:

- Deployment current scaling can differ from simulation current scaling.
- Torque-speed audit can validate against stale motor constants.

Recommended single owner:

- `configuration/robot/ddsm115_motor_parameters.yaml`
- Runtime module: `actuators/ddsm115_motor_model.py`

## 4. Import Dependency List

### `__init__.py`

Imports from project files:

- `from . import agents`

Imported by:

- `TwoWheeledRobot.tasks` package import chain.
- `scripts/rsl_rl/train.py`
- `scripts/rsl_rl/play.py`
- `scripts/lqr_control.py`
- `scripts/list_envs.py`
- `scripts/tools/extract_robot_physical_parameters.py`

Side effects during import:

- Registers both Gym tasks through `gym.register()`.
- Does not import the environment classes directly; entry points are string paths loaded later by Gym/Hydra.

### `sim_params.py`

Imports from project files:

- None.

Imported by:

- `robot_cfg.py`
- `standup_env_cfg.py`
- `standup_env.py`
- `residual_lqr_env.py`
- `scripts/lqr_control.py`

Side effects during import:

- Defines constants only.
- Computes speed constants from `math.pi`.

### `robot_cfg.py`

Imports from project files:

- `sim_params.py` constants for damping, solver settings, DDSM limits, CyberGear gains, bearing damping.

Imported by:

- `standup_env_cfg.py`

Side effects during import:

- Computes `_USD_PATH`.
- Constructs `TWO_WHEELED_ROBOT_CFG` at import time.
- Does not launch Isaac Sim or load the USD by itself.

### `standup_env_cfg.py`

Imports from project files:

- `robot_cfg.py::TWO_WHEELED_ROBOT_CFG`
- `sim_params.py::{CONTROL_DECIMATION, DDSM115_KT, DDSM115_NO_LOAD_SPEED, DDSM115_TAU_RATED, PHYSICS_DT}`

Imported by:

- `standup_env.py`
- `residual_lqr_env_cfg.py`
- Isaac Lab/Hydra registry through task entry point string.

Side effects during import:

- Constructs class-level Isaac Lab config objects, including `SimulationCfg`, `InteractiveSceneCfg`, `ImuCfg`, `ContactSensorCfg`, and a replaced robot articulation config.
- No simulator launch.

### `standup_env.py`

Imports from project files:

- `standup_env_cfg.py::StandupEnvCfg`
- `sim_params.py::{DDSM115_I_PEAK, DDSM115_KT, DDSM115_NO_LOAD_SPEED, DDSM115_TAU_PEAK, GROUND_DYNAMIC_FRICTION, GROUND_RESTITUTION, GROUND_STATIC_FRICTION, WHEEL_INTERNAL_DAMPING}`

Imported by:

- `residual_lqr_env.py` for subclassing.
- Gym/Hydra when `Template-Twowheeledrobot-Standup-v0` is constructed.

Side effects during import:

- Defines `StandupEnv`.
- No scene is built until the class is instantiated.

### `residual_lqr_env_cfg.py`

Imports from project files:

- `standup_env_cfg.py::StandupEnvCfg`

Imported by:

- `residual_lqr_env.py`
- Isaac Lab/Hydra registry through task entry point string.

Side effects during import:

- Defines `ResidualLqrEnvCfg`.
- Inherits and overrides class-level config values from `StandupEnvCfg`.

### `residual_lqr_env.py`

Imports from project files:

- `residual_lqr_env_cfg.py::ResidualLqrEnvCfg`
- `sim_params.py::{DDSM115_I_PEAK, DDSM115_KT, DDSM115_NO_LOAD_SPEED, DDSM115_TAU_PEAK}`
- `standup_env.py::StandupEnv`

Imported by:

- Gym/Hydra when `Template-Twowheeledrobot-ResidualLQR-v0` is constructed.

Side effects during import:

- Defines LQR dataclasses and module-level LQR constants.
- Does not solve CARE at import; `_calculate_lqr6_gain()` is called in `ResidualLqrEnv.__init__`.
- Imports SciPy lazily inside `_calculate_lqr6_gain()`.

### `agents/rsl_rl_standup_cfg.py`

Imports from project files:

- None.

Imported by:

- RSL-RL config registry through `rsl_rl_cfg_entry_point`.
- Package `agents` may expose it indirectly depending on Isaac Lab config loading.

Side effects during import:

- Defines `StandupPPORunnerCfg`.
- Creates class-level policy and algorithm config objects.

### `agents/rsl_rl_residual_lqr_cfg.py`

Imports from project files:

- None.

Imported by:

- RSL-RL config registry through `rsl_rl_cfg_entry_point`.

Side effects during import:

- Defines `ResidualLqrPPORunnerCfg`.
- Creates class-level policy and algorithm config objects.

## 5. Refactor Safety Assessment

### DDSM115 Motor Model Extraction: Very Safe

This logic is duplicated in two environment methods and has a compact tensor interface: desired current plus wheel velocity in, clamped current and torque outputs out. It can be extracted with numerical regression tests against fixed tensors before touching any Isaac behavior. The left-wheel sign should remain outside the motor model because it is a USD joint-axis convention, not motor physics.

### Robot Physical Parameter Extraction: Mostly Safe

Moving constants into a config file is conceptually safe, but it touches LQR, diagnostics, and deployment normalization. It should follow the motor model extraction or be done with explicit value-equivalence checks.

### LQR Controller Extraction: Risky

The LQR code is duplicated, but it is central to residual training and all floor benchmarks. It also includes sign conventions, yaw dynamics, and CARE solving. Extraction is worthwhile, but only after the motor model has tests and after the physical parameters have a single source.

### Residual Reward Extraction: Mostly Safe

The residual reward is simple and duplicated. It can be extracted into a pure function with tensor tests. The risk is lower than LQR extraction, but still affects learning curves and benchmark reward columns.

### Disturbance Generator Extraction: Mostly Safe

Pulse and step current disturbances can be isolated, but the training environment and diagnostic script currently represent timing differently: per-env random start samples in the environment, CLI/sample-index scheduling in `lqr_control.py`. Extract only after defining shared semantics for start time, sample count, duration, and left/right current mapping.

### Renaming Environment Files: Very Risky

Environment file names are embedded in Gym entry-point strings. Renaming them can break task creation, Hydra config loading, checkpoints, and scripts. Do this late with compatibility shims.

### Changing Task Registration Names: Very Risky

Task IDs are used by training commands, play commands, logs, docs, and scripts. Changing them breaks reproducibility and existing checkpoints unless aliases are preserved.

### Changing Training Scripts: Risky

The scripts are close to Isaac Lab templates and carry simulator launch ordering constraints. Refactor around them with wrappers/config files first rather than editing their launch flow early.

### Changing CSV Schemas: Very Risky

Many analysis scripts depend on current CSV column names, and some already carry alias lists. Rename columns only after introducing a schema module and backward-compatible readers.

## 6. Proposed First Refactor Target

The safest first refactor is to extract the shared DDSM115 motor model while preserving behavior exactly.

Recommended future module:

```text
source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/actuators/ddsm115_motor_model.py
```

Suggested tensor-friendly API:

```python
compute_ddsm115_motor_output(
    desired_current_a,
    wheel_velocity_rad_per_s,
    motor_parameters,
)
```

Expected outputs:

```python
commanded_current_a
current_limited_torque_nm
speed_limited_torque_nm
torque_speed_limit_nm
is_current_saturated
is_torque_speed_saturated
```

Suggested behavior-preserving extraction boundary:

```text
inside shared motor model
  -> clamp desired current to +/- DDSM115_I_PEAK
  -> convert current to torque with DDSM115_KT
  -> compute torque-speed limit from abs(wheel_velocity)
  -> clamp current torque to torque-speed limit
  -> report saturation flags and debug tensors

outside shared motor model
  -> map standup normalized actions to desired current
  -> sum LQR + residual + disturbance current
  -> apply left/right USD joint-axis sign convention
  -> call robot.set_joint_effort_target()
```

Why this is the first target:

- It has the least dependency on Isaac Lab lifecycle.
- It is currently duplicated in both environment classes.
- It is numerically testable without launching Isaac Sim.
- It directly reduces the chance that standup training, residual training, LQR diagnostics, and disturbance benchmarks diverge on actuator physics.
- It does not require changing task names, CSV schema, training commands, or reward behavior.
