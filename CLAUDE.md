# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An Isaac Lab extension (`source/TwoWheeledRobot`) plus standalone scripts for training and evaluating
control policies for a two-wheeled leg robot (four CyberGear leg joints, two DDSM115 current-controlled
wheel motors). It targets four registered Isaac Lab tasks and includes an STM32 hardware deployment
path (UART JSON protocol) for the policies trained here.

There is no build step and no automated test suite — verification is done by running Isaac Lab
scripts (training, diagnostics, sweeps) and inspecting CSV/plot output, since correctness here means
"physically/numerically plausible sim behavior," not unit-test pass/fail.

## Registered tasks

Defined in `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/__init__.py`:

| Task ID | Env class | Cfg | RSL-RL cfg |
|---|---|---|---|
| `Template-Twowheeledrobot-Standup-v0` | `standup_env.py::StandupEnv` | `standup_env_cfg.py` | `agents/rsl_rl_standup_cfg.py` |
| `Template-Twowheeledrobot-ResidualLQR-v0` | `residual_lqr_env.py::ResidualLqrEnv` (inherits `StandupEnv`) | `residual_lqr_env_cfg.py` | `agents/rsl_rl_residual_lqr_cfg.py` |
| `Template-Twowheeledrobot-PureNNBalance-v0` | `pure_nn_balance_env.py::PureNNBalanceEnv` (inherits `StandupEnv`) | `pure_nn_balance_env_cfg.py` | `agents/rsl_rl_pure_nn_balance_cfg.py` |
| `Template-Twowheeledrobot-NNDrive-v0` | `nn_drive_env.py::NNDriveEnv` (inherits `PureNNBalanceEnv`) | `nn_drive_env_cfg.py` | `agents/rsl_rl_nn_drive_cfg.py` |

Never rename these task IDs without preserving compatibility aliases — training runs, checkpoints, and
scripts reference them by string.

- **Standup**: gets the robot from a fallen pose to a stable stance. The base task; both other tasks
  inherit its reset/pose logic.
- **ResidualLQR**: an RL policy learns a residual correction on top of an analytical LQR balance
  controller. LQR logic here is intentionally duplicated between this env and `scripts/lqr_control.py`
  for frozen-policy evaluation.
- **PureNNBalance**: a from-scratch NN balance controller with no LQR in the loop, direct left/right
  DDSM115 current outputs, curriculum-based disturbances, and sim2real hardening (observation delay,
  yaw drift bias, wheel friction/damping randomization) — see `pure_nn_components.py` for the shared
  observation/reward/disturbance/curriculum building blocks it uses.
- **NNDrive**: joystick-commanded driving/balancing (velocity + yaw-rate commands with integrated
  position/heading references, clamped against odometry drift), 6 actions (4 CyberGear stance targets
  + 2 wheel currents), generated terrain (flat/bumps/inclines), and wider domain randomization
  (mass/inertia scale, platform COM shift, odometry scale, gyro biases, CyberGear gains, force noise).
  Obs (18) / action (6) contract is documented in `STM32_DEPLOYMENT.md` and must stay aligned with the
  STM32 joystick firmware (command slew, `pos_err` clamp at ±0.5 m, CyberGear target slew at 3 rad/s).

## Commands

Install the extension (editable) before running anything, inside your Isaac Lab Python environment:

```bash
python -m pip install -e source/TwoWheeledRobot
```

Train / play (works for any of the three task IDs above):

```bash
python scripts/rsl_rl/train.py --task <TaskId> --headless --num_envs 4096
python scripts/rsl_rl/play.py --task <TaskId> --num_envs 1 --checkpoint logs/rsl_rl/<run_dir>/<run>/model_<iter>.pt
```

Pure NN balance curriculum (five stages, wraps `train.py` as a subprocess, auto-resumes from latest checkpoint):

```bash
python scripts/train_pure_nn_curriculum.py --num_envs 4096 --headless
```

NN drive curriculum (five stages: flat balance → commands → pushes → generated terrain; same auto-resume pattern):

```bash
python scripts/train_nn_drive_curriculum.py --num_envs 4096 --headless
```

Benchmark an NN drive policy (station keeping, command tracking, pushes/payloads; `--terrain generator` for bumps/slopes):

```bash
python scripts/benchmark_nn_drive.py --policy <exported policy.pt> --num_envs 64 --headless
```

Export a trained policy to TorchScript/ONNX (done by `play.py`, or standalone for pure-NN current-output policies):

```bash
python scripts/export_pure_nn_current_onnx.py --policy <run>/model_<iter>.pt
python scripts/validate_onnx_policy.py  # sanity-check exported ONNX vs TorchScript
```

For NN drive policies add `--obs-dim 18 --cg-outputs 4` (6 outputs: 4 CyberGear target radians + 2 wheel currents).

Benchmark a pure NN balance policy against required disturbance scenarios:

```bash
python scripts/benchmark_pure_nn_balance.py --policy <exported policy.pt> --num_envs 64
```

Fixed-base DDSM115 free-spin motor test (no RL, suspends the robot):

```bash
python scripts/lqr_control.py --task Template-Twowheeledrobot-Standup-v0 --motor-test-current 0.25
```

Analytical LQR sign/model diagnostic while suspended (no ground contact):

```bash
python scripts/lqr_control.py --task Template-Twowheeledrobot-Standup-v0 --test-mode lqr-model --artificial-pitch-deg 1.0
```

LQR floor-contact balance evaluation (small initial pitch, real balancing):

```bash
python scripts/lqr_control.py --task Template-Twowheeledrobot-Standup-v0 --test-mode lqr-floor --floor-initial-pitch-deg 1.0 --no-plot
```

`lqr_control.py` also does residual-policy evaluation, disturbance injection, CSV logging, and live
matplotlib plotting; `--list-signals` shows every loggable/plottable signal, `--no-plot` disables plots
(useful headless/in Docker without display forwarding).

Disturbance sweeps and repeatability benchmarks (call `lqr_control.py` as a subprocess):

```bash
python scripts/run_lqr_disturbance_sweep.py --headless --currents 0.5 1.0 1.5 2.0
python scripts/run_step_disturbance_benchmark.py --headless --residual-policy logs/rsl_rl/residual_lqr_two_wheel/<run>/exported/policy.pt
python scripts/run_lqr_vs_residual_repeatability.py
```

Print LQR gains for manual tuning reference (no sim):

```bash
python scripts/calculate_lqr_gains.py
```

List all registered Isaac Lab environments:

```bash
python scripts/list_envs.py
```

Sanity-check a policy's sign conventions with a synthetic pose sweep (no hardware/UART needed):

```bash
python scripts/test_policy_angle_sweep.py --policy logs/rsl_rl/standup_two_wheel/<run>/exported/policy.pt
```

Run an exported policy against real hardware over UART:

```bash
python scripts/uart_policy_runner.py --policy <exported policy.pt> --port /dev/ttyACM0 --baud 115200
```

Lint/format (ruff config lives in `pyproject.toml`; also runs via pre-commit):

```bash
ruff check .
ruff format .
pre-commit run --all-files
```

## Architecture and editing rules

Full file-by-file dependency detail lives in `ARCHITECTURE_DEPENDENCY_MAP.md`, `PROJECT_STRUCTURE_AUDIT.md`,
and `DEVELOPER_GUIDE.md` — read those before non-trivial edits. They predate the `PureNNBalance` task
(added after `ResidualLQR`), so cross-check current files rather than trusting those docs verbatim for
that task. Key points that don't change:

- **Physical parameters and motor model are duplicated by design (for now)**, across
  `sim_params.py`, `standup_env.py::_pre_physics_step()`, `residual_lqr_env.py::_pre_physics_step()`,
  `scripts/lqr_control.py`, and `scripts/calculate_lqr_gains.py`. When changing DDSM115 motor physics,
  robot mass/inertia, or LQR gains, update every copy and re-run a free-spin or LQR floor test
  afterward — do not assume one edit propagates.
- **Observation/action contracts must stay aligned across sim and firmware.** `standup_env.py`'s
  observation/action layout is documented in `README.md` and `STM32_DEPLOYMENT.md`; the UART runner
  (`scripts/uart_policy_runner.py::build_observation()`) and the angle-sweep sanity script must match
  it exactly, including the wheel sign convention (left wheel torque is negated in sim because the USD
  is mirrored).
- **`robot_cfg.py`** is the only place that should reference the USD asset path and Isaac actuator
  groups — check joint/body names here before editing envs or scripts that reference them by name.
- **CSV column names in `scripts/lqr_control.py::sample_to_row()`** are consumed by the plotting
  scripts (`plot_lqr_debug_csv.py`, `plot_control_effort_lqr_vs_residual.py`,
  `plot_single_run_control_effort.py`) and by the sweep/benchmark scripts — don't rename columns
  without updating all of them.
- **DDSM115 wheels are current/torque-controlled, never position servos** — zero stiffness, zero extra
  passive damping (the torque-speed curve already bakes in losses that define no-load speed), PhysX
  peak torque `2.0 Nm`, velocity limit `20.94 rad/s` (200 rpm no-load). Policy action `±1` maps to the
  conservative rated envelope `±0.96 Nm` / `±1.28 A` at `Kt = 0.75 Nm/A`; the env additionally clamps
  current to `2.7 A` and applies a linear torque-speed derate to zero at 200 rpm.
- **Keep RSL-RL actor networks small** (`agents/rsl_rl_*_cfg.py`) if the policy is meant to run on the
  STM32 — the existing `[32, 32]` actors are sized for microcontroller deployment.
- Avoid large refactors while training experiments are active; change one concept at a time
  (motor physics vs. reward vs. observation vs. CSV schema), and re-verify with the relevant
  diagnostic script after each change rather than batching changes.
- `logs/`, `outputs/`, `__pycache__/`, and egg-info are generated and gitignored — never hand-edit or
  commit into them.
