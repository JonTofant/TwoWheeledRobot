# Developer Guide

## Purpose of This Project

This is an Isaac Sim / Isaac Lab project for a two-wheeled balancing robot. It currently includes:

- a standup RL task,
- a residual RL over LQR balancing task,
- an inline DDSM115 wheel motor model,
- LQR diagnostic and evaluation scripts,
- pulse and step disturbance benchmarks,
- hardware deployment helper scripts.

## Most Important Files

| File | Purpose |
|---|---|
| `scripts/rsl_rl/train.py` | Isaac Lab / RSL-RL training entry point for registered tasks. |
| `scripts/rsl_rl/play.py` | Loads checkpoints, runs policies, and exports TorchScript/ONNX policies. |
| `scripts/lqr_control.py` | Main LQR diagnostic/evaluation script; includes LQR, residual-policy eval, disturbances, CSV logging, and plotting. |
| `scripts/calculate_lqr_gains.py` | Prints 4-state LQR gains for manual tuning/reference. |
| `scripts/run_lqr_disturbance_sweep.py` | Runs pulse-style LQR disturbance sweeps through `lqr_control.py`. |
| `scripts/run_lqr_vs_residual_repeatability.py` | Repeats pulse disturbance tests for LQR and residual PPO. |
| `scripts/run_step_disturbance_benchmark.py` | Runs step disturbance benchmarks for LQR and residual PPO. |
| `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/standup_env.py` | Standup task environment: actions, observations, rewards, resets, DDSM115 motor logic. |
| `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/residual_lqr_env.py` | Residual RL over LQR environment: LQR currents, residual actions, training disturbances, residual rewards. |
| `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/standup_env_cfg.py` | Standup task config: timing, sensors, spawn, reward weights, action scaling. |
| `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/residual_lqr_env_cfg.py` | Residual LQR config: observation/action sizes, residual limits, training disturbance distribution. |
| `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/robot_cfg.py` | Robot USD articulation and actuator group configuration. |
| `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/sim_params.py` | Shared simulation, friction, solver, CyberGear, and DDSM115 constants. |
| `scripts/uart_policy_runner.py` | Runs exported TorchScript standup policy over UART for hardware-side testing. |

## I Want To Change...

| Goal | Edit here first | Notes |
|---|---|---|
| Change residual RL reward | `residual_lqr_env.py::_get_rewards()` | Keep evaluation reward in `scripts/lqr_control.py::residual_reward()` consistent. |
| Change standup reward | `standup_env.py::_get_rewards()` and `standup_env_cfg.py` | Reward weights mostly live in config. |
| Change residual observation | `residual_lqr_env.py::_get_observations()` | Must match `scripts/lqr_control.py::residual_observation()` for frozen policy evaluation. |
| Change standup observation | `standup_env.py::_get_observations()` | Keep `scripts/uart_policy_runner.py::build_observation()` aligned for exported policy deployment. |
| Change LQR gains/model | `residual_lqr_env.py` and `scripts/lqr_control.py` | These currently duplicate LQR logic. Also check `scripts/calculate_lqr_gains.py`. |
| Change DDSM115 motor physics | `standup_env.py::_pre_physics_step()` and `residual_lqr_env.py::_pre_physics_step()` | Keep both copies identical until motor model is extracted. |
| Change pulse/step disturbance | `scripts/lqr_control.py::actuator_disturbance_current()` | Sweep scripts call this indirectly. |
| Change training disturbance distribution | `residual_lqr_env.py::_sample_disturbance()` and `residual_lqr_env_cfg.py` | This affects residual RL training. |
| Change robot physical parameters | `sim_params.py`, `residual_lqr_env.py`, `scripts/lqr_control.py`, `scripts/calculate_lqr_gains.py` | Currently duplicated; update carefully. |
| Change robot USD or actuator groups | `robot_cfg.py` | Check joint/body names used by envs and scripts. |
| Change CSV logging | `scripts/lqr_control.py::sample_to_row()` | Downstream plotting scripts depend on column names. |
| Run residual RL training | `scripts/rsl_rl/train.py` | Use task `Template-Twowheeledrobot-ResidualLQR-v0`. |
| Run LQR floor evaluation | `scripts/lqr_control.py` | Use `--test-mode lqr-floor`. |
| Run pulse benchmark | `scripts/run_lqr_disturbance_sweep.py` | Calls `lqr_control.py` as subprocess. |
| Run step benchmark | `scripts/run_step_disturbance_benchmark.py` | Calls `lqr_control.py` as subprocess. |
| Run hardware UART policy | `scripts/uart_policy_runner.py` | Uses exported TorchScript policy. |

## Common Commands

```bash
python scripts/rsl_rl/train.py --task Template-Twowheeledrobot-ResidualLQR-v0 --headless --num_envs 4096
python scripts/rsl_rl/play.py --task Template-Twowheeledrobot-ResidualLQR-v0 --num_envs 1 --checkpoint logs/rsl_rl/residual_lqr_two_wheel/<run>/model_<iter>.pt
python scripts/lqr_control.py --task Template-Twowheeledrobot-Standup-v0 --test-mode lqr-floor --floor-initial-pitch-deg 1.0 --no-plot
python scripts/run_lqr_disturbance_sweep.py --headless --currents 0.5 1.0 1.5 2.0
python scripts/run_step_disturbance_benchmark.py --headless --residual-policy logs/rsl_rl/residual_lqr_two_wheel/<run>/exported/policy.pt
```

## Current Architecture Notes

- `ResidualLqrEnv` inherits from `StandupEnv`.
- DDSM115 motor logic is currently duplicated in both environment files.
- LQR logic is currently duplicated between residual training and diagnostic/evaluation scripts.
- Residual observation and residual reward logic are duplicated between training and frozen-policy evaluation.
- Do not rename Isaac task IDs unless compatibility aliases are preserved.
- Do not rename CSV columns without updating analysis scripts.

## Safe Editing Rules

1. Change one concept at a time.
2. After changing motor physics, run a free-spin or LQR floor test.
3. After changing residual observations, verify frozen policy evaluation still receives the same observation order.
4. After changing reward, document the old and new reward formula.
5. After changing CSV logging, run at least one plotting script.
6. Keep old task names working.
7. Avoid large refactors while training experiments are active.

## Future Refactor Plan

1. Extract DDSM115 motor model.
2. Extract shared physical parameter source.
3. Extract LQR controller.
4. Extract residual observation builder.
5. Extract residual reward function.
6. Extract disturbance generators.
7. Only later simplify scripts and configs.
