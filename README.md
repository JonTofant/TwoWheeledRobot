# TwoWheeledRobot

Isaac Lab extension for a two-wheeled leg robot with four CyberGear leg joints and two
current-controlled DDSM115 wheel motors. The STM32 firmware lives in a separate repository.

Scope is the MDPI *Actuators* work: training a joystick-commanded drive/balance policy over
domain randomization derived from measured DDSM115 unit-to-unit variation. The Standup,
ResidualLQR and PureNNBalance tasks and the LQR tooling were removed on 2026-08-05.

## Task

```text
Template-Twowheeledrobot-NNDrive-v0
```

Velocity + yaw-rate joystick commands with integrated position/heading references, 20
observations, 6 actions (4 CyberGear stance targets + 2 wheel currents), generated terrain
(flat / bumps / 3-9 deg slopes), and per-episode randomization of actuator characteristics,
mass/COM, odometry scale and IMU biases.

`Template-Twowheeledrobot-NNDriveDemo-v0` is a presentation-only variant with a hand-built
scene for stills and video. It is never trained against.

The observation and action layouts are the deployment contract and are documented in
`STM32_DEPLOYMENT.md`. They must stay aligned with the firmware.

## Layout

```text
source/TwoWheeledRobot/TwoWheeledRobot/
  tasks/direct/twowheeledrobot/
    __init__.py               # gym registration
    nn_drive_env.py           # the task: actions, observations, rewards, resets
    nn_drive_env_cfg.py       # task parameters, reward weights, randomization ranges
    pure_nn_components.py     # observation/reward/command/disturbance building blocks
    pure_nn_balance_env.py    # base class (NNDriveEnv -> PureNNBalanceEnv -> StandupEnv)
    standup_env.py            # base class: scene, sensors, joint ids, motor model
    nn_drive_demo_env.py      # presentation-only scene
    robot_cfg.py              # USD articulation and actuator config
    sim_params.py             # shared physics and hardware constants
    agents/rsl_rl_nn_drive_cfg.py
  docs/ColectedUSD_v2/        # robot USD asset

scripts/
  rsl_rl/{train,play,cli_args}.py
  train_nn_drive_curriculum.py     # five-stage curriculum
  benchmark_nn_drive.py            # scenario benchmark
  diagnose_turn_failure.py         # velocity / yaw-rate sweeps
  export_pure_nn_current_onnx.py   # TorchScript -> ONNX with deployment scaling
  validate_onnx_policy.py
  record_isaac_demo.py             # stills/clips from the demo scene
  list_envs.py

STM32_DEPLOYMENT.md         # deployment contract: obs/action layout, sign conventions
docs/experiments/           # dated experiment logs
```

`logs/`, `outputs/`, `__pycache__/` and egg-info are generated and gitignored.

## Running

Install the extension (editable) inside your Isaac Lab Python environment:

```bash
python -m pip install -e source/TwoWheeledRobot
```

Train the full curriculum (five stages, auto-resuming, exports at the end):

```bash
python scripts/train_nn_drive_curriculum.py --num_envs 4096 --headless
```

Single stage, or resume a specific run:

```bash
python scripts/rsl_rl/train.py --task Template-Twowheeledrobot-NNDrive-v0 \
  --headless --num_envs 4096 env.curriculum_stage=3 env.terrain_mode=flat
```

Benchmark and diagnose a trained policy:

```bash
python scripts/benchmark_nn_drive.py --policy <run>/exported/policy.pt --num_envs 64 --headless
python scripts/diagnose_turn_failure.py --policy <run>/exported/policy.pt --headless --velocities 0.1 0.3 0.4 0.55
```

Export for the STM32 (20 inputs, 6 outputs):

```bash
python scripts/export_pure_nn_current_onnx.py --policy <run>/exported/policy.pt \
  --output <run>/exported/policy_drive.onnx --obs-dim 20 --cg-outputs 4 \
  --cg-authority-rad 1.5708 --i-max-a 2.0 --require-validation
```

Lint:

```bash
ruff check .
ruff format .
```

## Editing rules that matter

- **Physical parameters and the motor model are duplicated by design** across `sim_params.py`
  and `standup_env.py::_pre_physics_step()`. Change every copy together and re-verify.
- **The observation/action contract is shared with firmware**, which lives in a separate
  repository. `STM32_DEPLOYMENT.md` and `pure_nn_components.py::DriveObservationBuilder` must
  agree with it exactly, including the wheel sign convention (left wheel torque is negated in
  sim because the USD is mirrored). Changing the layout is a breaking change for the firmware.
- **`robot_cfg.py`** is the only place that should reference the USD path and Isaac actuator
  groups. Check joint/body names there before referencing them elsewhere.
- **DDSM115 wheels are current/torque-controlled, never position servos** — zero stiffness,
  zero extra passive damping, PhysX peak torque 2.0 Nm, velocity limit 20.94 rad/s.
- **Keep the actor network small** (`agents/rsl_rl_nn_drive_cfg.py`) — it must fit the STM32.
- Change one concept at a time (motor physics vs. reward vs. observation) and re-verify with
  the relevant diagnostic script rather than batching changes.

There is no automated test suite. Verification means running the training, benchmark and
diagnostic scripts and inspecting the output, since correctness here is "physically plausible
sim behaviour", not unit-test pass/fail.
