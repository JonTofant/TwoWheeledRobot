# TwoWheeledRobot

Isaac Lab extension for training a two-wheeled leg robot to stand itself up from fallen poses.

The cleaned project keeps one task:

```text
Template-Twowheeledrobot-Standup-v0
```

The policy controls four CyberGear leg joints and two DDSM115 wheel currents. Episodes start from sampled fallen orientations and terminate when the robot reaches a stable upright stance or times out.

## Layout

```text
source/TwoWheeledRobot/
  config/extension.toml
  TwoWheeledRobot/
    tasks/direct/twowheeledrobot/
      __init__.py                  # Gym registration
      standup_env.py               # task logic: actions, observations, rewards, resets
      standup_env_cfg.py           # task parameters and reward weights
      robot_cfg.py                 # USD articulation and actuator config
      sim_params.py                # shared physics and hardware constants
      agents/rsl_rl_standup_cfg.py # PPO runner config
    docs/ColectedUSD_v2/           # robot USD asset

scripts/rsl_rl/
  train.py
  play.py
  cli_args.py

RealImplementationCode/            # STM32 firmware-side implementation files
STM32_DEPLOYMENT.md                # firmware deployment notes
```

Generated files such as `logs/`, `outputs/`, `__pycache__/`, and egg-info are intentionally ignored and should not be committed.

## Stand-Up Task

Observation space: 16 values, manually normalized for firmware parity.

```text
[0:3]   projected_gravity_b
[3:6]   body angular velocity / 10 rad/s
[6:10]  CyberGear joint extension fractions
[10:16] previous action
```

Action space: 6 values in `[-1, 1]`.

```text
[0:4] CyberGear absolute joint targets
[4]   left wheel current command
[5]   right wheel current command
```

The left wheel torque is negated in simulation because the USD wheel is mirrored. Keep that sign convention aligned with firmware.

## Running

From the repo root:

```bash
python scripts/rsl_rl/train.py \
  --task Template-Twowheeledrobot-Standup-v0 \
  --headless --num_envs 4096
```

Play a trained checkpoint:

```bash
python scripts/rsl_rl/play.py \
  --task Template-Twowheeledrobot-Standup-v0 \
  --num_envs 1
```

Outputs are written under `logs/rsl_rl/standup_two_wheel/`.

## Tuning Notes

- `standup_env_cfg.py` contains reward weights, spawn probabilities, success thresholds, and domain randomization.
- `sim_params.py` contains timing, friction, damping, actuator gains, and the DDSM115 torque constant.
- `robot_cfg.py` is the only place that should reference the USD path and Isaac actuator groups.
- Keep the network in `agents/rsl_rl_standup_cfg.py` small if the policy will run on STM32. The current `[32, 32]` actor is sized for microcontroller deployment.
