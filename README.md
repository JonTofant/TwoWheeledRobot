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

Observation space: 18 values, manually normalized for firmware parity.

```text
[0:3]   projected_gravity_b
[3:6]   body angular velocity / 10 rad/s
[6:10]  CyberGear joint extension fractions
[10:12] DDSM115 wheel velocities / 15.7 rad/s
[12:18] previous action
```

`CyberGear joint extension fractions` are normalized CyberGear joint positions in this joint order:

```text
[front_left, front_right, back_left, back_right]
```

The physical joint limits are `front_left/back_right = [-10 deg, +90 deg]` and `front_right/back_left = [-90 deg, +10 deg]`. The policy sees those as normalized values near `[-1, +1]`, with sign flips on the mirrored joints so positive means "more extended" for every leg.

Action space: 6 values in `[-1, 1]`.

```text
[0:4] CyberGear absolute joint targets
[4]   left wheel current command
[5]   right wheel current command
```

The left wheel torque is negated in simulation because the USD wheel is mirrored. Keep that sign convention aligned with firmware.

The wheel actuator hard limit in simulation is 2.0 Nm, matching the measured DDSM115 peak. The policy command scaling is intentionally lower: `wheel_torque_command_limit = 1.6 Nm`, or about `2.13 A` with `Kt = 0.75 Nm/A`, so retraining keeps a safety margin for the real robot.

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

Run the deterministic LQR-style controller without loading RSL-RL:

```bash
python scripts/lqr_control.py \
  --task Template-Twowheeledrobot-Standup-v0 \
  --num_envs 1 \
  --num_steps 1000
```

The LQR script launches Isaac Sim with the same Hydra task-config pattern as the RL play script, keeps the four CyberGear action channels at a neutral retracted stance (`--cg-neutral-action -1.0`), and computes the left/right wheel current commands from projected gravity, body angular velocity, and wheel velocity feedback. Tune the controller with flags such as `--k-pitch`, `--k-pitch-rate`, `--k-wheel-vel`, `--k-roll`, and `--k-roll-rate`.

Outputs are written under `logs/rsl_rl/standup_two_wheel/`.

## UART Policy Runner

Run an exported JIT policy against the STM32 over UART:

```bash
python scripts/uart_policy_runner.py \
  --policy logs/rsl_rl/standup_two_wheel/<run>/exported/policy.pt \
  --port /dev/ttyACM0 \
  --baud 115200
```

STM32 to host, one JSON object per line:

```json
{"roll":0.0,"pitch":1.57,"yaw":0.0,"gyro":[0.0,0.0,0.0],"cg":[0.0,0.0,0.0,0.0],"ddsm":[0.0,0.0]}
```

Host to STM32, one JSON object per line:

```json
{"cg_target":[0.0,0.0,0.0,0.0],"wheel_current":[0.0,0.0],"action":[0.0,0.0,0.0,0.0,0.0,0.0]}
```

Roll, pitch, yaw, gyro, and CyberGear angles are radians by default. Use `--degrees` if the STM32 packet sends degrees and deg/s. DDSM115 velocities are part of the observation and should be sent as `[left, right]` wheel angular velocity in rad/s using the same sign convention as simulation.

To sanity-check policy outputs without the robot, run a synthetic roll/pitch sweep:

```bash
python scripts/test_policy_angle_sweep.py \
  --policy logs/rsl_rl/standup_two_wheel/<run>/exported/policy.pt
```

This prints normalized actions, CyberGear target angles, and wheel current commands for upright, side-fallen, forward/back-fallen, and diagonal poses. Use it to catch obvious sign mistakes before trying UART on hardware.

## Tuning Notes

- `standup_env_cfg.py` contains reward weights, spawn probabilities, success thresholds, and domain randomization.
- `sim_params.py` contains timing, friction, damping, actuator gains, and the DDSM115 torque constant.
- `robot_cfg.py` is the only place that should reference the USD path and Isaac actuator groups.
- Keep the network in `agents/rsl_rl_standup_cfg.py` small if the policy will run on STM32. The current `[32, 32]` actor is sized for microcontroller deployment.
