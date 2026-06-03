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
[10:12] DDSM115 wheel velocities / 20.94 rad/s
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

The DDSM115 wheels are modeled as current/torque-controlled direct-drive actuators, never as position servos. Wheel joint stiffness and extra passive motor damping are zero, the PhysX peak limit is `2.0 Nm`, and the velocity limit is the documented `200 rpm` no-load speed (`20.94 rad/s`). The torque-speed curve already includes the losses that define no-load speed, so adding motor damping would double-count them.

For conservative training, policy action `+/-1` maps to the rated torque envelope of `+/-0.96 Nm`, or `+/-1.28 A` with `Kt = 0.75 Nm/A`. The environment additionally clamps current to `2.7 A`, torque to `2.0 Nm`, and applies a linear torque-speed envelope that reaches zero torque at `200 rpm`.

Useful validation points are: a free wheel should approach `200 rpm`, `1.5 A` should produce approximately `0.96-1.125 Nm`, and low-speed peak torque should never exceed `2.0 Nm`.

The Waveshare interface also exposes current, velocity, position, mode, and error-code feedback at communication rates up to `500 Hz`. Its raw command ranges are `-8 A` to `+8 A` for current mode, `-330 rpm` to `+330 rpm` for speed mode, and `0 deg` to `360 deg` for position mode. This Isaac Lab task intentionally uses only current/torque mode. Bus overcurrent, phase-current, overtemperature, and five-second locked-rotor protection are not yet modeled as stateful faults.

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

Run the fixed-base DDSM115 current-driven free-spin test without loading RSL-RL:

```bash
python scripts/lqr_control.py \
  --task Template-Twowheeledrobot-Standup-v0 \
  --motor-test-current 0.25
```

The default `free-spin` mode suspends the robot by fixing its base above the ground, exposes a physical-units `RobotState` to `compute_action()`, and converts the returned physical-units `RobotAction` current command into the environment action. Try `--motor-test-current 0.25`, `1.0`, `1.5`, or `2.7`. Use `--left-motor-test-current` and `--right-motor-test-current` for separate commands.

Run the analytical LQR sign and DDSM115 motor-model diagnostic while the robot remains suspended:

```bash
python scripts/lqr_control.py \
  --task Template-Twowheeledrobot-Standup-v0 \
  --test-mode lqr-model \
  --artificial-pitch-deg 1.0
```

This mode prints the analytical `A`, `B`, and `K` matrices, checks that `+1` and `-1` degree pitch produce opposite current commands, and passes the selected artificial pitch command through the shared DDSM115 current/torque model. It is not a balancing test because the wheels have no ground contact.

After suspended-air signs and motor direction are confirmed, the separate floor-contact mode can be started with a small 0.5-3 degree initial pitch:

```bash
python scripts/lqr_control.py \
  --task Template-Twowheeledrobot-Standup-v0 \
  --test-mode lqr-floor \
  --floor-initial-pitch-deg 1.0
```

The CSV and live plots include wheel RPM, wheel angular velocity, desired current, saturated current, current-produced torque, torque-speed limit, and actual applied torque. Live plots use matplotlib and work with Docker display forwarding such as `xhost`. Disable them with `--no-plot`, or list every signal and group with `--list-signals`.

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
