# STM32 Deployment Notes

These notes describe the cleaned stand-up policy interface. The Isaac Lab task is:

```text
Template-Twowheeledrobot-Standup-v0
```

## Policy Contract

The trained actor takes 18 normalized `float32` observations and returns 6 actions in `[-1, 1]`.

Observations:

```text
0..2    projected gravity in body frame
3..5    body angular velocity / 10 rad/s
6..9    CyberGear joint extension fraction
10..11  DDSM115 wheel velocity / 20.94 rad/s
12..17  previous action
```

If the STM32 sends roll/pitch/yaw instead of projected gravity, convert roll and pitch to projected gravity before inference. Yaw does not affect gravity direction and is not used by this policy.

CyberGear extension fractions are normalized joint positions in this order:

```text
front_left, front_right, back_left, back_right
```

The two mirrored joints use opposite signs so that positive normalized value means "more extended" on every leg. The conversion used by the Python UART runner is:

```text
cg_norm = ((cg_angle * [1,-1,-1,1] + 10deg) / 100deg) * 2 - 1
```

Actions:

```text
0..3  CyberGear target angles
4     left DDSM115 current command
5     right DDSM115 current command
```

The simulation maps wheel actions to current with `wheel_current_max` from `standup_env_cfg.py` and then torque with `DDSM115_KT` from `sim_params.py`. The conservative policy envelope is `0.96 Nm`, or `1.28 A` at `0.75 Nm/A`. The motor model clamps current to `2.7 A`, peak torque to `2.0 Nm`, and reduces available torque linearly to zero at the `200 rpm` no-load speed.

## Sign Convention

The left wheel USD is mirrored, so simulation negates left wheel torque:

```c
torque_left  = -current_left  * DDSM115_KT;
torque_right =  current_right * DDSM115_KT;
```

Keep the firmware-side motor direction mapping consistent with the physical wiring, not blindly with the USD. The important external behavior is that positive wheel action should help the learned policy perform the same maneuver on hardware as in simulation.

## Files To Keep Aligned

- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/standup_env.py`
- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/standup_env_cfg.py`
- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/sim_params.py`
- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/agents/rsl_rl_standup_cfg.py`
- `RealImplementationCode/`

When changing observation normalization, action scaling, motor signs, or the actor network shape, update the STM32 inference code in the same commit.

## Export

Train:

```bash
python scripts/rsl_rl/train.py \
  --task Template-Twowheeledrobot-Standup-v0 \
  --headless --num_envs 4096
```

Play/export from a checkpoint:

```bash
python scripts/rsl_rl/play.py \
  --task Template-Twowheeledrobot-Standup-v0 \
  --num_envs 1
```

Check exported policies under `logs/rsl_rl/standup_two_wheel/<run>/exported/`.

## Pure NN Balance Controller

Task:

```text
Template-Twowheeledrobot-PureNNBalance-v0
```

Policy rate is 50 Hz (`dt = 0.02 s`). The deployed current model takes 8 normalized `float32` observations and returns two wheel current commands in amperes:

```text
0  x_rel / 1.0 m
1  linear_velocity / 1.0 m/s
2  pitch / 25 deg
3  pitch_rate / 4.0 rad/s
4  yaw_error / pi rad
5  yaw_rate / 4.0 rad/s
6  previous_left_current / I_max
7  previous_right_current / I_max
```

The inference-ready ONNX wrapper applies `tanh(actor(obs)) * I_max`; default `I_max = 2.0 A`. Default training does not apply additional command smoothing or hard slew limiting. Optional hardware-safety testing can enable:

```text
I_filtered = alpha * I_previous + (1 - alpha) * I_network
optional slew limit = 0.3 A per 20 ms sample
```

Train all curriculum stages and export the current-output ONNX model:

```bash
python scripts/train_pure_nn_curriculum.py --num_envs 4096 --headless
```

Manual export from an existing `policy.pt`:

```bash
python scripts/export_pure_nn_current_onnx.py \
  --policy logs/rsl_rl/pure_nn_balance_two_wheel/<run>/exported/policy.pt \
  --output logs/rsl_rl/pure_nn_balance_two_wheel/<run>/exported/policy_current.onnx \
  --i-max-a 2.0
```

The export script runs ONNX Runtime validation and fails if max error is `>= 1e-4`.

Benchmark scenarios:

```bash
python scripts/benchmark_pure_nn_balance.py \
  --policy logs/rsl_rl/pure_nn_balance_two_wheel/<run>/exported/policy.pt \
  --num_envs 64 --headless
```

## NN Drive Controller (joystick velocity/yaw commands)

Task:

```text
Template-Twowheeledrobot-NNDrive-v0
```

Policy rate is 66.7 Hz (`dt = 0.015 s`), same as the balance controller. The
actor is `[64, 64]` (~5.8k float32 parameters, ~23 KB — trivially fits the
STM32F446RE). The deployed model takes 18 normalized `float32` observations and
returns 6 commands.

### Observation layout (divide raw value by the listed scale)

```text
 0  pos_err / 0.5 m          clamp(x_odom - pos_ref, -0.5, +0.5) BEFORE dividing
 1  velocity / 1.0 m/s       wheel odometry mean: 0.5*(wL + wR)*R_wheel
 2  pitch / 25 deg           rad
 3  pitch_rate / 4.0 rad/s
 4  yaw_err / 1.5 rad        wrap(yaw - yaw_ref) to [-pi, pi] BEFORE dividing
 5  yaw_rate / 4.0 rad/s
 6  velocity_cmd / 1.0 m/s   slew-limited joystick command (see below)
 7  yaw_rate_cmd / 2.0 rad/s slew-limited joystick command
 8-11  cg_pos / 0.45 rad     CyberGear joint angles [fl, fr, bl, br]
12-13  prev_current / 2.0 A  previous wheel current commands [left, right]
14-17  prev_cg_action        previous tanh CyberGear actions, already [-1, 1]
```

### Joystick contract (must run on the STM32 every 15 ms tick)

```c
// slew-limit raw joystick input (1.0 m/s^2, 4.0 rad/s^2):
v_cmd += clamp(v_joy - v_cmd, -1.0f * dt, +1.0f * dt);
w_cmd += clamp(w_joy - w_cmd, -4.0f * dt, +4.0f * dt);
// integrate references:
pos_ref += v_cmd * dt;              // m
yaw_ref += w_cmd * dt;              // rad
// errors fed to the network:
pos_err = clamp(x_odom - pos_ref, -0.5f, 0.5f);   // anti-windup for odometry drift
yaw_err = wrap_pi(yaw - yaw_ref);
```

The `pos_err` clamp is essential: it keeps unbounded real-world odometry drift
from pushing the network out of its training distribution (the old balance
policy's ~10 s falls came from exactly this failure mode). With zero commands
the same terms give station keeping, including on inclines.

### Action layout (ONNX output `commands`, after built-in tanh scaling)

```text
0-3  CyberGear position targets in rad [fl, fr, bl, br]
     range +-0.45 rad around zero stance; firmware MUST additionally:
       - clamp to joint limits (fl/br: [-10, +90] deg, fr/bl: [-90, +10] deg)
       - slew-limit the applied target at 3.0 rad/s (0.045 rad per 15 ms tick)
4-5  left/right DDSM115 current commands in A (+-2.0 A)
```

Train all curriculum stages (flat → commands → pushes → terrain) and export:

```bash
python scripts/train_nn_drive_curriculum.py --num_envs 4096 --headless
```

Manual export from an existing `policy.pt`:

```bash
python scripts/export_pure_nn_current_onnx.py \
  --policy logs/rsl_rl/nn_drive_two_wheel/<run>/exported/policy.pt \
  --output logs/rsl_rl/nn_drive_two_wheel/<run>/exported/policy_drive.onnx \
  --obs-dim 18 --cg-outputs 4 --cg-authority-rad 0.45 --i-max-a 2.0
```

Benchmark station keeping, command tracking, and disturbances (add
`--terrain generator` for the bumps/slopes mix):

```bash
python scripts/benchmark_nn_drive.py \
  --policy logs/rsl_rl/nn_drive_two_wheel/<run>/exported/policy.pt \
  --num_envs 64 --headless
```

Files to keep aligned for this task: `nn_drive_env.py`, `nn_drive_env_cfg.py`,
`pure_nn_components.py` (DriveObservationBuilder / CommandGenerator /
CyberGearStanceProcessor), `agents/rsl_rl_nn_drive_cfg.py`, and the STM32
inference + joystick code.

## UART Runner

Host-side runner:

```bash
python scripts/uart_policy_runner.py \
  --policy logs/rsl_rl/standup_two_wheel/<run>/exported/policy.pt \
  --port /dev/ttyACM0 \
  --baud 115200
```

STM32 sends JSON lines:

```json
{"roll":0.0,"pitch":1.57,"yaw":0.0,"gyro":[0.0,0.0,0.0],"cg":[0.0,0.0,0.0,0.0],"ddsm":[0.0,0.0]}
```

Host sends JSON lines:

```json
{"cg_target":[0.0,0.0,0.0,0.0],"wheel_current":[0.0,0.0],"action":[0.0,0.0,0.0,0.0,0.0,0.0]}
```

DDSM115 velocity is part of the policy observation. Send `[left, right]` wheel angular velocity in rad/s using the same sign convention as simulation.
