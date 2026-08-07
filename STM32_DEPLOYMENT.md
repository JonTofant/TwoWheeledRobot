# STM32 Deployment Notes

Deployment contract for the joystick-commanded drive policy. The Isaac Lab task is:

```text
Template-Twowheeledrobot-NNDrive-v0
```

The Standup, ResidualLQR and PureNNBalance sections were removed on 2026-08-05
along with those tasks; this file now covers only the deployed drive policy.

## Sign Convention

### World axes (checked against the USD, 2026-08-05)

```text
Y   fore/aft   — forward is -Y. Driving is along this axis.
X   lateral    — sideways lean (roll). A differential drive cannot correct it.
Z   up         — yaw rotation is about this axis.
```

The axis *assignment* is what the simulation depends on: `pitch` reads the
body-frame gravity Y component, `roll` reads X, yaw is about Z, and the
disturbance `AXIS_*` constants in `pure_nn_components.py` follow the same map.

The **sign** (forward = -Y, not +Y) does not affect any simulation result. The
robot is close to symmetric, every disturbance range is symmetric about zero,
and `x_rel`/`velocity` come from wheel odometry rather than world position, so
the sign only decides which way the robot drives in the world.

**It does matter on hardware.** "Positive velocity command" is defined by the
wheel odometry sign, not by a world axis, so the firmware must map positive
command to whichever physical direction the robot's own forward is — the same
rule as the wheel-direction note below: follow the physical wiring, not the USD.
Getting it backwards gives a robot that balances correctly and drives the wrong
way in response to the joystick.

### Wheel direction

The left wheel USD is mirrored, so simulation negates left wheel torque:

```c
torque_left  = -current_left  * DDSM115_KT;
torque_right =  current_right * DDSM115_KT;
```

Keep the firmware-side motor direction mapping consistent with the physical wiring, not blindly with the USD. The important external behavior is that positive wheel action should help the learned policy perform the same maneuver on hardware as in simulation.

## NN Drive Controller (joystick velocity/yaw commands)

Task:

```text
Template-Twowheeledrobot-NNDrive-v0
```

Policy rate is 66.7 Hz (`dt = 0.015 s`), same as the balance controller. The
actor is `[64, 64]` (~5.8k float32 parameters, ~23 KB — trivially fits the
STM32F446RE). The deployed model takes 20 normalized `float32` observations and
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
 8-11  cg_pos / 1.5708 rad   CyberGear joint angles [fl, fr, bl, br] (pi/2)
12-13  prev_current / 2.0 A  previous wheel current commands [left, right]
14-17  prev_cg_action        previous tanh CyberGear actions, already [-1, 1]
18  roll / 25 deg            rad, from the same IMU as pitch
19  roll_rate / 4.0 rad/s    rad/s, gyro axis matching roll
```

**Roll/roll_rate were added 2026-08-04 (18 -> 20 values).** They are appended
rather than placed next to pitch so that indices 0-17 keep their meaning: the
firmware change is two extra values at the end, not a renumbering. Any policy
exported before that date takes 18 inputs and is not loadable against this
layout — check the ONNX input shape rather than assuming.

Sign convention: roll is rotation about the fore/aft axis (leaning sideways),
positive in the same sense as the sim's `roll_from_projected_gravity`, i.e.
`atan2(g_x, -g_z)` on body-frame projected gravity. `roll_rate` is the gyro
component about that same axis. Getting this sign wrong is worse than omitting
the values, because the policy actively servos roll with the legs — verify it
on the bench before driving the robot.

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
     zero-centred piecewise mapping from each tanh output:
       - tanh=-1 -> that joint's lower limit
       - tanh= 0 -> 0 rad (hardware failsafe pose)
       - tanh=+1 -> that joint's upper limit
     limits [fl, fr, bl, br]:
       lower = [-10, -90, -90, -10] deg
       upper = [+90, +10, +10, +90] deg
     firmware MUST defensively clamp to the same limits and slew-limit the
     applied target at 3.0 rad/s (0.045 rad per 15 ms tick).
4-5  left/right DDSM115 current commands in A (+-2.0 A)
```

The piecewise mapping uses both halves of the policy action without ever asking
for an invalid angle. For example, front-left maps `[-1, 0, +1]` to
`[-10 deg, 0 deg, +90 deg]`, while front-right maps it to
`[-90 deg, 0 deg, +10 deg]`. This intentionally preserves zero action as the
zero-angle hardware failsafe; a single affine lower-to-upper mapping would put
zero action at the range midpoint instead.

Policies trained before this mapping change are incompatible and must not be
re-exported under the new contract without retraining.

Train all curriculum stages (flat → commands → pushes → terrain) and export:

```bash
python scripts/train_nn_drive_curriculum.py --num_envs 4096 --headless
```

Manual export from an existing `policy.pt`:

```bash
python scripts/export_pure_nn_current_onnx.py \
  --policy logs/rsl_rl/nn_drive_two_wheel/<run>/exported/policy.pt \
  --output logs/rsl_rl/nn_drive_two_wheel/<run>/exported/policy_drive.onnx \
  --obs-dim 20 --cg-outputs 4 --i-max-a 2.0 --require-validation
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

## Host <-> STM32 message format

The firmware lives in a separate repository; there is no host-side runner here.
The wire format the policy expects is recorded for reference.

STM32 sends JSON lines:

```json
{"roll":0.0,"pitch":1.57,"yaw":0.0,"gyro":[0.0,0.0,0.0],"cg":[0.0,0.0,0.0,0.0],"ddsm":[0.0,0.0]}
```

Host sends JSON lines:

```json
{"cg_target":[0.0,0.0,0.0,0.0],"wheel_current":[0.0,0.0],"action":[0.0,0.0,0.0,0.0,0.0,0.0]}
```

DDSM115 velocity is part of the policy observation. Send `[left, right]` wheel angular velocity in rad/s using the same sign convention as simulation.
