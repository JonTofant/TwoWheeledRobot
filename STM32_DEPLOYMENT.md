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
STM32F446RE). The deployed model takes 21 normalized `float32` observations and
returns 6 commands.

### Observation layout (divide raw value by the listed scale)

```text
 0  pos_err / 0.5 m          clamp(x_odom - pos_ref, -0.5, +0.5) BEFORE dividing
 1  velocity / 1.0 m/s       wheel odometry mean: 0.5*(wL + wR)*R_wheel
 2  pitch / 25 deg           rad
 3  pitch_rate / 4.0 rad/s
 4  sin(yaw - yaw_ref)       no scaling, already [-1, 1] -- RAW difference, do NOT wrap first
 5  cos(yaw - yaw_ref)       no scaling, already [-1, 1] -- RAW difference, do NOT wrap first
 6  yaw_rate / 4.0 rad/s
 7  velocity_cmd / 1.0 m/s   slew-limited joystick command (see below)
 8  yaw_rate_cmd / 2.0 rad/s slew-limited joystick command
 9-12  cg_pos / 1.5708 rad   CyberGear joint angles [fl, fr, bl, br] (pi/2)
13-14  prev_current / 2.0 A  previous wheel current commands [left, right]
15-18  prev_cg_action        previous tanh CyberGear actions, already [-1, 1]
19  roll / 25 deg            rad, from the same IMU as pitch
20  roll_rate / 4.0 rad/s    rad/s, gyro axis matching roll
```

**`yaw_err` changed from a single clamped radian at index 4 to an unclamped
sin/cos pair at [4]/[5] on 2026-08-10** (renumbering everything from index 4
onward — a deliberate one-time break from the append-only convention below,
since this repo's firmware isn't deployed yet and is still ours to redesign
freely). Any policy exported before this date, or any layout with a single
scalar at index 4, is a different, incompatible contract — check the ONNX
input shape and export date, not just the total count.

Why the change: the old `yaw_err` was `wrap_pi(yaw - yaw_ref)`, clamped to
`+-cmd_yaw_err_clamp_rad` (1.0 rad) by keeping `yaw_ref` itself pinned close to
true `yaw` via reference anti-windup (mirroring what `pos_ref` still does).
That clamp did two jobs: kept the observation bounded, and kept the raw
wrapped error from ever sweeping past `+-pi` and wrapping — a step
discontinuity in the "which way to turn" signal that measured **100% fall
rate at a sustained 1.2 rad/s command, versus 20% with the reference pinned**.
It also had a side effect nobody wanted: because the reference silently
absorbs drift beyond the clamp, a robot that slowly spun in place during
station-keeping could show a small, healthy-looking `yaw_err` while its true
heading had drifted far from where it started — the observation literally
could not see it.

`sin(yaw - yaw_ref)` / `cos(yaw - yaw_ref)`, computed from the RAW (unwrapped)
difference, fixes both at once: it is smooth and bounded in `[-1, 1]` for any
error magnitude, with no wrap point to hit no matter how far `yaw_ref` runs
ahead of an unachievable command. Because of this, `yaw_ref` no longer needs
the anti-windup treatment at all — firmware should simply let it integrate
`w_cmd * dt` every tick, same as before, with no back-calculation step. (It
should still be periodically re-wrapped to `(-pi, pi]` against ITSELF — not
against `yaw` — purely so the accumulated radians don't grow to a large float
over a long-running deployment; this is a no-op on `sin`/`cos` of the
difference, not a correctness fix.)

This assumes the firmware's `yaw` comes from the BNO085's onboard sensor
fusion (absolute-ish heading), not raw gyro integration alone — see the sign
convention note below and confirm this on the bench. If `yaw` is itself
derived by dead-reckoning gyro integration with no independent correction,
it carries the same kind of drift `pos_err`'s wheel odometry does, and that
drift needs to be accounted for the same way (`odometry_scale_range` in sim)
before trusting this channel over a long run.

**Roll/roll_rate were added 2026-08-04 (18 -> 20 values, now [19]/[20]).**
They were appended after pitch/pitch_rate rather than grouped next to them so
that indices 0-17 stayed stable at the time — the convention the yaw change
above now deliberately breaks from, for the reasons given there.

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
yaw_ref = wrap_pi(yaw_ref);         // numerical hygiene only, NOT anti-windup -- see above
// errors fed to the network:
pos_err = clamp(x_odom - pos_ref, -0.5f, 0.5f);   // anti-windup for odometry drift
yaw_err_sin = sinf(yaw - yaw_ref);  // RAW difference -- do not wrap_pi() this first
yaw_err_cos = cosf(yaw - yaw_ref);
```

The `pos_err` clamp is essential: it keeps unbounded real-world odometry drift
from pushing the network out of its training distribution (the old balance
policy's ~10 s falls came from exactly this failure mode). `yaw_ref` gets no
equivalent clamp — see above for why it doesn't need one. With zero commands
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
