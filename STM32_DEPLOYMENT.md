# STM32 Deployment Notes

Deployment contract for the joystick-commanded drive policy. The task actually
trained and deployed is:

```text
Template-Twowheeledrobot-NNDriveFixedStance-v0       (MLP)
Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0    (GRU)
```

The full 6-action `Template-Twowheeledrobot-NNDrive-v0` (legs under policy
control) is a related but currently-untrained task, covered separately near
the end of this document for reference only — see "NN Drive Controller" below
for which sections apply to what.

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

**Two registered tasks share this controller family.** The full 6-action task
(`Template-Twowheeledrobot-NNDrive-v0`, legs policy-controlled) is documented
at the end of this section for reference, but **it is not what is currently
trained or deployed.** Every policy actually trained and exported to date
(the point/range/range+GRU comparison, 2026-08-10) uses the fixed-stance
variant:

```text
Template-Twowheeledrobot-NNDriveFixedStance-v0       (MLP)
Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0    (GRU, same env, only the
                                                       network differs)
```

Both register the *identical* `NNDriveFixedStanceEnvCfg` — observation, reward,
dynamics and randomization are byte-identical between the MLP and GRU arms;
only `rsl_rl_cfg_entry_point` (the policy network) differs. **This is the
contract to implement.** Policy rate is 66.7 Hz (`dt = 1/66.7 ≈ 0.015 s`). The
MLP actor is `[64, 64]` (~5.8k float32 parameters). The GRU actor is a single
`GRU(13, 64)` memory layer feeding the same `[64, 64]` MLP head (~22k
parameters) — see the GRU section below, its calling contract is different.

### Observation vector — 13 values, `float32`, in this order

Every entry is *raw value divided by the listed scale* before being fed to the
network. All are measured/computed every 15 ms tick from the robot's own
sensors and the joystick reference state below — **none of them are something
firmware invents; they are either a direct sensor reading, a simple derived
quantity, or the integrated joystick reference.**

| # | Name | Scale | Formula / measurement | Notes |
|---|------|-------|------------------------|-------|
| 0 | `pos_err` | 0.5 m | `clamp(x_odom - pos_ref, -0.5, +0.5)` | see joystick contract below |
| 1 | `velocity` | 1.0 m/s | `0.5*(wL + wR)*R_wheel` | wheel odometry mean; `wL`/`wR` = measured wheel angular velocity, rad/s, **left/right sign per the wiring note below** |
| 2 | `pitch` | 25 deg (rad) | `atan2(g_y, -g_z)` on body-frame gravity | from BNO085 |
| 3 | `pitch_rate` | 4.0 rad/s | gyro, axis matching pitch | from BNO085 |
| 4 | `sin(yaw - yaw_ref)` | 1.0 (already [-1,1]) | `sinf(yaw - yaw_ref)` | **raw difference — do NOT `wrap_pi()` first** |
| 5 | `cos(yaw - yaw_ref)` | 1.0 (already [-1,1]) | `cosf(yaw - yaw_ref)` | same |
| 6 | `yaw_rate` | 4.0 rad/s | gyro, yaw axis | from BNO085 |
| 7 | `velocity_cmd` | 1.0 m/s | `v_cmd` (slew-limited joystick, see below) | |
| 8 | `yaw_rate_cmd` | 2.0 rad/s | `w_cmd` (slew-limited joystick, see below) | |
| 9 | `prev_current_left` | 2.0 A | previous tick's **actually-sent** left current command | see note below |
| 10 | `prev_current_right` | 2.0 A | previous tick's **actually-sent** right current command | see note below |
| 11 | `roll` | 25 deg (rad) | `atan2(g_x, -g_z)` on body-frame gravity | from BNO085, same IMU as pitch |
| 12 | `roll_rate` | 4.0 rad/s | gyro, axis matching roll | from BNO085 |

**CyberGear legs are NOT part of the observation or action vector at all** in
this task — see "Legs (not policy-controlled)" below. That is 8 fewer
observations and 4 fewer actions than the full 6-action task; if you're
looking at a 21-observation / 6-action layout anywhere, that's the *other*
task, not this one.

**`prev_current` (obs [9]/[10]) must be the command the previous tick actually
sent to the DDSM115** (the ONNX output from the previous inference, i.e. the
same value your firmware wrote to the current-command register), not a sensor
reading of measured current. In sim this is `command_current` — the value
*after* delay/gain/deadzone/bias/current-limit/lag, i.e. what was actually
applied to the wheel joint that step. Feeding back a measured/estimated
current instead would be feeding the network something it never saw in
training.

### Sensor noise/bias: nothing to add on the firmware side

Training randomizes a per-episode bias/noise draw on pitch, pitch-rate, roll,
roll-rate, yaw-rate, and velocity (from wheel odometry) — e.g. pitch carries a
`±3°` mounting-bias draw plus `~0.15°` Gaussian noise per step, velocity
carries a `±3%` odometry-scale error, etc. (exact ranges in
`nn_drive_env_cfg.py` / `ExportedPolicy/*/params/env.yaml`). **This is not
something firmware implements** — it exists so the trained policy is already
robust to whatever fixed bias and noise floor your actual BNO085 mounting and
wheel odometry happen to have. Firmware just reports its best real
measurement; do not add synthetic noise or try to correct for a "typical"
bias.

`yaw` (used for [4]/[5]) is assumed to come from the BNO085's onboard sensor
fusion (an absolute-ish heading), not raw gyro integration — training applies
**zero** simulated drift/noise to it, unlike every other channel. If firmware
derives `yaw` by integrating gyro alone with no independent correction, it
will drift in a way the policy was never trained to expect; confirm the
fusion mode on the bench before trusting this channel over a multi-minute run.

### Joystick contract (run every 15 ms tick)

```c
// slew-limit raw joystick input (1.0 m/s^2, 4.0 rad/s^2):
v_cmd += clamp(v_joy - v_cmd, -1.0f * dt, +1.0f * dt);
w_cmd += clamp(w_joy - w_cmd, -4.0f * dt, +4.0f * dt);
// integrate references:
pos_ref += v_cmd * dt;              // m
yaw_ref += w_cmd * dt;              // rad
yaw_ref = wrap_pi(yaw_ref);         // numerical hygiene only, NOT anti-windup -- see below
// errors fed to the network:
pos_err = clamp(x_odom - pos_ref, -0.5f, 0.5f);   // anti-windup for odometry drift -- ONLY position gets this
yaw_err_sin = sinf(yaw - yaw_ref);  // RAW difference -- do not wrap_pi() this first
yaw_err_cos = cosf(yaw - yaw_ref);
```

**Only `pos_ref` gets anti-windup.** `pos_err`'s hard clamp exists because
wheel odometry has no independent correction and can drift without bound —
the clamp keeps the network's input bounded and matches what it was trained
on. `yaw_ref` gets no equivalent back-calculation step: `sin`/`cos` of the raw
(unwrapped) difference is already bounded in `[-1, 1]` for any error
magnitude, so there is no unbounded-input risk to guard against on that axis,
and the earlier clamped-scalar version's own wrap-discontinuity failure mode
(measured 100% fall rate at a sustained 1.2 rad/s command with the raw wrapped
value vs. 20% with the reference pinned) doesn't apply either, since sin/cos
has no wrap seam to hit. `yaw_ref`'s own periodic re-wrap is purely so the
accumulated radian count doesn't grow unbounded as a float over a long
deployment — it is a no-op on `sin(yaw - yaw_ref)`/`cos(yaw - yaw_ref)`, not a
behavior change.

With zero joystick input the same terms give station-keeping, including on
inclines.

### Action vector — 2 values, `float32`: `[current_left_A, current_right_A]`

**The ONNX graph's output is the final current command in Amps, ready to send
directly to the DDSM115 — tanh-squashing and the ×2.0 A scale are already
baked into the graph** (`export_pure_nn_current_onnx.py`'s
`append_deployment_output`). Firmware does not apply tanh or any scaling
itself; the two output floats are the commands.

**No slew-limiting or smoothing on the current command is active for these
policies** — confirmed directly against the trained config
(`ExportedPolicy/*/params/env.yaml`): `action_smoothing_alpha: 0.0`,
`enable_current_slew_limit: false`, `hardware_safe_current_slew_limit: false`.
The code path for a hardware-safety slew limiter exists
(`CurrentActionProcessor` in `pure_nn_components.py`, `current_slew_limit_a`)
but was **not** active during training — the policy was never exposed to it,
so firmware must not add one. Send exactly what the network outputs, every
tick.

**Wheel direction**, per the sign-convention section above:

```c
torque_left  = -current_left  * DDSM115_KT;   // left wheel USD is mirrored in sim
torque_right =  current_right * DDSM115_KT;
```

Match this to the physical wiring, not blindly to the USD.

### What the motor-model randomization means for firmware: nothing to implement

Training randomizes per-episode motor gain (±3%), deadzone (0.031-0.078 A,
measured via EMB-18), bias (0, measured), current-loop time constant
(5-10 ms), and current limit (1.9-2.1 A) on top of the current command. **All
of these model properties the real DDSM115 already has physically** —
per-unit manufacturing variation in its gain, its own real deadzone, its own
real current-loop response time. Firmware sends the ONNX's raw current
command to the DDSM115's command bus and lets the DDSM115's own internal
(real) hardware handle the rest, exactly as it already does. The
randomization's job is making the *policy* robust to not knowing which
specific unit it's driving — it is not asking firmware to reimplement a motor
model.

The one exception firmware genuinely must replicate is the **torque-speed
limiter's sign convention**: back-EMF only derates torque doing positive work
against rotation (motoring); torque opposing rotation (braking) is
current/thermal-limited, not back-EMF-limited, and must not be derated the
same way. This is enforced by the DDSM115's own firmware already (it is a
property of the real motor+driver, like the items above), so again nothing
for the STM32 side to add — noted here only so it isn't mistaken for
something requiring a workaround if braking behavior looks stronger than
motoring behavior near top speed. That's correct.

**Action delay**: training randomizes a 0-or-1-tick delay on the applied
current command (50/50 per episode) to cover ordinary bus/processing latency.
This is not something to add deliberately — whatever latency your real
command path naturally has (ideally within this range) is what the policy was
trained to tolerate. The same 0-or-1-tick randomization is separately applied
to the *observation* vector as a whole (not just current) for the same
reason.

### Legs (not policy-controlled)

The CyberGear legs are **not** part of this task's observation or action
vector. `leg_action_mode = "fixed"`: all four legs are held at a constant
target, `fixed_leg_stance_rad = (0, 0, 0, 0)` — i.e. **all four CyberGears at
0 rad**, the hardware failsafe pose, via ordinary position control at the sim
nominal gains (`kp=30 Nm/rad, kd=3 Nm·s/rad`, matching `cybergear.c`). This is
completely independent of the neural network; firmware just holds this fixed
pose the whole time the drive policy is active.

### GRU (recurrent) policy — different calling contract

`Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0`'s exported ONNX has **two
inputs and two outputs**, not one of each:

```text
inputs:  obs   float32[1, 13]   -- identical 13 values above, same order/scale
         h_in  float32[1, 1, 64]  -- (num_layers=1, batch=1, hidden_dim=64)
outputs: commands  float32[1, 2]   -- same current-command contract as the MLP
         h_out     float32[1, 1, 64]
```

Firmware must:
1. Hold a persistent `h` buffer, `64` floats, zero-initialized at boot and at
   any deliberate policy reset (e.g. re-arming after an E-stop or picking the
   robot up).
2. Every tick: run inference with `(obs, h)`, get `(commands, h_out)`, send
   `commands` to the motor drivers, then set `h = h_out` for the *next*
   tick's `h_in`. **Do not re-zero `h` every tick** — that discards the
   memory the whole point of this policy is to have, and would make it behave
   like a badly undertrained memoryless policy rather than the trained
   recurrent one.
3. Never mix `h` buffers across the MLP and GRU policies, or between the
   FixedStance and any other GRU-shaped export — the buffer is only valid
   paired with the exact ONNX graph that produced it.

Check `graph.output` count on any exported `policy_drive.onnx` before wiring
it up: 2 outputs = plain MLP (this section's simple contract), 3 outputs =
GRU (this section), 4 = LSTM (not currently used, but the export script
supports it) with an additional `c_in`/`c_out` cell-state pair following the
same rule as `h`.

### Quick reference: what firmware implements vs. doesn't

| | Firmware implements | Firmware does NOT implement |
|---|---|---|
| Observations | Read all sensors, compute pos_err/yaw_ref/sin/cos, divide by scale | Any noise, bias, or drift injection |
| Actions | Send ONNX output directly to DDSM115 current registers | tanh, ×2.0 A scaling, slew-limiting, smoothing (all baked in or inactive) |
| Motor model | Nothing — DDSM115's own firmware handles gain/deadzone/lag/limit | Any current-loop model, torque-speed limiter (DDSM115 firmware's job) |
| Legs | Hold all 4 CyberGears at 0 rad via normal position control | Anything driven by the network — they're not connected to it |
| GRU state | Persist `h`, thread it through every tick, zero at reset | — |
| Delay | Nothing — just whatever your bus naturally has | Deliberately adding delay |

### Policies trained before 2026-08-10 are incompatible

The observation layout changed twice in the last redesign pass (roll/roll_rate
appended 2026-08-04; `yaw_err` replaced by sin/cos and everything renumbered
2026-08-10) and the CyberGear action mapping changed earlier still. Check the
ONNX input/output shapes and the export date before assuming any existing
`policy_drive.onnx` matches this contract — do not guess from the file count
alone.

### Where the actual exported policies are

The three trained comparison arms (point / range / range+GRU, all seed 42,
2026-08-10) are archived at:

```text
ExportedPolicy/fixedstance_point_2026-08-10/<stage5 run>/exported/policy_drive.onnx      (MLP, baseline)
ExportedPolicy/fixedstance_range_2026-08-10/<stage5 run>/exported/policy_drive.onnx      (MLP, proposed 1)
ExportedPolicy/fixedstance_range_gru_2026-08-10/<stage5 run>/exported/policy_drive.onnx  (GRU, proposed 2)
```

Each archived run directory also has `params/env.yaml` (exact resolved config,
including every DR range quoted above), `selected_checkpoint.json` (which
checkpoint was selected and why), and the raw benchmark JSON — see
`scripts/archive_run_for_paper.py` for what's kept and why.

Training and export commands for this task:

```bash
# Full 5-stage curriculum (MLP):
python scripts/train_nn_drive_curriculum.py \
  --task Template-Twowheeledrobot-NNDriveFixedStance-v0 \
  --experiment-name nn_drive_fixed_stance \
  --num_envs 4096 --headless

# Same, GRU:
python scripts/train_nn_drive_curriculum.py \
  --task Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0 \
  --experiment-name nn_drive_fixed_stance \
  --num_envs 4096 --headless

# Manual export from an existing policy.pt (obs-dim/cg-outputs derived
# automatically from --task inside train_nn_drive_curriculum.py; by hand:
python scripts/export_pure_nn_current_onnx.py \
  --policy <run>/exported/policy.pt \
  --output <run>/exported/policy_drive.onnx \
  --obs-dim 13 --cg-outputs 0 --i-max-a 2.0 --require-validation

# Benchmark (station keeping, command tracking, disturbances; add
# --terrain generator for the bumps/slopes mix):
python scripts/benchmark_nn_drive.py \
  --task Template-Twowheeledrobot-NNDriveFixedStance-v0 \
  --checkpoint <run>/model_<N>.pt --num_envs 64 --headless
```

Files to keep aligned for this task: `nn_drive_env.py`, `nn_drive_env_cfg.py`
(`NNDriveFixedStanceEnvCfg`), `pure_nn_components.py`
(`DriveObservationBuilder` / `CommandGenerator` / `CurrentActionProcessor`),
`agents/rsl_rl_nn_drive_cfg.py` (`NNDriveFixedStancePPORunnerCfg` /
`NNDriveFixedStanceGRUPPORunnerCfg`), and the STM32 inference + joystick code.

---

## Full 6-action task (`NNDrive-v0`) — reference only, not currently deployed

The rest of this section describes `Template-Twowheeledrobot-NNDrive-v0`, the
6-action variant with the CyberGear legs under policy control. No policy on
this variant has been trained or exported since the 2026-08-10 observation
redesign; if you revive this task, everything above about the 13-observation
contract does not apply — recompute the equivalent 21-observation contract
(4 CyberGear angles + 4 CyberGear previous actions inserted, 6 actions instead
of 2) the same way this section derived the 13-observation one, and update
this document accordingly rather than trusting the numbers that follow, which
predate the yaw redesign.

### Legacy observation layout (predates the 2026-08-10 yaw redesign)

```text
 0  pos_err / 0.5 m          clamp(x_odom - pos_ref, -0.5, +0.5) BEFORE dividing
 1  velocity / 1.0 m/s       wheel odometry mean: 0.5*(wL + wR)*R_wheel
 2  pitch / 25 deg           rad
 3  pitch_rate / 4.0 rad/s
 4  yaw_err / 1.5 rad        wrap(yaw - yaw_ref) to [-pi, pi] BEFORE dividing -- STALE, see above
 5  yaw_rate / 4.0 rad/s
 6  velocity_cmd / 1.0 m/s   slew-limited joystick command
 7  yaw_rate_cmd / 2.0 rad/s slew-limited joystick command
 8-11  cg_pos / 1.5708 rad   CyberGear joint angles [fl, fr, bl, br] (pi/2)
12-13  prev_current / 2.0 A  previous wheel current commands [left, right]
14-17  prev_cg_action        previous tanh CyberGear actions, already [-1, 1]
18  roll / 25 deg            rad, from the same IMU as pitch
19  roll_rate / 4.0 rad/s    rad/s, gyro axis matching roll
```

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

## Host <-> STM32 message format (bench/host-inference setup — confirm still in use)

The firmware lives in a separate repository; there is no host-side runner here.
This wire format is for a setup where a **host PC runs the ONNX policy and
streams commands to the STM32** over serial, rather than running inference
on-device — a different integration pattern from the rest of this document,
which assumes on-device inference. **Confirm this is still the intended path
before relying on it**; it predates the FixedStance-only contract above and
was not re-verified when that section was written.

If still in use, updated for the FixedStance (2-action) contract — STM32
sends JSON lines:

```json
{"roll":0.0,"pitch":1.57,"yaw":0.0,"gyro":[0.0,0.0,0.0],"cg":[0.0,0.0,0.0,0.0],"ddsm":[0.0,0.0]}
```

`cg` here is a **telemetry reading** of the four CyberGear joint angles, not
part of the policy's observation vector (see "Legs (not policy-controlled)"
above) — still worth sending for logging/monitoring even though the network
never sees it. `ddsm` is `[left, right]` wheel angular velocity in rad/s,
same sign convention as simulation — this feeds `velocity` (obs [1]) and
`x_odom` (for `pos_err`, obs [0]) host-side.

Host sends JSON lines:

```json
{"cg_target":[0.0,0.0,0.0,0.0],"wheel_current":[0.0,0.0]}
```

`cg_target` is always `[0,0,0,0]` for this task (the fixed stance — not
policy output). `wheel_current` is the 2-element ONNX output directly, in
Amps, already tanh-scaled — no further transform. There is no separate
`action` array for this task; the old 6-element one was for the full
`NNDrive-v0` task's `[cg_target(4), wheel_current(2)]` layout, which doesn't
apply here.
