# NNDrive experiment log — 2026-07-27/28

Branch `feature/mdpi-actuators` (off `feature/ERK2026`), commit `e518082`.
Task `Template-Twowheeledrobot-NNDrive-v0`. Control dt 15 ms (1 ms physics × decimation 15).
All runs: 4096 envs, RTX 5080, container `isaac-lab-dev`.

## Objective

Hardware symptom: the robot drives away instead of holding station, and the torso is not
level enough to carry a payload. Secondary symptom reported later: driving the robot
outside its trained position range made it "kill itself".

## Starting point

Deployed policy `2026-07-09_17-11-35_stage5`, benchmarked at 64 envs × 15 s:

| scenario | fall rate | rms pitch° | rms roll° |
|---|---|---|---|
| station_keeping | 0.000 | 6.28 | 5.58 |
| drive_forward (0.40 m/s cmd) | 0.000 | 6.69 | 5.35 |
| turn_in_place (1.2 rad/s cmd) | **0.984** | 6.12 | 3.95 |
| drive_and_turn | 0.297 | 6.42 | 4.67 |

## Findings

### 1. PPO exploration-noise runaway (real, fixed)

`Policy/mean_noise_std` grew monotonically **0.17 → 2.04** across curriculum stages 1→5.
Actions are tanh-squashed, so std 2.0 is effectively bang-bang exploration. Cause:
`entropy_coef=0.004` against only ~1.3 reward/step, so the entropy bonus outweighed the
policy-gradient term on `log_std`. Consequence: `pos_err`, pitch and roll flatlined at
0.23 m / 6° / 6° from stage 2 onward and never improved over 2300 iterations.

Fix: `entropy_coef` 0.004 → 0.0005; `gamma` 0.995 → 0.998 (3 s → 7.5 s horizon).
Result: `mean_noise_std` ends at 0.065–0.085 instead of 2.04.

### 2. Unbounded penalties made falling optimal (real, fixed)

`yaw_error` is wrapped to ±π and was penalised unbounded at `0.5·yaw_err²`, reaching
**4.9/step at ±π** — roughly double the ~2.3/step the robot forfeits by falling over.
Past ~110° of accumulated heading error, diving for the floor was the optimal policy.

Fix: bound every per-step penalty below the fall economics (yaw error clamped at 1.0 rad,
attitude rates at 2 rad/s). Worst-case per-term budget verified explicitly.

### 3. Reference integrator windup (real, fixed — the turning fix)

`pos_ref` and `yaw_ref` integrate the joystick command with no anti-windup. The measured
yaw-rate ceiling is ~0.75 rad/s while stage 5 commanded up to 2.0 rad/s, so the heading
reference ran away, `yaw_err` swept past ±π and **wrapped** — a step discontinuity in
observation channel 4.

Evidence (six independent runs, two policies): time-of-death landed at **2.0× the time for
heading error to reach π**.

| w_cmd | achieved | err rate | t to π | died at | ratio |
|---|---|---|---|---|---|
| 1.2 | 0.393 | 0.807 | 3.89 s | 8.29 s | 2.13 |
| 1.6 | 0.119 | 1.481 | 2.12 s | 4.16 s | 1.96 |
| 2.0 | 0.054 | 1.946 | 1.61 s | 3.15 s | 1.95 |

Confirmed by pinning the reference to actual heading (`--pin-yaw-ref`): fall rate at
1.2 rad/s dropped **100% → 20%** with *no retraining*.

Fix: back-calculate both references so neither can run further ahead than its clamp
(`cmd_pos_err_clamp_m = 0.5`, new `cmd_yaw_err_clamp_rad = 1.0`); cap commanded yaw rate to
the measured ceiling. **Result: `turn_in_place` fall rate 98.4% → 7.8%.**

The yaw ceiling itself is *not* motor-limited: 0.86 A of a 2.0 A budget, torque-speed derate
clipping <1% of steps. It also did not lift after retraining with the windup fixed, so it is
a genuine capability limit rather than a learned artefact.

### 4. Disturbance force axes transposed (real bug, fixed — but not the driving cause)

Forward travel is **world/platform +Y** (verified by driving the wheels; platform quaternion
is identity). The ±8 N "human push" was assigned to index 0 = **lateral**, and the ±2.5 N
"lateral" push to index 1 = fore/aft. So every policy ever trained practised against large
sideways shoves a differential drive cannot answer, while the recoverable fore/aft push was
3× too small. Also affects `PureNNBalance`, which shares the generator.

Fixed via named `AXIS_*` constants. **A/B tested (stage 5 only, same stage-4 checkpoint,
300 iterations, axis fix as the only variable): no meaningful effect on driving.**

## Results

Benchmark, 64 envs × 15 s, three policies:

| scenario | fall: deployed / +noise+reward+AW / +axis fix |
|---|---|
| station_keeping | 0.000 / 0.094 / 0.094 |
| drive_forward | 0.000 / 0.062 / 0.078 |
| drive_backward | 0.078 / 0.078 / 0.031 |
| turn_in_place | 0.984 / 0.109 / **0.078** |
| drive_and_turn | 0.297 / 0.141 / **0.078** |
| payload_while_still | 0.234 / 0.203 / 0.219 |

Torso attitude (the payload-carrying requirement), station keeping:
rms pitch **6.28° → 5.22°**, rms roll **5.58° → 4.63°**.

### 5. Exploration collapse into a stand-still attractor (root cause of the driving failure)

A forward-velocity sweep (`--velocities`) on the current policy:

| v_cmd | achieved | mean pitch° | rms current A | derate clip |
|---|---|---|---|---|
| 0.05 | 0.010 | −5.68 | 0.293 | 0.000 |
| 0.20 | 0.005 | −4.66 | 0.264 | 0.000 |
| 0.40 | 0.004 | −4.87 | 0.270 | 0.000 |
| 0.55 | 0.004 | −4.43 | 0.239 | 0.000 |

No velocity response at all — achieved speed is flat at ~0.005 m/s and *decreases* with
command. Current sits at 0.24–0.29 A of a 2.0 A budget, also decreasing, with zero
torque-derate clipping. The robot is not motor-, torque-, or traction-limited. **It is not
trying.**

Same sweep on the original deployed policy (trained at `mean_noise_std` 2.04):

| v_cmd | old achieved / pitch° | new achieved / pitch° |
|---|---|---|
| 0.10 | 0.042 / +6.00 | 0.007 / −5.35 |
| 0.30 | 0.099 / +6.17 | 0.004 / −4.85 |
| 0.55 | 0.128 / +5.42 | 0.004 / −4.43 |

The noisy policy responds monotonically to the command and leans forward to do it. The
low-noise policy is flat and leans the opposite way.

**Cause: `entropy_coef` 0.004 → 0.0005 (finding 1) cured the std runaway and simultaneously
collapsed the policy into a stand-still local optimum.** Compounded by
`cmd_stage_velocity_max_mps[0] = 0.0` — stage 1 trained 200 iterations with **no velocity
command at all**, building a "never move" attractor that later stages could not escape at
std 0.08.

This is not a shaping failure: standing still under a 0.40 m/s command already forfeits
~1.65 reward/step (0.74 vel_track + 0.75 position + 0.16 vel_err) against an alive bonus of
1.0. The incentive is large and the policy still will not drive.

Fix (applied 2026-07-28): `cmd_stage_velocity_max_mps` → `(0.15, 0.30, 0.40, 0.50, 0.55)` so
no standstill-only stage exists; `entropy_coef` → 0.002, between the runaway and the
collapse. Watch `Policy/mean_noise_std` stays under ~0.4.

Also unresolved: `station_keeping` fall rate regressed 0.000 → 0.094, and world drift while
commanded to hold still is 0.51 m over 15 s.

## Caveats on metrics — important for reproducing these numbers

- **`final_pos_err_m` is no longer a drive-away measure.** Reference anti-windup bounds
  `x_odom − pos_ref` to ±0.5 m by construction, so its collapse from 5.5 m → 0.45 m is
  definitional, not behavioural. Use `achieved_speed_mps` / `world_drift_m` (added for this
  reason) instead.
- **`push_while_still` / `push_while_driving` are not comparable across the axis fix.** Those
  scenarios now apply the ±8 N push fore/aft instead of laterally, which is a genuinely
  harder disturbance. Their apparent regression (0.078 → 0.172 / 0.219) reflects a harder
  test, not a worse policy.
- **Training-log metrics repeatedly failed to surface real failures.** The 98%
  `turn_in_place` fall rate was invisible in the training log. Benchmark before believing a
  training curve.
- The axis-fix A/B resumed from a stage-4 checkpoint carrying two stages of wrong-axis
  training; it tests improvement *from there*, not a from-scratch correct-axis policy.

## Tooling changes

- `scripts/diagnose_turn_failure.py` (new) — yaw-rate sweep with `--pin-yaw-ref`.
- `scripts/benchmark_nn_drive.py` — ground-truth `achieved_speed_mps`, `world_drift_m`,
  `rms_roll_deg`; `max_pos_err_m` now uses the unclamped error.
- `scripts/train_nn_drive_curriculum.py` — `--load-run` pins the resume baseline (mtime
  ordering silently chained experiments off each other); iteration counts cut from
  [300,400,500,600,800] to [200,350,200,250,300], 280 → ~137 min.

## Timing

Physics is 99.4% of wall clock; the network is 0.6% (so shrinking the net buys nothing).
`decimation=15` at 1 ms physics means 15 substeps per control decision — the only remaining
large lever (1.5 ms × 10 or 2.5 ms × 6 keeps the 15 ms control period), untested because it
changes contact dynamics.

Per-experiment cost is now ~35 min (stage 5 only, pinned baseline) versus 280 min originally.

## Runs

| run | what |
|---|---|
| `2026-07-09_17-11-35_stage5` | deployed baseline |
| `2026-07-27_16-24-44_stage5` | noise + reward fixes |
| `2026-07-28_08-58-24_stage5` | + reference anti-windup |
| `2026-07-28_10-15-23_stage5` | + disturbance axis fix (current) |

Raw benchmark output: `outputs/2026-07-27/nn_drive_*.txt`.

## Method note

Every root cause found here came from a ~10-20 minute diagnostic sweep against a frozen
checkpoint, not from training. Three rounds of hypothesis-then-retrain (4.5 h, 2.3 h, 0.6 h)
found nothing on the driving symptom; the velocity sweep found it in 20 minutes. Training-log
metrics actively misled: the 98% turn_in_place fall rate and the total absence of velocity
response were both invisible in them.

## Next step

Full curriculum retrain with findings 1-5 applied (~137 min). Success criteria:
`Policy/mean_noise_std` < 0.4, `achieved_speed_mps` > 0.2 at a 0.40 m/s command,
`station_keeping` fall rate back to ~0.
