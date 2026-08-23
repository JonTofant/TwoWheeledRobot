# Noise floor, deadzone and breakaway — 2026-08-23

Branch `feature/hold-smoothness`. Task `NNDriveFixedStanceGRU-v0`, stage 5 only,
4096 envs, 150-iteration fine-tunes. Raw evidence in `data/2026-08-23-breakaway/`
(runs live under the gitignored `logs/`).

Follows `2026-08-21-hold-smoothness.md`, which recorded the AC-penalty reward
terms failing their gate.

## 1. The reward terms did nothing. The noise floor did everything.

Same baseline, same 150 iterations, same metric:

| | `hold_pitch_ac_deg` | `hold_pos_err_abs` | stage-5 gate |
|---|---|---|---|
| floor + the three steadiness terms | 2.18 -> 1.01 | 0.083 -> 0.136 | **FAILED** |
| floor alone | 2.22 -> 1.00 | 0.082 -> 0.101 | **PASSED** |

The terms moved their own target metric from 1.01 to 1.00 and cost a gate
failure. They are retired. Weights remain in the cfg defaulted live but are
zeroable; `feature/hold-smoothness` should not be merged with them enabled.

`action_std_floor` 0.15 -> 0.05 improved velocity tracking on **all eleven**
scenarios (`drive_forward_slow` -40%, `payload_while_still` drift 0.537 -> 0.318,
better than the deployed arm C). Justified because `Policy/mean_noise_std` sat
pinned at the old floor for 86/100/100/100/99.7% of stages 1-5, and the floor was
chosen against a deadzone range EMB-18 later measured 2.5x smaller.

**`hold_pitch_ac_deg` is a discredited metric.** It halved in BOTH runs because it
is measured during training, where the policy is being injected with exploration
noise; lowering the noise floor lowers it mechanically. The deterministic
benchmark shows `station_keeping` `rms_pitch_deg` moving 0.584 -> 0.577, i.e. not
at all. Do not report it.

## 2. Deadzone sweep: threshold, not proportional

Deployed arm C policy, nothing varied but the deadzone, `station_keeping`:

| deadzone | rms_pitch | vs zero |
|---|---|---|
| 0 mA | 0.621 deg | 1.00x |
| 53.4 mA (the modelled value) | 0.589 deg | 0.95x |
| 107 mA | 1.039 deg | 1.67x |
| 160 mA | 2.624 deg | 4.22x |

Flat to ~53 mA, steep above ~100 mA. **At the deadzone the sim models, deadzone
contributes nothing.** An earlier static-equilibrium estimate (deadzone torque vs
`m*g*l`, predicting a 1.63 deg band at 53.4 mA) does NOT correspond to these
numbers and should not be used.

Caveat that turned out to matter: the policy trained at 31-78 mA, so the 107/160
mA points are out-of-distribution. Section 4 shows most of that degradation is
OOD, not mechanism.

## 3. Two firmware facts, verified by grep on `origin/feature/mdpi-actuators`

- **`DZ_{LEFT,RIGHT}_{POS,NEG} = 0.04 A` are never read.** Only hits across every
  `Core/{Src,Inc}/*.{c,h}` are the definitions and the `extern`s. The command path
  (`DDSM115setCurrent`, `DDSM115TransactCurrent`) clamps, scales `(i/8)*32767`, and
  packs. There is **no deadzone compensation at all** -- 100% uncompensated, not
  "under-corrected by a third" as CLAUDE.md said (now fixed).
- Adding compensation would create a NEW sim2real gap unless the sim's deadzone
  range is simultaneously changed to the *residual*. Because a flat `DZ_fw` meets a
  36-78 mA distribution, that residual straddles zero. The sim's
  `sign(i)*clamp(|i|-dz, 0)` already handles a negative deadzone correctly (it
  becomes over-drive), so configuration B is implementable without code changes --
  but it is two coupled knobs to keep in sync forever versus one.

## 4. Constant breakaway: right mechanism, wrong model

Training against a CONSTANT 0.031-0.160 A range held `station_keeping` rms_pitch
at **0.571 deg** while benchmarked over that same range -- against **2.624 deg**
for a policy that had never seen it. So the high-deadzone oscillation is largely
**trainable away**, and section 2's sweep was mostly measuring OOD failure.

It failed the gate on `drive_backward_slow` velocity error (0.122-0.127 vs 0.120)
on three of four candidates: applying breakaway magnitude *while rolling* is an
artifact. Real stiction is breakaway-at-rest, kinetic-while-moving.

## 5. Stribeck model (committed, default-off)

`dz(w) = dz_kinetic * (1 + (mult-1) * exp(-|w|/w_s))`, `w_s = 0.5 rad/s`.
`motor_breakaway_multiplier_range` defaults to `(1.0, 1.0)`;
`probe_motor_breakaway.py` asserts that default is **bit-identical** to the old
plant with and without `wheel_omega`, so the 2026-08-10 paper arms stay
reproducible. Both call sites pass wheel speed.

Enabled at `[2.0, 3.0]`, the fine-tune **passed the gate** (score 1.444 vs the
floor-only run's 1.366) and fixed the driving penalty outright:
`drive_backward_slow` velocity error 0.0744 vs the floor-only run's 0.1091.

## 6. Cross-benchmark: a clean specialization tradeoff

| metric (mean over hold scenarios) | floor-only | stribeck |
|---|---|---|
| **on the BREAKAWAY plant** | | |
| hold pitch | 1.905 | **1.611** |
| hold velerr | 0.0552 | **0.0431** |
| fall rate | 0.0085 | **0.0057** |
| **on the NOMINAL plant** | | |
| hold pitch | **1.612** | 1.683 |
| hold drift | **0.169** | 0.292 |
| hold velerr | **0.0358** | 0.0437 |

Each policy wins on the plant it trained against. `station_keeping` alone is more
favourable to stribeck (pitch 0.478 vs 0.577 nominal, 0.631 vs 0.974 breakaway);
the aggregate is dominated by `payload_while_still`.

**Which to deploy depends on an unmeasured hardware property.**

## 7. Breakaway explains about half the sim2real gap, not all of it

`station_keeping` rms_pitch, deployed policy:

| | |
|---|---|
| sim, nominal plant | 0.577 deg |
| sim, breakaway 2-3x | 0.974 deg |
| **hardware, measured** | **~2.0 deg** |

Breakaway at 2-3x accounts for ~1.7x of the ~3.5x gap. The remainder is
unexplained: a larger true breakaway, a larger Stribeck velocity (`w_s = 0.5
rad/s` = 0.025 m/s, while the measured limit cycle swings to 0.075 m/s, so the
term has largely decayed over most of the cycle), or cogging, which
`pure_nn_balance_env_cfg.py` explicitly does not model.

**Deliberately NOT tuned to close the gap.** `w_s` and the multiplier are both
unmeasured; fitting them until sim reproduces the hardware number would prove
nothing. The honest next step is to measure breakaway on the bench, which needs
the settle transient the rig currently discards.

## Deliverables

Both gate-passed and exported:

- `logs/.../2026-08-23_11-00-54_floor_only_stage5/exported/policy_drive.onnx`
- `logs/.../2026-08-23_12-54-06_stribeck_stage5/exported/policy_drive.onnx`

## Next

1. **Flash both and run the firmware test bench.** The rig already captures idle
   station-keeping telemetry; whichever policy shows the smaller limit cycle wins,
   and *which sim plant predicted it* tells you whether breakaway is real. This is
   the decisive experiment and the sim cannot substitute for it.
2. Fix `rew_position_far` (dead -- see CLAUDE.md). Independent of all of this.
3. Only if 1 is ambiguous: measure breakaway on the bench and set `w_s` and the
   multiplier from data rather than assumption.
