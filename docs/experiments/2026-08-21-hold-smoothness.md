# Station-keeping steadiness — 2026-08-21 (NEGATIVE RESULT)

Branch `feature/hold-smoothness`, commit `77c0c8c`.
Task `Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0`, stage 5 only, 4096 envs.
Fine-tune resumed from arm C's selected stage-5 checkpoint
(`2026-08-10_23-51-29_range_gru_stage5/model_775.pt`), 150 iterations, ~23 min.

Raw evidence in `data/2026-08-21-hold-smoothness/` (the run itself lives under the
gitignored `logs/`, so the four selection benchmarks and the resolved `agent.yaml`
are copied here).

## Objective

Hardware measurement (TwoWheeledRobot_Embedded `Results/paper/range_gru_segmented.csv`,
`idle` stage): the deployed range+GRU policy station-keeps in a **0.83 Hz limit cycle**,
±2 deg pitch, ±0.1 m/s, wandering 0.52 m over a 17 s hold. Pitch and velocity peak at the
same frequency — one coupled rocking mode. Arm B does the same at 0.45 Hz, ~2.5x amplitude.

Replaying that capture through `DriveReward`'s weights charges ~0.07/step for the whole
oscillation against a 1.0/step alive bonus:

| term | charge/step | why it is blind |
|---|---|---|
| `rew_pitch` | 0.0007 | the 3 deg deadband is WIDER than the swing; 75% of samples sit in the zero-gradient region |
| `rew_delta_current` | 0.0020 | measures step-to-step change at 66.7 Hz; one 15 ms step of a 1.2 s period moves the action <1% of amplitude |
| `pitch_rate` | 0.0251 | |
| `hold_velocity` | 0.0410 | |

That diagnosis still holds and is not what failed.

## Changes tested (two at once — see "Confound")

1. Three hold-gated reward terms built on `SteadinessTracker`, an EMA residual acting as a
   first-order high-pass (tau 0.7 s, corner 0.23 Hz): `rew_hold_pitch_steady` 1.2,
   `rew_hold_vel_steady` 0.8, `rew_hold_action_ac` 0.3.
2. `agent.action_std_floor=[0.05,0.05]`, down from 0.15. Justified because
   `Policy/mean_noise_std` sat pinned at the 0.15 floor for **86% / 100% / 100% / 100% /
   99.7%** of stages 1-5 of the original arm — PPO had wanted lower noise since ~iteration 40
   of stage 1 — and because the floor was chosen against a 0.03-0.20 A deadzone that EMB-18
   has since measured at 0.031-0.078 A, 2.5x smaller.

## Result: stage-5 gate failed on all four shortlisted checkpoints

```
model_850.pt: drive_backward_slow rms_vel_err 0.137 > 0.120; payload_while_still drift 0.841 > 0.500
model_875.pt: payload_while_still drift 0.672 > 0.500
model_900.pt: payload_while_still drift 0.872 > 0.500
model_924.pt: station_keeping 0.608 | drive_backward_slow 0.122 | push_while_still 0.808 | payload_while_still 1.105
```

No checkpoint was promoted and no ONNX was exported.

### The training metrics said it was working

| metric (150 iters) | first 5 | last 10 |
|---|---|---|
| `mean_noise_std` | 0.129 | 0.050 |
| `hold_pitch_ac_deg` | 2.18 | **1.01** |
| `hold_velocity_abs` | 0.152 | 0.061 |
| `hold_action_ac_abs` | 0.156 | 0.061 |
| `fall_rate` | 0.0000 | 0.0001 |
| `hold_pos_err_abs` | 0.083 | **0.136** ← the only one that moved the wrong way |

Frozen-checkpoint benchmark, same policy, disagrees: `rms_pitch_deg` on `station_keeping`
went 0.584 -> 0.634 (worse), and `world_drift_m` 0.059 -> 0.105 (worse).

**`hold_pitch_ac_deg` is a bad success metric and should not be trusted alone.** It measures
the AC residual only, so a policy that stops correcting and drifts away in a straight line
scores *better* than one that actively holds station by rocking. It was added in the same
commit as the terms it was meant to evaluate, which is exactly how that goes wrong. This is
the 2026-07-28 lesson (`docs/experiments/2026-07-28-nndrive-station-keeping.md`) repeating:
benchmark a frozen checkpoint, and read `world_drift_m` before anything else.

## Why the reward terms are wrong

On a two-wheeled balancer, rejecting a push **requires moving** — lean, drive under the
disturbance, return. That correction *is* an oscillation in pitch and velocity. Penalising
the AC component of both during hold taxes the recovery mechanism itself, and the cheapest
way to lower AC pitch is not to balance better but to stop correcting and drift.

The failures are concentrated in exactly the disturbed hold scenarios (`payload_while_still`,
`push_while_still`) where that mechanism is load-bearing.

### Attribution: the damage is hold-gated, the benefit is not

The three reward terms are the only hold-gated change; the noise floor applies everywhere.
Comparing `model_875` against arm C's `model_775` over all 11 scenarios:

| | mean `world_drift_m` change |
|---|---|
| hold-gated scenarios (terms active) | **+0.088 m (worse)** |
| driving scenarios (terms inactive) | **-0.039 m (better)** |

Degradation is also monotonic in training: `model_875` least bad, `model_924` (newest) worst.
More exposure to the new reward, more drift.

### The noise-floor half looks beneficial

Velocity tracking improved almost everywhere, which the hold-gated terms cannot explain:

| scenario | arm C | model_875 |
|---|---|---|
| `drive_forward_slow` | 0.1179 | **0.0746** (-37%) |
| `turn_in_place` | 0.0607 | **0.0487** |
| `drive_and_turn` | 0.0859 | **0.0719** |
| `push_while_driving` | 0.1127 | **0.0946** |

`agent.yaml` in the data dir confirms the floor resolved to `[0.05, 0.05]` — configured
AND applied.

## Confound

Both changes were made in one run, deliberately, to maximise the chance of a flashable
policy before a 14-day break. The attribution above is inferential (hold-gated vs not,
monotonicity, which metrics moved) rather than measured. It is strong enough to justify
the next experiment but is not a clean ablation.

## Next experiments, in priority order

1. **Noise floor alone.** Set `rew_hold_pitch_steady = rew_hold_vel_steady =
   rew_hold_action_ac = 0.0` and keep `action_std_floor=[0.05,0.05]`. The reward is then
   byte-identical to arm C's, so a pass is directly comparable and flashable, and it tests
   the half the evidence supports. ~40 min. **Do this first.**
2. **Redesign the steadiness terms so they cannot pay for drifting.** The flaw is that AC
   pitch and position error are independent, so the policy can buy one with the other.
   Options, roughly in order of appeal:
   - Gate the steadiness terms on *low position error* as well as centred joystick, so
     stillness only pays once the robot is actually home. A drifting policy earns nothing.
   - Multiply the steadiness bonus by the existing position bonus rather than adding it,
     making them jointly necessary.
   - Apply the AC penalty to pitch only *after* the disturbance has settled (gate on
     recent disturbance-free time), so push recovery is exempt by construction.
3. Only after 1 and 2 land: re-check whether the 0.83 Hz cycle actually survives a
   lowered noise floor on its own. It may be substantially an artefact of training under
   ±0.3 A of exploration noise the deployed policy no longer has, in which case no
   reward change is needed at all.

## Status

Branch `feature/hold-smoothness` holds the terms and the tracker, unmerged and not
recommended for merge as-is. `probe_steadiness_tracker.py` passes and the filter itself is
correct — the high-pass does what it claims (DC out at 0.00e+00, 0.83 Hz through at 0.975).
The mechanism is sound; what it is attached to is not.
