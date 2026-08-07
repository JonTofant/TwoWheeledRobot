# NNDrive stage-1 regression: COM range, fall definition, reward shape — 2026-08-07

## Summary

Stage 1 was failing its gate at 42-47% termination. Three separate causes, found
in this order:

1. `com_offset_y_range_m = ±30 mm` demanded ~±11° of permanent trim lean. Dominant.
2. The fall rule was measuring *lean*, not falling. Every termination it reported
   was a recovery swing overshooting 25°, not a robot on the floor.
3. Two per-step penalties were unbounded or gradient-dead, one of them pre-existing.

After all three: **stage-1 gate passes**, 1.6-4.7% termination.

| condition | station | forward | backward | rms_pitch |
|---|---:|---:|---:|---:|
| ±30 mm COM, tilt rule (baseline) | 42.2% | 40.6% | 46.9% | 5.9° |
| ±8 mm COM, tilt rule | 20.3% | 17.2% | 14.1% | ~2.0° |
| ±5 mm COM, contact rule, bonus reward | 14.1% | 7.8% | 12.5% | 4.2-5.2° |
| + 9° spawn, roll/tracking reweight (capped penalties) | 59.4% | 51.6% | 57.8% | 21-24° |
| + penalties uncapped | **1.6%** | **1.6%** | **4.7%** | 1.6-3.4° |

All runs: four-leg, seed 42, 4096 envs, 200 iterations, stage 1, flat terrain.

## 1. Every termination was inside the settle window

`termination_rate` equalled `1 - sampled_env_fraction` to the environment in 5 of
6 baseline cells. `sampled_env_fraction` counts environments alive past
`settle_steps = (cmd_settle_s + 1.0)/dt` = 2.0 s, so *all* deaths happened in the
first two seconds and essentially none across the remaining 13 s. Confirmed
independently by training: `mean_episode_length` 778 of 1333 steps ≈
0.58 × 1333 + 0.42 × ~50.

This is only visible because `fall |= ...` sits outside the `if step >= settle_steps`
guard in `benchmark_nn_drive.py`, while every other metric is inside it. The kill
counter includes the window the averages are designed to skip.

## 2. COM offset was the dominant cause

`scripts/diagnose_early_death.py` scores every per-episode randomized parameter
for separation between environments that die early and those that survive.

| parameter | died<2s | survived | Cohen's d | AUC |
|---|---:|---:|---:|---:|
| `abs_com_offset_y_m` | 0.0197 | 0.0118 | **0.999** | **0.758** |
| `com_offset_y_m` (signed) | 0.0029 | −0.0022 | 0.285 | 0.585 |
| `abs_spawn_pitch_deg` | 6.40 | 5.69 | 0.199 | 0.558 |
| 23 others | | | ≤0.17 | ≤0.55 |

Magnitude separates, sign does not — the signature of a balance disturbance that
is bad in either direction. Dose-response was monotonic across 8 deciles:
13.3% → 18.8% → 25.8% → 38.3% → 47.7% → 57.8% → 65.6% → **78.9%**.

Causal confirmation, same checkpoint, no retraining:

| eval condition | station | forward | backward |
|---|---:|---:|---:|
| baseline | 42.2% | 40.6% | 46.9% |
| reset perturbation zeroed | 32.8% | 29.7% | 25.0% |
| COM offset zeroed | 15.6% | 12.5% | 10.9% |
| both zeroed | 0% | 0% | 0% |

**Why ±30 mm was so severe.** `Platform_Group` is 2.269 kg of the robot's
3.802 kg (59.7%), so a platform COM shift `d` moves the whole-robot COM by
`0.597·d`. Measured trim sensitivity is ~0.6° of permanent lean per mm of
whole-robot offset (effective pendulum height ~93 mm, backed out from observed
trim). ±30 mm therefore demanded up to **±11° of permanent lean** against a 25°
fall threshold.

**Why it appeared now.** Per `nn_drive_env_cfg.py`, this was
`com_offset_x_range_m` until 2026-08-05 and was applied to the *lateral* axis.
The fore/aft trim randomization had never actually trained. Fixing the axis is
what surfaced the magnitude.

## 3. The fall rule was not measuring falling

With COM at ±8 mm, re-benchmarking the same checkpoint with
`fall_pitch_threshold_deg=80` (config only, no code change):

| scenario | 25° rule | 80° rule | survival |
|---|---:|---:|---:|
| station | 20.3% | **0%** | 15.0 s |
| forward | 17.2% | **0%** | 15.0 s |
| backward | 14.1% | **0%** | 15.0 s |

Not one robot fell. Episodes spawn at pitch ~U(±12°) with rate ~U(±0.5 rad/s), and
a legitimate recovery swing overshoots 25° easily; the rule fired after 5
consecutive steps (75 ms) of that.

Replaced with contact-based termination: any body other than the two wheels
touching the ground (`fall_mode = "contact"`). Attitude is no longer a termination
criterion at all — a leaning robot is not a fallen robot, and with a ±3° IMU
mounting bias there is no attitude the policy can be held to.

Requires `activate_contact_sensors=True` in `robot_cfg.py`. With it False a
ContactSensor still constructs and reports — zeros, forever.

## 4. Spawn pitch envelope

After the COM fix, the separation diagnostic re-run on the contact-terminating
policy: `abs_spawn_pitch_deg` d = **2.307**, AUC = **0.956**, with
`abs_com_offset_y_m` collapsed to d = 0.235. Dose-response is a cliff:

| spawn \|pitch\| | 0-7.6° | 7.6-9.2° | 9.2-10.6° | 10.7-12.0° |
|---|---:|---:|---:|---:|
| fall rate | **0.0%** | 3.9% | 13.3% | **53.9%** |

640 of 1024 environments spawned below 7.6° and none fell. The recoverable
envelope is ~9°; the spawn was ±12°. `reset_ranges()` ignored `curriculum_stage`
entirely, so stage 1 got the full range on iteration 1 while command velocity,
yaw rate and disturbance kind all ramp. Now scaled per stage:
9.0 / 10.0 / 11.0 / 12.0 / 12.0°.

## 5. Reward: non-negative per step

The old design bounded each penalty below the ~2.3/step a fall forfeits
(`rew_alive + rew_vel_track + rew_yaw_rate_track`). That argument had already
failed once (2026-07-28: unbounded wrapped-yaw penalty at 4.9/step, 98%
turn-in-place falls) and had to be re-derived whenever a weight moved.

Now every goal term is a bounded non-negative bonus (`tent_bonus`,
`flat_top_bonus`) and only actuation costs subtract, worst case 0.706 below
`rew_alive`. With all rewards ≥ 0 and γ < 1 a longer episode weakly dominates, so
diving cannot pay — by shape, not calibration.

Attitude is a *band*: full weight inside ±3° with zero gradient, decaying to ~0 by
15°. The reward reads true pitch while the policy observes a copy carrying a
±3° per-episode mounting bias, so demanding an exact angle inside that band asks
the policy to resolve what its sensor cannot, and trains a precision the hardware
IMU can never deliver.

### Two penalty bugs, both found by the ≥ 0 assertion

**`cg_rate` was unbounded (pre-existing).** It is computed on `target_angle`,
which is *not* slew-limited — `cg_target_slew_radps` limits `applied_target`, one
stage later. One action reversal can swing all four targets across their full
range: Σ(Δθ)² ≈ 39 rad², i.e. **−156/step**. A trained policy never jumps that
far, so it logged as −0.006 and never surfaced. Random-action probing hit
−32/step immediately.

**Capping a penalty's output is always wrong.** Both caps tried here saturated and
went gradient-dead while reading as a harmless constant in TensorBoard:

- `cg_rate` pinned at −0.0977 of its −0.10 cap for an entire run, so it stopped
  discouraging flapping at all.
- `cg_pos` then pinned at −0.1966 of its −0.20 cap. Nothing restored the legs to
  centre, they parked at `cg_action_abs` 0.67, and the resulting COM shift put the
  robot into a 15-24° lean and a −0.16 m/s drift regardless of command. This is
  the 59% row in the summary table.

Bounds now go on the *input* (`cg_rate_delta_clamp_rad`, `delta_current_clamp_a`)
or into the weight (`rew_cg_pos` 0.0988 → 0.05, uncapped).

## Verification

`scripts/verify_contact_and_reward.py` asserts, in one short run:

1. wheels report non-zero contact force (catches `activate_contact_sensors=False`)
2. the spawn does not self-terminate
3. the per-stage spawn-pitch ramp actually reached the reset draw
4. per-step reward ≥ 0 under random actions driving to 55° tilt
5. the reward ceiling is not truncating bonuses
6. **no reward term has a dead gradient** — the check that would have caught both
   penalty bugs before a 25-minute run

## Open

- `rms_vel_err` 0.101 against a 0.12 limit on the forward scenario, with
  `mean_velocity` +0.002 on a +0.10 command. Velocity tracking is the weak point;
  it passes on a technicality.
- The stage gate's `termination_rate ≤ 0.10` and `rms_pitch_deg ≤ 8.0` were
  written when termination meant a 25° tilt overshoot. They are not like-for-like
  with contact-based falling and should be re-derived.
- Single seed throughout. Nothing here supports a claim finer than the large
  effects reported.
