# NNDrive stage-1 regression: COM range, fall definition, reward shape — 2026-08-07

## Summary

Stage 1 was failing its gate at 42-47% termination. One well-evidenced cause, one
measurement error in the fall criterion, and one round of reward changes that was
made at the same time and later withdrawn.

**What survives:**

1. `com_offset_y_range_m = ±30 mm` demanded ±13.3° of permanent trim lean. This
   was the dominant cause (§2) and is now ±5 mm on the six-action task, ±1 mm on
   the fixed-stance task.
2. The tilt fall rule measures *lean*, not falling: every termination it reported
   was a recovery swing overshooting 25° (§3). Contact-based termination is
   implemented and verified, but is **not** the default — it is coupled to the
   reward shape (§5).
3. Spawn pitch now ramps per curriculum stage; the recoverable envelope is ~9°
   and stage 1 was spawning at ±12° (§4).
4. `cg_rate` was an unbounded penalty reaching −156/step (§5).

**What was withdrawn:** the non-negative reward refactor. The property it
guaranteed held, but it rescaled per-step reward ~9× without retuning PPO, which
destabilised the adaptive learning-rate schedule and made outcomes non-monotone
in training time. Its one durable idea — the attitude deadband — was kept. See §5.

| condition | station | forward | backward | rms_pitch |
|---|---:|---:|---:|---:|
| ±30 mm COM, tilt rule (baseline) | 42.2% | 40.6% | 46.9% | 5.9° |
| ±8 mm COM, tilt rule | 20.3% | 17.2% | 14.1% | ~2.0° |
| ±5 mm COM, contact rule, bonus reward | 14.1% | 7.8% | 12.5% | 4.2-5.2° |
| + 9° spawn, roll/tracking reweight (capped penalties) | 59.4% | 51.6% | 57.8% | 21-24° |
| + penalties uncapped | 1.6% | 1.6% | 4.7% | 1.6-3.4° |

Six-action task, seed 42, 4096 envs, 200 iterations, stage 1, flat terrain. The
last row passed the gate, but on a single seed and under the reward that was
subsequently withdrawn — see §5 for why single-seed results from this sequence do
not generalise.

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
`0.597·d`. Trim sensitivity is 0.758° of permanent lean per mm of whole-robot
offset, from a COM height above the wheel axis measured directly at 75.63 mm by
`scripts/measure_nominal_com.py`. ±30 mm therefore demanded up to **±13.3° of
permanent lean** against a 25° fall threshold.

(An earlier estimate of ~93 mm, backed out from observed trim rather than
measured, was 25% high and understated every trim figure derived from it. The
corrected sensitivity gives ±2.3° for the ±5 mm six-action range and ±0.45° for
the ±1 mm fixed-stance range.)

The same measurement settles a separate question: the nominal whole-robot COM
sits **+0.29 mm** fore/aft and **−0.03 mm** laterally of the wheel axis, a 0.216°
resting lean. The asset is symmetric on both axes, so any forward/backward or
left/right asymmetry observed in a policy is the policy, not the plant.

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

## 5. Reward: non-negative per step — TRIED AND WITHDRAWN

The old design bounded each penalty below the ~2.3/step a fall forfeits
(`rew_alive + rew_vel_track + rew_yaw_rate_track`). That argument had already
failed once (2026-07-28: unbounded wrapped-yaw penalty at 4.9/step, 98%
turn-in-place falls) and had to be re-derived whenever a weight moved. So every
goal term was rewritten as a bounded non-negative bonus (`tent_bonus`,
`flat_top_bonus`), leaving only actuation costs negative and bounded below
`rew_alive`. With all rewards ≥ 0 and γ < 1 a longer episode weakly dominates a
shorter one, so diving cannot pay — by shape rather than calibration.

**The property held. It was withdrawn anyway, because it destabilised PPO.**

Rewriting the terms multiplied per-step reward by ~9 (1.3 → 11.6),
`Train/mean_reward` by ~15×, and `Loss/value_function` by ~80× (550 → 7,000–24,000),
while `value_loss_coef = 1.0` and `desired_kl = 0.01` stayed at values tuned for
the old scale. The adaptive-KL schedule then stopped converging. Learning rate
across the last 600-iteration runs, sampled every 60 iterations:

```
seed 42  0.0011  0.0100  0.0000  0.0100  0.0006  0.0100  0.0100  0.0100  0.0100  0.0100
seed 43  0.0017  0.0100  0.0100  0.0000  0.0013  0.0009  0.0100  0.0100  0.0100  0.0100
seed 44  0.0044  0.0000  0.0100  0.0002  0.0000  0.0000  0.0044  0.0100  0.0100  0.0100
```

It oscillates across the full 1e-5…1e-2 range and sits at the ceiling late in
training. Under the restored weights the same schedule decays smoothly
(0.0057 → 0.0020 → 0.0006).

Consequences, all of which were initially misread as task properties:

- **Outcomes are non-monotone in training time.** 2 of 3 seeds passed at 200
  iterations; 1 of 3 at 600. Seed 43 collapsed in its final 60 iterations
  (`mean_episode_length` 1311 → 742). An earlier reading of "200 iterations is
  too few", based on seed 44 alone improving from 31% to 0% termination, did not
  generalise.
- **Per-step reward stopped discriminating.** `reward_total` was 11.36–11.67
  across all three seeds despite a 20× spread in fall rate. Only accumulated
  survival separated them, and `rew_alive` is 8.6% of per-step reward.
- **Two saturation bugs were introduced by bounding penalties at their output.**
  `cg_rate` pinned at −0.0977 of a −0.10 cap for a whole run; `cg_pos` then
  pinned at −0.1966 of a −0.20 cap, which stopped centring the legs, let them
  park at `cg_action_abs` 0.67, and put the robot into a 15–24° lean via the
  resulting COM shift.

**Kept from the attempt**, each independently evidenced:

- The **pitch/roll deadband**, which was the actual insight and survives in the
  restored reward as `−w·(max(0, |θ| − flat))²`. The reward reads true pitch while
  the policy observes a copy carrying a ±3° mounting bias, so an exact θ = 0 is a
  sim-only skill.
- **`vel_track_sigma` 0.25 → 0.08.** The exp kernel is the small-error
  discriminator; at 0.25 the gap between tracking a +0.10 m/s command and standing
  still was 3.4% of per-step reward, and a constant forward bias was cheaper than
  tracking.
- **The `cg_rate` delta clamp**, a genuine pre-existing unbounded penalty
  (−156/step reachable, logged as −0.006).

**Also withdrawn: contact-based termination as the default.** It is implemented,
verified and better-evidenced than the tilt rule (§3), but it is *coupled* to the
reward: with penalty-based shaping, a robot that can sit past 25° indefinitely
accrues sustained negative reward and diving becomes optimal again. The tilt rule
bounds the attitude penalties by ending those episodes. Re-enabling contact
termination requires re-bounding the attitude penalties in the same change.
`fall_mode` retains both paths; the default is `"tilt"`.

**Method note.** The two changes were made together and each was then tuned
against symptoms the other caused, which cost several training runs. The COM fix
(§2) was the only well-evidenced change in the batch, and running it alone as a
control was the experiment that should have come first.

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
