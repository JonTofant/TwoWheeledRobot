# NNDrive fixed-stance action-interface A/B — 2026-08-07

> **CORRECTED 2026-08-07, later the same day. Do not act on the comparison below.**
>
> Both arms were confounded by `com_offset_y_range_m = ±30 mm`, which
> `docs/experiments/2026-08-07-nndrive-stage1-com-and-fall-definition.md` later
> identified as the dominant cause of stage-1 falls (Cohen's d = 0.999 against
> ≤0.29 for every other randomized parameter). Neither arm could trim it, so the
> comparison measured which random flail survived a disturbance neither could
> handle. See "What this actually showed" at the end.
>
> The directional conclusion — that the leg-action interface is not the cause and
> the problem is shared — was right. The reasoning and the numbers were not.

## Question

Is the stage-1 regression caused primarily by learning balance with the current
four-CyberGear action mapping, or does it remain when the policy controls only
the wheels and the legs are held at the zero-radian hardware-safe stance?

## Controlled comparison

`scripts/run_nn_drive_action_ab.py` trained both variants from scratch for 200
iterations with seed 42 and 4096 environments. Both used the same 20
observations, rewards, stage-1 command distribution, flat terrain, domain
randomization and PPO settings. Only the action interface changed:

- `four_leg`: 4 mapped leg targets + 2 wheel currents (6 outputs)
- `fixed_stance`: four fixed 0 rad targets + 2 wheel currents (2 outputs)

The final `model_199.pt` from each arm was evaluated with 64 environments for
1000 steps on the same deterministic scenarios. Raw results are in
`outputs/nn_drive_action_ab/2026-08-07_fixed_vs_four/summary.json`.

## Results

| scenario | metric | four-leg | fixed stance |
|---|---:|---:|---:|
| station keeping | termination rate | 42.2% | 37.5% |
| station keeping | survival | 9.11 s | 9.58 s |
| station keeping | world drift | 0.557 m | 0.452 m |
| slow forward | termination rate | 40.6% | 20.3% |
| slow forward | mean velocity | -0.007 m/s | -0.002 m/s |
| slow backward | termination rate | 46.9% | 42.2% |
| slow backward | mean velocity | +0.023 m/s | -0.035 m/s |
| slow backward | mean yaw rate | +0.050 rad/s | -0.553 rad/s |
| all three | lower-is-better score | 44.74 | 34.82 |

Neither arm passes the stage-1 gate. Fixed stance improves the aggregate score
by 22%, but station-keeping termination changes by only 4.7 percentage points
and remains far above the 10% gate. It also fails to move forward on command and
develops a large unintended yaw rate while moving backward.

## Conclusion

The new leg-action mapping is not sufficient to explain the regression: the
failure survives after removing all learned leg actions. It may add learning
burden—the fixed arm's slow-forward survival is better—but the leading problem
is in the shared wheel-only balance/command-learning path. Do not spend a full
curriculum on either arm yet. This is a single paired seed; if a later diagnosis
depends on the small 4.7-point station-keeping difference, repeat with more
seeds first.

## What this actually showed

**The aggregate score was 97% termination rate.** `checkpoint_score` is
`100 × termination_rate + <everything else ≈ 0.4>`, so the "22% score
improvement" and the termination difference are one number reported twice.

**Two of three scenarios were noise.** Termination differences with 95% CIs from
evaluation sampling alone (n = 64):

| scenario | four-leg | fixed | diff | 95% CI | p |
|---|---:|---:|---:|---:|---:|
| station keeping | .422 | .375 | +.047 | ±.169 | 0.59 |
| forward slow | .406 | .203 | +.203 | ±.156 | 0.010 |
| backward slow | .469 | .422 | +.047 | ±.172 | 0.59 |

Only the forward cell moved, and that p ignores 6-way multiple comparison *and*
training-seed variance, which one seed per arm cannot estimate at all. It also
failed to replicate: re-benchmarking the same checkpoints with the reset
perturbation zeroed put fixed-stance forward at 39.1%, worse than four-leg.

**Every termination was inside the 2 s settle window** — `termination_rate`
equalled `1 - sampled_env_fraction` to the environment in 5 of 6 cells. The arms
were being scored on a reset transient, not on driving.

**"Fails to move forward" was shared, not a fixed-stance flaw** (four-leg
−0.007 m/s on a +0.10 command; fixed −0.002). The backward yaw drift was
genuinely fixed-stance-specific.

**The question was ill-posed.** `map_cybergear_tanh_to_joint_target` maps 0 → 0
for every joint, so `fixed_stance` is exactly `four_leg` with `actions[0:4]`
pinned to zero — a strict subset. The four-leg optimum is guaranteed no worse.
The informative reading is not "which arm wins" but "is the extra authority being
used", and there `Episode/cg_action_abs` rose 0.24 → 0.72 over training while
buying nothing measurable. That remains open.
