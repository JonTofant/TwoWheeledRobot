# C/D pilot after the hardware regression

The r2 C43 GRU improved simulation metrics but regressed on hardware. The pilot
keeps the corrected plant, observations and rewards, restores the shared 0.15
exploration floor, and compares fresh C (GRU) and D (wide MLP) on seed 43.
Deployment actor capacity is matched; C also has a larger recurrent critic.
This pilot is development data, not an untuned multi-seed confirmation.

Both arms use 4096 environments, a 64-step rollout, and stage budgets
300/350/200/250/150: 1250 updates per arm. Only stage 5 is shortened versus r2;
this saves 10.7% of updates, not necessarily 10.7% of total wall time. Adequacy
must be checked with frozen policies. Stage promotion always uses the final
checkpoint and does not change the fixed budgets.

After each arm completes, `select_cd_pilot_checkpoint.py` benchmarks four
predetermined candidates: the end of stage 4, first saved checkpoint at or after
25% and 50% of stage-5 updates, and stage-5 final. RSL-RL resumes at the saved
iteration label, so the final label is the stage-4 label plus 150 minus one.
The lowest existing `checkpoint_score` among candidates passing every stage-5
gate on the 11-scenario range validation benchmark is selected. Validation uses
base seed 1001, generator terrain, 64 environments and 1000 steps. Every
scenario independently resets Python/Torch RNG to base seed plus canonical
scenario index and resets recurrent state. Environment autoresets may still
diverge between policies after different termination events.

The final checkpoint and validation choice are then measured separately on
base seed 2001, under both range and nominal motor distributions. Identical
final/selected checkpoints share the exact test payload. If every candidate
fails validation, retain the failed arm and its final test results; do not
export a fallback or start seeds 42/44. A failure produces a nonzero driver
exit and a failure marker in STATUS.txt, while the other arm is still attempted.

Only the validation choice is exported. Native current-output ONNX validation
is required; the GRU is additionally expanded into the STM32AI graph and checked
for sequential equivalence over 1024 steps. This is not an on-target CubeAI or
hardware validation. No firmware is generated, flashed, committed or pushed by
the training driver. Output model hashes are recorded in deployment_selection.json.
The original stage-5 selected_checkpoint.json continues to mean fixed-budget
final, so its metrics cannot be confused with deployment selection.

Commit the source/configuration and keep the live mount unchanged throughout
training. Run inside the existing Isaac Lab container:

```bash
cd /workspace/TwoWheeledRobot
bash scripts/tools/run_cd_pilot.sh
```

The wrapper first requires a clean tracked pilot source and a successful live
preflight (`[verify] DONE failures=[]`). It records the source commit and copies
protocol.json. A unique default tag protects previous results; existing output
directories are rejected. Status and raw logs are in `logs/retrain_<tag>_<date>`;
validation, test and deployment exports are in `logs/pilot_outputs/<tag>`.

Before replication, run matched physical C/D tests with the same firmware,
motor settings and repeated protocol, log exact model hashes, and vary test
order. Keep the old and recovered GRU as separate physical references. Preserve
all r2 seeds, including C42. Freeze the revised protocol before seeds 42/44.
