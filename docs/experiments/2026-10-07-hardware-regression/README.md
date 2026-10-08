# Hardware regression investigation — 7 October 2026

Status: completed on 8 October 2026. The observation-timing correction passed 33 live simulator checks. Both bounded 100-update adaptations finished validation and independent testing; neither qualified for deployment. Physical recovery has not been demonstrated. See `RESULTS.md` for the full results and `PAPER_PATH.md` for the manuscript route.

## Finding and scope

The fresh C pilot's reported hardware failure is genuine: one unforced standing attempt ended 5.336 s after rearm with zero commanded speed/yaw, before the automatic testbench. Network replay matches its MCU currents to 7.2e-7 A. The intended policy was deployed and its recurrent state/output scaling behaved as expected in that capture. There is no evidence here of an inference deadline miss. These checks do not prove sensor calibration or motor torque fidelity.

A remaining training/deployment timing mismatch was found in `NNDriveEnv._get_observations`: after changing previous-current observations to the policy's own command, the environment still delayed the entire completed observation vector. For environments with one tick of observation delay, the previous-current feature therefore represented an additional tick of history. Firmware uses the immediate preceding logical network command regardless of sensor age. Velocity/yaw commands and integrated references were also delayed with the full vector.

The new default `obs_delay_mode=sensors` delays raw noisy sensor measurements first, then constructs errors using current references and supplies current local commands/previous outputs. The first observation after reset initializes sensor history from a valid measurement instead of an all-zero vector. Partial resets clear only the affected histories. `legacy_full` remains available explicitly for historical evaluation; reproducing August observations additionally requires its historical previous-current source and plant/config. Existing saved results and models are not rewritten.

All earlier structural corrections remain: logical mirrored-wheel torque limiting per physics substep, per-episode contact randomization, command-valued previous current, and correct recurrent resets in evaluation. Rewards, motor gain/deadzone, damping, stance gains, current limits and firmware are unchanged by this fix.

## Frozen evidence before adaptation

The diagnostic screen used three frozen GRUs, 12 timing/damping conditions, 32 robots per condition/policy and 15 s of zero-command standing on flat terrain. This is 1,152 simulated robot trials. Cohorts use independent random initial draws, not matched per-robot pairs. No invalid-state or physics-broken events occurred. Full per-condition falls, survival, coverage, drift, velocity, currents and current changes are in `latency_sweep.json`.

| One action-delay tick + one observation-delay tick | Original GRU | Existing recovered GRU | Fresh C pilot |
|---|---:|---:|---:|
| Full-vector delay: falls | 34.4% | 0% | 0% |
| Fresh local features: falls | 0% | 0% | 0% |
| Full-vector delay: mean absolute command change | 0.349 A/tick | 0.054 A/tick | 0.152 A/tick |
| Fresh local features: mean absolute command change | 0.177 A/tick | 0.089 A/tick | 0.490 A/tick |
| Full-vector delay: velocity RMS | 0.199 m/s | 0.082 m/s | 0.053 m/s |
| Fresh local features: velocity RMS | 0.213 m/s | 0.121 m/s | 0.123 m/s |

The fast pilot current changes resemble a feature of the actual hardware trace, while the simulator still does not reproduce its fall. The screen's "fresh local features" condition refreshed odometry/reference-derived features as well as commands/history; it was a diagnostic approximation, not the final raw-sensor implementation. This evidence supports timing sensitivity as a contributor, not a complete causal explanation or a physical ranking. The original hardware-proven model can fare worse in these corrected simulations, illustrating the remaining transfer gap.

Frozen replay across the pilot's five stages shows the sharp current response already in stage 1. The median direct previous-action Jacobian spectral radius on sampled hardware inputs is about 0.91 for the selected pilot versus 0.16 for the old policy and 0.17 for the recovered policy. This is a local input sensitivity, not a closed-loop stability certificate. Stage-5 truncation alone cannot explain a behavior present in stage 1. Alternative-model replay is open-loop and cannot establish their counterfactual physical trajectories.

## What is not established

- The exact physical cause of the fall is not uniquely identified by one capture.
- The old-to-new damping/friction changes altered the plant and learning problem, but the tests do not isolate either as the cause. Reverting the wheel-sign bug is not justified.
- The exploration floor was already restored to 0.15 in the fresh pilot. It is training noise, not a firmware current multiplier.
- At low speed, current feedback best matches commands two ticks earlier, with approximate slopes 0.75/0.82 in the failed capture. Feedback age and calibration are included; this is not an identified loaded torque transfer function. The simulator's 0/1 action plus 0/1 observation delays do not establish that physical latency has been measured or faithfully covered.
- The 0.0007 damping number derives from free-running sweeps and motor assumptions. Loaded damping was not directly measured.

## Recovery experiment

The frozen protocol is `recovery_protocol.json`: original hardware-proven checkpoint 775 (seed-42 ancestry), 100 stage-5 updates with adaptation RNG seed 43, 4096 environments, 64-step rollout, floor 0.15, fresh optimizer and fixed learning rate 5e-5. Only the final checkpoint is eligible. The new timing semantics are the environment correction. Rewards and remaining plant parameters are held fixed. This is development adaptation, not a fresh seed-43 replication and not an architecture ablation.

Validation uses base seed 4001 and independent testing base seed 5001, all 11 canonical scenarios, generator terrain, 64 robots per policy, 1000 steps. The original, previous recovery, failed pilot and new final checkpoint share each scenario/configuration, in independent randomized cohorts. Terrain origins are explicitly assigned so every policy sees the same canonical 64-environment terrain layout; randomized starts/motor draws remain independent cohorts. An initial harness version gave contiguous groups different terrain-column distributions. Its outputs are preserved under `invalid_unbalanced_terrain` and excluded from qualification/comparison. No policy was changed in that repair. Existing stage-5 gates remain unchanged. Both validation and independent test must pass before preparing a hardware candidate; failed outputs are retained without reselection. No hardware advantage is inferred from these tests.

## Evidence locations

- `logs/hardware_c_pilot_2026-10-07/ANALYSIS.md`: original trace/firmware audit, source firmware commit `44eb23a28f71bf93b3f37f040d4d07454c91495a`.
- `logs/regression_diagnosis_2026-10-07/`: raw frozen-sensitivity outputs, current-response fits, checkpoint-feedback replay, live preflight, training/evaluation logs, scripts, source snapshot and exact protocol.
- `PAPER_PATH.md`: findings mapped to the supplied review, proposed discussion wording and minimal additional hardware data.

No robot was flashed or actuated. All r2 results including C42 and the original rollback graph remain unchanged.

## Second bounded attempt — 8 October

The first corrected-timing adaptation from the original GRU remains upright in the completed balanced-terrain validation scenarios but misses slow-speed tracking gates. A second, final bounded development attempt starts from the failed pilot checkpoint 1150 instead, retaining the same 100-update budget, fixed 5e-5 learning rate, optimizer reset, floor, seed, rewards and corrected sensor contract. This tests whether the pilot's stronger simulated tracking can be preserved while reducing its fast-current response. Its frozen protocol and process records are in `logs/regression_pilot_adaptation_2026-10-08/`.

The second final checkpoint is 1249. Validation and independent test use new base seeds 6001/7001. In addition to all existing gates, deployment preparation requires standing mean absolute command change to be at least 25% below the parent pilot on both seeds. That screen was declared before evaluating the adapted model. It is an engineering symptom check, not a hardware-stability guarantee. No intermediate checkpoint selection, threshold relaxation or third training attempt is planned in this investigation. Both attempts remain recorded regardless of outcome.

## Final decision — 8 October

Both adapted models completed 704 initial episodes on each of their validation and test splits with zero falls, 15 s survival and all 867 post-settling scored samples per episode. These are simulator results; the failed fresh parent also passed its simulator checks before falling on hardware.

- **Original-weight adaptation, final 874:** failed three validation tracking gates. Independent testing still failed slow reverse tracking (0.160 m/s RMS error against a 0.120 threshold). It is not a qualified replacement.
- **Pilot-weight adaptation, final 1249:** passed all existing stage-5 gates on both splits, but failed the predeclared current-change screen. Standing command change was 0.2346 vs parent 0.2268 A/tick in validation (+3.4%), and 0.1987 vs 0.2222 A/tick in independent testing (-10.6%). The required reduction was at least 25% on both splits. It is not a qualified replacement.

No new candidate was exported, built, packaged, pushed to the embedded branch or flashed. Existing working and rollback policies remain untouched. No third training attempt or checkpoint reselection was launched. Checkpoint/source hashes, resolved configs, exact 100-update counts and complete benchmark markers were verified. Export/target checks are not applicable to these rejected finals.

The correction fixes a real timing contract error, but these bounded adaptations did not establish a useful physical recovery. This does not prove that corrected training can never work: the tested adaptation budget is limited. It does mean that another large retrain is not justified by these results alone. The remaining investigation needs physical actuator/sensor timing and loaded motor-response evidence; current-feedback lag in this capture cannot identify torque onset by itself.

The immediate paper route is to retain the archived original hardware implementation and its actual measurements, disclose the mismatches, narrow the claim to demonstrated embedded feasibility, and report the failed corrected pilot and recovery attempts separately. The old controller's better observed hardware behavior remains a result, not proof that reintroducing a software bug is the correct design.
