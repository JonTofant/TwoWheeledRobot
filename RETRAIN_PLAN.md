# Retrain plan: paper arms "r2" (weekend of 2026-10-03)

> **2026-10-07/08 regression investigation:** the fresh C pilot fell during
> unforced hardware standing despite passing its simulation selection.
> A further mismatch was found: full-vector observation delay also delayed the
> locally known previous command. NNDrive now defaults to `obs_delay_mode=sensors`;
> `legacy_full` explicitly preserves the earlier observation pipeline. The r2
> and original pilot results remain historical artifacts of their saved source,
> not results under this new default. Do not launch another multi-seed retrain
> from this historical recipe before evaluating the bounded recovery and hardware
> transfer. See [the investigation](docs/experiments/2026-10-07-hardware-regression/README.md)
> and [the publication path](docs/experiments/2026-10-07-hardware-regression/PAPER_PATH.md).

> **Pre-pilot audit completed (2026-10-06):** 52 frozen-checkpoint checks found
> a better C42 policy at the end of stage 4; later stage-5 updates first lost
> velocity tracking and then introduced falls. No retraining was launched.
> See [the audit](logs/prepilot_audit_2026-10-06/PREPILOT_REVIEW.md) and
> [the proposed pilot protocol](logs/prepilot_audit_2026-10-06/pilot_protocol.json).
> The revised pilot covers C/D only, seed 43, with stage budgets
> 300/350/200/250/150 (1250 updates) and the same held-out validation selection
> for both arms. Only stage 5 is shortened; the 64-step rollout stays unchanged.
> `scripts/tools/run_cd_pilot.sh` now runs preflight and the fixed-budget
> curriculum, then applies the shared validation selection, separate test and
> validated export. Preserve all r2 seeds.

> **2026-10-06 hardware update:** C43 was reported to fall under small pushes,
> track poorly, and command insufficient current on the real robot. The r2
> simulation improvement did not establish improved hardware transfer. The
> fixed-stance exploration floor is now restored to the August value 0.15;
> structural fixes remain in place. `run_retrain_arms.sh` now defaults to tag
> `r3` so new evaluations cannot overwrite the completed r2 data. The r2
> procedure and values below describe the historical run, not today's defaults.
> First develop a bounded warm-start adaptation of the hardware-proven August
> GRU, rather than repeat all four arms. Restoring the floor is a hypothesis,
> not a confirmed diagnosis; hardware A/B is still required.

Branch **`feature/mdpi-retrain`**. The code is ready and committed (`4ec192e`, `d2e3992`).
It was smoke-tested end to end (all 4 arm types: train → export → config check → benchmark)
and passed 23 live-sim checks. A full run was started and then **stopped on purpose** on
2026-10-01 after 26 iterations. Learning looked normal and the iteration speed matched August
(~5.5 s).

## 1. Before you start (5 min)

```bash
cd ~/Projects/TwoWheeledRobot
git checkout feature/mdpi-retrain
git status                      # must be clean (AGENTS.md untracked is fine)
git log --oneline -1            # d2e3992, or a later commit you made on purpose
docker start isaac-lab-dev
nvidia-smi                      # nothing big on the GPU (4096 envs needs ~8 GB)

# Pre-flight: live-sim check that every fix is active (~2 min, no training)
docker exec isaac-lab-dev bash -c "cd /workspace/TwoWheeledRobot && \
  /isaac-sim/python.sh scripts/tools/verify_retrain_changes.py --headless" | grep verify
# expect the last line:  [verify] DONE failures=[]
```

If you changed code after `d2e3992`, rerun the pre-flight. Each run records the commit hash
and any dirty files, but a clean commit is the provenance the reviewer asked for.

## 2. Launch

Full run: 4 arms × seeds 42/43/44. That is ~2.5–3 h per arm, **~30–36 h total**, run seed-major,
so seed 42 (the arms that go on the robot) finishes first, after ~10–12 h.

```bash
docker exec -d isaac-lab-dev bash -c 'cd /workspace/TwoWheeledRobot && \
  bash scripts/tools/run_retrain_arms.sh 42 43 44 > logs/retrain_r2_driver.log 2>&1'
```

Short on time? Run `... run_retrain_arms.sh 42` first, then `43 44` later. Only seed 42 is
needed for the hardware runs. A subset of arms: prefix the command with `RETRAIN_ARMS="C D"`.

**Do not edit source files while it runs.** Every curriculum stage starts a fresh process
that imports the code, so a broken edit kills all remaining stages.

## 3. Watch it

```bash
cat logs/retrain_r2_*/STATUS.txt                         # one line per arm/bench; FAIL lines are loud
grep -E "Learning iteration|Mean reward|Mean episode length|noise std" \
     logs/retrain_r2_*/r2_point_s42_train.log | tail -8  # live stats of the first arm
```

Sanity check after ~10 min, at stage 1 iteration ~100 of the first arm. The August reference
for the same point (arm A, stage 1) was: episode length 1333 (full 20 s), fall rate 0,
reward around +1000, and noise std down at its floor. That floor is now **0.05** (it was 0.15).
Reward values are not directly comparable, because some reward terms changed. **Episode
length reaching 1333 is the key sign.** If episodes are still short at iteration 150, stop the
run (`docker exec isaac-lab-dev pkill -f run_retrain_arms; docker exec isaac-lab-dev pkill -f train_nn_drive_curriculum; docker exec isaac-lab-dev pkill -f rsl_rl/train.py`)
and investigate.

After each arm, the driver re-reads the resolved `params/env.yaml` and `params/agent.yaml`
and fails loudly if any fix is missing (`CONFIG MISMATCH` in STATUS.txt).

### If something fails
- A Kit crash at benchmark start is retried 3× automatically. A crash during training is not
  retried.
- To re-run just one arm: `RETRAIN_ARMS="C" bash scripts/tools/run_retrain_arms.sh 43`.
  It creates new run dirs, and the driver always picks the newest.
- Leftovers from the stopped run can be deleted. They are root-owned, so use the container:
  `docker exec isaac-lab-dev bash -c 'cd /workspace/TwoWheeledRobot && rm -rf logs/rsl_rl/nn_drive_fixed_stance/2026-10-01_09-20-18_r2_point_s42_stage1 logs/rsl_rl/nn_drive_fixed_stance/*smoke* logs/retrain_r2_2026-10-01 logs/retrain_smoke* logs/retrain_r2_driver.log'`

## 4. Outputs

| What | Where |
|---|---|
| Runs (5 stage dirs per arm) | `logs/rsl_rl/nn_drive_fixed_stance/<date>_r2_{point,range,range_gru,range_wide}_s<seed>_stage{1..5}` |
| Deployable ONNX | `<stage5 dir>/exported/policy_drive.onnx` (validated vs PyTorch during export) |
| Benchmark JSONs | `docs/paper/figures/data_r2/arm_{A,B,C,D}_under_{range,nominal}_s<seed>.json` (August data stays untouched in `data/`) |
| Driver log / status | `logs/retrain_r2_<date>/` |

## 5. After training

1. **Archive** seed 42 (and the other seeds if they're going in the paper), one call per arm:
   `python scripts/archive_run_for_paper.py --dest fixedstance_r2_range_gru_s42 logs/rsl_rl/nn_drive_fixed_stance/*_r2_range_gru_s42_stage*`
2. **Figures and table.** `scripts/make_paper_figures.py` still needs updating, which Claude can
   do once the data exists:
   - arm D;
   - `--data-dir docs/paper/figures/data_r2`;
   - drift measured over the zero-command scenarios only. The old "World drift" column averaged
     in commanded driving distance.
3. **Firmware** (`TwoWheeledRobot_Embedded`):
   - **No observation change needed.** Training now feeds back the clamped command, which is
     exactly what `main.c` already does.
   - C (GRU): run `Tools/expand_gru_for_stm32ai.py` on the new `policy_drive.onnx`, then
     X-CUBE-AI generate.
   - A, B and the new **D** ([145,145] MLP, 23,492 params, ~94 KB weights, same as the GRU)
     each need X-CUBE-AI generate, plus a `POLICY_RANGE_WIDE` option next to
     `ACTIVE_NN_POLICY` in `main.c`.
   - Record the firmware commit hash for each capture.
4. **Robot**: the 11-stage test bench for A, B, C and D at nominal wheels, plus C with the
   larger left wheel. Record battery voltage, floor surface and startup outcome. If time allows,
   repeat each a few times with the policy order randomized; that's the reviewer's
   "independent trials".

## 6. What changed vs. the August arms (for the paper's methods)

| Item | August arms | r2 | Evidence |
|---|---|---|---|
| obs 9–10 "previous current" | motor-model current (after delay/gain/deadzone) | clamped policy command, **same as firmware** | Old arms evaluated with the firmware's version: deployed MLPs degraded (A station-keeping pitch 1.80→2.95°), GRU barely changed |
| Current-loop lag τ | 5–10 ms, applied per 15 ms step → had **no effect** | removed | Telemetry: measured current = command 2 ticks (30 ms) earlier, no slow rise (R² 0.94–0.97). That's transport delay, which the 0/1-step delays model |
| Torque-speed envelope | left wheel had motoring/braking swapped; no-load 200 rpm | both wheels correct, per 1 ms substep, **270 rpm** (measured) | Sim probe: left and right limits now identical |
| Tire/floor friction | one value per run (seed 42 always 0.884/0.413) and **halved** by the default robot material (effective 0.44/0.21) | per episode per robot, **effective** 0.5–1.1 static / 0.4–0.9 dynamic, dynamic ≤ static | Material probe |
| Wheel joint friction coeff. | 0.007–0.013 | removed | Sim probe: 0 mNm effect. Friction is modelled once, by the deadzone |
| Wheel viscous damping | 0.006–0.014 (assumed) | **0.0007** fixed (measured) | Slope of EMB-18 descending sweeps |
| Current limit 1.9–2.1 A, bias 0 | randomized / fixed | removed | No current >1.9 A in a stage-5 replay |
| Obs/action delay {0,1} | hard-coded, **also in arm A** | same, now in the config and stated for every arm | — |
| Leg gains kp/kd | ±5% in B/C only | fixed 30/3 in every arm | A vs B now differ **only** in wheel gain + deadzone |
| COM fore-aft ±1 mm | on | removed (COM height stays) | negligible by design |
| Reward | "excess position" (0.03% of reward), +3 clip (never reached) | both removed | Stage-5 reward replay |
| Station-keeping drift reward | distance from episode spawn | distance from where the current hold began | Spawn version saturated on 7.5% of hold steps |
| Action noise floor | 0.15 (noise pinned at it all run) | 0.05 | Aug 23 fine-tune: better tracking in all 11 scenarios |
| Checkpoint | gated shortlist; final picks all failed their gate; selected on seed 42 = report seed | fixed budget 300/350/200/250/300, last checkpoint | — |
| Arms | A point, B range, C GRU | + **D: [145,145] MLP**, capacity-matched to the GRU (23,492 vs 23,618) | separates memory from network size |
| Benchmark | 64 envs, GRU state carried between scenarios | 256 envs, GRU reset per scenario | — |

## 7. Paper text still to write/fix (from the review audit)

- **Conventions**:
  - pitch = atan2(g_y, −g_z) and roll = atan2(g_x, −g_z) from body gravity;
  - pitch/roll rates from the body gyro;
  - yaw rate = world-z angular velocity (identical in sim and firmware);
  - heading = ZYX yaw;
  - sim fall = total tilt arccos(−g_z) > 25° for 5 consecutive 15 ms steps;
  - hardware fall = |pitch| > 0.60 rad, latched until |pitch| < 5°.
- **Hardware metrics**: the current column is RMS of the mean |i| over both wheels; pooled
  RMS would read B 0.193 / C 0.129 A. Drift = mean over the 3 zero-command stages of the
  maximum |x_odom − x_odom(stage start)|. The SD is the sample SD over stages, not over trials.
- **Provenance**: the August captures ran firmware `49f423d` (not `e4aba9b`). The MCU runs at
  84 MHz. Cube.AI 8.1.0. The firmware ONNX files are byte-identical to the archived exports.
- **Firmware hidden-state reset**: also at every test-bench checkpoint and after a fall, not
  only at boot.
- **Still open, state as limitations**:
  - the stopping-edge deadzone has no breakaway or hysteresis;
  - leg-zero calibration ("physical 10° = logical zero" is asserted, not measured);
  - roll sign not bench-verified;
  - no on-target Cube.AI validation.

## 8. Where things are

- Review: `~/Downloads/review-2026-09-30-101737.pdf`; manuscript: `~/Downloads/IEEE_Access_Paper.zip`.
- Evaluation of the August arms (E1 obs semantics, E2 limiter, GRU reset, friction sweep):
  `logs/paper_review_eval_2026-09-30/`. It has the harness, JSONs and `eval_summary.txt`.
- Pre-flight check: `scripts/tools/verify_retrain_changes.py`. Driver: `scripts/tools/run_retrain_arms.sh`.
- `git stash@{0}` (from `feature/hold-smoothness`) is the left-wheel limiter fix, now superseded
  by `4ec192e`. It can be dropped.
