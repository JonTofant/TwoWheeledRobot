# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

README.md covers the task, the file layout and the script command lines. This file covers
only what reading the code will not tell you. Facts below were verified 2026-08-05.

## Running anything

Isaac Lab is **not importable on the host** (`import isaaclab` → ModuleNotFoundError).
Everything runs in the `isaac-lab-dev` Docker container, with this repo live-mounted at
`/workspace/TwoWheeledRobot`:

```bash
docker start isaac-lab-dev   # it is often stopped
docker exec isaac-lab-dev bash -c 'cd /workspace/TwoWheeledRobot && /isaac-sim/python.sh scripts/...'
```

- Kit startup is ~75 s **per invocation**. Batch work into one script rather than many
  short runs.
- GPU is an RTX 5080 (16 GB); 4096 envs uses ~8 GB.
- `DRIVE_TERRAINS_CFG` sets `use_cache=False`, so terrain regenerates on every stage
  (~1 min extra startup each).
- ruff is a host-level user install (`~/.local/bin/ruff`), not part of the container env.
  `ruff check .` currently reports 22 pre-existing violations, mostly E501 — do not chase
  them.

## Coordinate and hardware conventions

- USD world frame: **Y is fore/aft with forward = -Y**, X is lateral (the roll axis), Z is
  yaw. The axis *assignment* is load-bearing — pitch reads body-frame gravity Y, roll reads
  X, and the `AXIS_*` disturbance constants follow the same map. The *sign* is not: the
  robot is near-symmetric, disturbance ranges are symmetric about zero, and
  velocity/position come from wheel odometry rather than world position, so -Y only decides
  which way it drives in the world. It does matter for firmware.
- The left wheel USD is mirrored, so simulation negates left wheel torque. Firmware must
  follow the physical wiring, not the USD.
- The robot is symmetric. Measured airborne with gains pinned: all four legs reach the same
  angle from the same command to within 0.003 deg. Any observed left/right asymmetry is the
  policy, not the plant.
- The authored origin sits slightly below ground. `spawn_upright_z = 0.06859` is the ROOT
  height that puts wheels on the ground, ~17 mm above the wheel centres — not the
  wheel-centre height.

## Architecture facts not visible in one file

- `NNDriveEnv -> PureNNBalanceEnv -> StandupEnv`. The latter two are **base classes only**:
  their gym registrations were removed, they are unreachable as tasks, and they still
  contain dead code (fallen-pose sampling, self-righting rewards) that the drive task never
  uses. Flattening them is an open opportunity.
- **The live control rate is 66.7 Hz (15 ms)**, from `PureNNBalanceEnvCfg.decimation = 15`.
  `sim_params.CONTROL_DECIMATION = 20` and its "20 ms / 50 Hz" comment apply to `StandupEnv`
  only and are dead for the trained task. Changing them changes nothing.
- **`NNDriveEnv._pre_physics_step` does not call `super()`** — it re-implements the DDSM115
  `Kt` / torque-speed-limiter path inline. The copy in `standup_env.py::_pre_physics_step`,
  including the whole `wheel_actuator_model` branch (which still defaults to
  `mujoco_torque`), never executes for `NNDrive`. This is what "physical parameters are
  duplicated by design" means concretely: change every copy together, and remember that
  editing only the `standup_env.py` one is a silent no-op.
- The per-unit wheel motor model *is* shared: `CurrentActionProcessor` in
  `pure_nn_components.py` owns tanh→ampere scaling, gain/deadzone/bias draws, action delay
  and current-loop lag for both envs.
- Observations are 20 wide. `roll`/`roll_rate` are appended at `[18]`/`[19]` rather than
  grouped next to pitch so that firmware indices 0-17 keep their meaning.
- The curriculum spawns a **fresh `train.py` subprocess per stage**. Breaking an import
  mid-run kills every remaining stage — never edit source while training.
- Hydra overrides reach any cfg field as `env.<field>=<value>`; every reward weight can be
  zeroed from the CLI to bisect which term caused a behaviour change.
- `train_nn_drive_curriculum.py --load-run` pins the resume baseline. Without it the resume
  point is newest-mtime, so repeated single-stage experiments silently chain off each other
  instead of off a fixed baseline.
- The export's `--obs-dim` must equal `observation_space`. A stale value does not corrupt
  the graph; it silently skips validation unless `--require-validation` is passed.

## Verification discipline

There is no test suite. Correctness means "physically plausible sim behaviour", checked by
running scripts and reading output.

- **Training-log metrics actively mislead.** See `docs/experiments/2026-07-28`: a 98%
  `turn_in_place` fall rate and a total absence of velocity response were both invisible in
  them; three rounds of hypothesis-then-retrain (7.4 h) found nothing that a 20-minute
  velocity sweep found immediately. Always benchmark a frozen checkpoint.
- **Benchmark attitude figures are alive-masked**: a policy that falls fast shows excellent
  pitch/roll. Read `fall_rate` and episode length first or the attitude numbers mean nothing.
- Success criteria for a drive policy: `mean_noise_std` < 0.4, station-keeping fall rate ~0,
  achieved speed > 0.2 m/s at a 0.40 m/s command.

## Isaac runtime traps

- `env.close()` tears down the whole simulation app — a make/close-per-scenario loop
  silently exits 0 after the first scenario. Build one env and mutate `env.unwrapped.cfg`
  between `reset()` calls (see `benchmark_nn_drive.py`).
- Stepping under `torch.inference_mode()` turns env buffers into inference tensors, so a
  later `reset()` outside that context crashes on in-place writes. Wrap reset and stepping
  together.
- Scripts must `parse_known_args()` **before** `AppLauncher` and reassign `sys.argv`, or the
  launcher consumes their arguments.

## The firmware repository

The STM32 firmware is a **separate repo** at `~/Projects/TwoWheeledRobot_Embedded` (source
under `STM32_Diablo_Robot_Source/Core/Src/`). It is not vendored here.

- Use the `feature/mdpi-actuators` branch — it exists there too (local + origin), under the
  same name this repo uses. It was branched from `ERK` on 2026-08-05 and **currently points
  at the same commit** (`b395431`, "Add NNDrive policy with reference anti-windup") with no
  commits of its own yet — which is why `ERK` keeps surfacing: despite the name, that is
  where the drive work lives.
- `main` has **diverged**, not merely lagged: 24 ahead / 7 behind, merge base `3eb25d0`. The
  7 main-only commits are unrelated work lines (`Barrier_fixed_dt` and `State-machine`
  merges, two reverts, a pre-mechanical-upgrade tuning midpoint). Do not merge or rebase
  onto `main` expecting a no-op.
- **Always `git fetch` there before reading it** — the working copy is often behind the
  remote. Check `git status` first and prefer fetch + inspect over a blind `git pull`: that
  repo frequently carries uncommitted local changes, and a pull can fail or auto-merge into
  them.
- It carries its own CLAUDE.md and STM32_DEPLOYMENT.md. The deployment contract belongs
  there, verified against firmware that actually runs.
- Confirmed values: `cybergear.c` initialises all four motors at `kp = 30.0f`, `kd = 3.0f`
  (quantised over kp 0-500, kd 0-5), matching sim `CYBERGEAR_STIFFNESS` / `CYBERGEAR_DAMPING`.
- **`DZ_{LEFT,RIGHT}_{POS,NEG} = 0.04 A` in `DDSM115.c` are never read** (verified
  2026-08-23 by grepping every `Core/{Src,Inc}/*.{c,h}` on `origin/feature/mdpi-actuators`:
  the only hits are the definitions and the `extern`s in `DDSM115.h`). The command path is
  `DDSM115setCurrent` / `DDSM115TransactCurrent`: clamp to `DDSM_COMMAND_LIMIT_A`, scale
  `(i/8.0f)*32767`, pack. **There is no deadzone compensation at all** -- the deadzone is
  100% uncompensated, not "under-corrected by a third" as this file previously said. The
  variables are non-`const` and `extern`, i.e. set up for STM Studio Live Expressions
  tuning, and never wired in. (The "below-deadzone fit" comment in `DDSM115.h` is about the
  current *feedback* scale, not command compensation.)

- **`rew_position_far` is dead** (verified 2026-08-23). `_apply_reference_anti_windup` does
  `pos_ref += pos_err - pos_err.clamp(+-clamp)`, which leaves `x_odom - pos_ref` *already
  clamped*. So `position_error_raw()` -- whose docstring promises "a pull toward home at any
  drift distance" -- returns something bounded to +-`cmd_pos_err_clamp_m`, `pos_far` is
  `<= v*step_dt` (~0.0075 m at 0.5 m/s), and the term delivers <=0.003/step against a designed
  0.6/step max. A policy past 0.5 m of drift has NO restoring gradient. `rew_hold_world_drift`
  is the only drift signal in the reward a drifting policy cannot flatten.
- `DisturbanceGenerator.active_for_recovery()` is a ready-made "a disturbance is live"
  predicate with **zero callers**. `_sample_payload` writes `payload_torque` only -- the
  "downward force plus payload_torque" in `nn_drive_env_cfg.py`'s comment does not exist.

## External sources of truth

- Domain-randomization ranges are owned by **Notion**, not this repo: EMB-18 (the DDSM115
  measurement campaign) and EXP-B (the range-trained arm). Every randomized parameter must
  be recorded in the paper's section 2.4 as IDENTIFIED (bench-measured) or ASSUMED (bounded
  guess).
- **PAPER-02 is a standing honesty gate**: a range that is configured but not applied
  silently turns EXP-B into a second point-estimate policy. Extended 2026-08-05 — also check
  the AXIS/INDEX, not just that the write happens: the platform COM offset was applied every
  episode at full magnitude, to the wrong degree of freedom, for the whole history of the
  task.

## Known-unverified

- **`motor_tau_s_range` is inert -- it does nothing at all** (verified 2026-08-23). The range
  is `(0.005, 0.010)` and the control `dt` is 15 ms, so `lag_alpha = dt / clamp(tau, min=dt)`
  is *exactly 1.0* for every draw and `command_current = motor_target` unconditionally. The
  current-loop pole is zero in every environment. It was previously listed here as merely
  unmeasured; measuring it is wasted effort unless the true value exceeds 15 ms, and the fix
  is to widen the range or drop the parameter, not to bench it.
- Whether wheel friction is modelled twice — the measured deadzone AND a joint friction
  coefficient. Open task in Notion.
- `STM32_DEPLOYMENT.md` in this repo is partly **specification, not observation**: claims
  like "firmware MUST slew-limit at 3.0 rad/s" were written as guidance and never verified
  against firmware. Trust the observation/action layout and sign conventions; treat the
  firmware-behaviour claims as unconfirmed.
