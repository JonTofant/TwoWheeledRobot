# 2026-11-05 — Why the retrained policies are worse on the robot

Sim and offline telemetry analysis only. Nothing was flashed, retrained or committed.
Harness: `~/Projects/TwoWheeledRobotSim2Sim/regression_2026-11/` (MuJoCo plant, firmware-exact
observation pipeline, 66.7 Hz). Python: `~/miniconda3/envs/twowheeledmujoco/bin/python`.

## Short answer

1. **Isaac cannot see this failure.** It has no wheel Coulomb friction and treats reported wheel
   velocity as instantaneous. On an Isaac-like plant the failed C pilot (0.44 deg) beats the August
   GRU (0.51 deg). The real plant reverses that.
2. **The retrained policies really are less robust; it was not a lucky August seed.** All 3 August
   GRU seeds hold 1.8-2.7 deg on the fitted plant. All 7 post-fix GRUs are 4.3-6.0 deg, and one falls
   every time (exact Mann-Whitney p = 0.017).
3. **The strongest suspect is `prev_current_obs_source=command`** (commit 4ec192e). Not proven; see
   "What is not established".

## Ruled out

- **The firmware.** `main.c`/`DDSM115.c`, 49f423d vs the current branch: only the D slot, debug
  counters and leg telemetry differ.
- **The robot.** 11-03 recording, August GRU on current firmware: 474 s, no falls, velocity lag
  105 ms, current response unchanged.
- **The IMU.** Mainline already reads the BNO08x directly; gyro vs d(pitch)/dt lag is about -2 ms.

## Measured from telemetry

| Quantity | Value | Sim assumption |
|---|---|---|
| Reported wheel velocity vs d(encoder)/dt | filter inside the DDSM115, centroid 131.6 / 131.7 ms in two independent captures, -44 deg at 1 Hz, -77 deg at 2 Hz | 0-1 tick (0-15 ms) |
| Velocity resolution | 1 rpm (firmware passes it through unfiltered) | continuous + noise |
| Command to body response | about 6-10 ms (gyro) | 0-1 tick |
| Command to reported current | 2 ticks, 0.8 gain | - |

The 30 ms "transport delay" behind removing the lag model in d2e3992 is mostly the age of the
current report, not torque delay.

## Model validation

Fitted on the August GRU and the C pilot: wheel Coulomb friction 0.03 Nm per wheel, deadzone
0.05 A, gain 0.85, delay 10 ms, identified velocity filter. The range and point MLPs were not used.

| Policy | Robot: pitch SD, freq, outcome | MuJoCo | In fit? |
|---|---|---|---|
| August GRU | 1.89 deg, 0.83 Hz, survived | 2.68 deg, 0.82 Hz, survived | yes |
| Range MLP | 4.81 deg, 0.45 Hz, survived | 4.73 deg, 0.43 Hz, survived | **no** |
| C pilot | 7.93 deg, 0.56 Hz, fell at 5.3 s | 6.00 deg, 0.53 Hz, survived | yes |
| Point MLP | 11.8 deg, 0.56 Hz, fell | 9.74 deg, 0.47 Hz, survived | **no** |

The model gets frequencies and ranking right but **does not reproduce the two falls** at the best
fit. Treat it as a ranking and mechanism tool, not a fall predictor. MLP drive speeds in this plant
also look wrong (`aug_mlp` at a 0.3 m/s command), so drive numbers are for before/after only.

## Which plant effect does what (ablation, fitted plant)

| Removed | August GRU pitch SD | C pilot pitch SD |
|---|---|---|
| nothing | 2.69 | 6.01 |
| wheel friction | **0.49** (2.0 Hz, cycle gone) | 4.16 |
| velocity filter | 2.29 | **1.56** |
| both, gain 1 (Isaac-like) | 0.51 | 0.40 |

So the long-open 0.83 Hz limit cycle and the ~3.5x station-keeping gap are wheel Coulomb friction.
The C pilot's growing 0.55 Hz oscillation is mainly the velocity filter.

The earlier "torque is 0.5x" reading was an artefact: friction lowers the fitted gain. With friction
modelled the regression matches at gain 0.85-1.0. **Do not scale firmware current by 1.6x.**

## Policy generations (GRUs, stock firmware, fitted plant)

| Policy | Self-feedback dI/dprevI | Pitch SD | With encoder velocity | Isaac-like |
|---|---|---|---|---|
| August s42 (flashed) | +0.39 | 2.68 | 2.35 | 0.51 |
| August s43 | +0.48 | 1.76 | 1.57 | 0.26 |
| August s44 | +0.13 | 2.62 | 2.22 | 0.68 |
| r2 s42 | -0.80 | 5.91 | 2.57 | 0.52 |
| r2 s43 | -0.85 | 4.49 | 2.12 | 0.90 |
| r2 s44 | -0.84 | falls 100% | falls 100% | 1.88 |
| C pilot s43 | -1.22 | 6.00 | 1.99 | 0.44 |
| old-plant s42 | -0.82 | 5.75 | 2.32 | 1.40 |
| old-plant s43 | -0.82 | 4.36 | 2.94 | 1.36 |
| old-plant s44 | -0.83 | 4.33 | 2.09 | 1.13 |

Restoring damping (old-plant) and restoring the floor (C pilot) each left the sign unchanged and the
result bad. In a simple recursion I[k] = -0.82 I[k-1] + f(state), slow gain is multiplied by 0.55 and
every-other-tick gain by 5.6, which fits the 33 Hz alternation captured on the robot. August MLPs
(also negative, -0.38/-0.47) are also worse, which fits, but this is correlation.

Also tested and rejected as explanations here: "low use of reported velocity" (r2 s42 and old-plant
s42 use little yet are bad), and the exploration floor (it tracks pitch gain but not outcome).

## Firmware option, untested on the robot

Compute obs[1] from encoder position over 2 ticks instead of the motor's filtered velocity. Position
had 0 glitches in about 20,000 ticks; 2-tick velocity noise is about 0.02 m/s (training assumed
0.01). Driving is unchanged or better in the plant. Pitch SD results, fitted plant: August GRU
2.69 -> 2.35, C pilot 5.97 -> 1.98, r2 s43 4.55 -> 2.12, range MLP 4.74 -> 1.65.

This changes the observation of a hardware-proven policy, so keep a compile-time switch.

## Before flashing anything else

Across the plant variants on stock firmware, old-plant MLPs fall 69-100%, and r2 MLP 78%. Old-plant
GRUs are predicted to oscillate 4-6 deg. That is the cohort the paper's pending C/D hardware section
plans to test.

## What is not established

- **Which training change caused it.** Everything is confounded: `prev_current=command`, limiter
  sign, 270 rpm, hold reward and delay handling landed together. Settling it needs a single-factor
  retrain with `env.prev_current_obs_source=model`.
- Friction 0.03 Nm comes from fitting closed-loop behaviour, not from a bench measurement.
- The plant is one rigid body: no leg compliance, no Stribeck curve.
- Nothing here has been confirmed on hardware.

## Reproduce

Run from `~/Projects/TwoWheeledRobotSim2Sim/regression_2026-11/`:
`signal_lags.py`, `velocity_filter.py`, `vel2.py` (signal chain), `fit_plant.py`, `fir_id.py` (plant
fit), `scan2.py` (friction fit), `ablate2.py`, `screen2.py` + `show2.py`, `seeds.py`, `prev_sep.py`,
`drive.py`, `hwchecks.py`, `rec_1103.py`. `scan.py`, `ablate.py`, `screen.py` are the superseded
deadzone-only plant, whose 0.5x gain conclusion was wrong. Isaac parameters: `dump_isaac.py`.
