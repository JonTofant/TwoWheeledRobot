# LQR Model Suspended-Air Test Report

Date: 2026-06-03

## Purpose

Prepare and validate the analytical LQR current controller while the robot is
suspended above the floor. This test checks model calculation, command signs,
and the DDSM115 motor-model path. It does not test or claim balancing
performance because the wheels have no ground contact.

## Changes Made

- Added `--test-mode lqr-model` to `scripts/lqr_control.py`.
- Added the supplied physical parameters, analytical `A` and `B` matrices,
  LQR weights `Q` and `R`, and continuous-time Riccati solution.
- Added an artificial pitch input for suspended-air testing.
- Added a `+1 degree` / `-1 degree` current sign check.
- Routed the LQR desired current through the shared DDSM115 current, torque,
  saturation, and torque-speed model.
- Added a separate opt-in `--test-mode lqr-floor` mode for later floor-contact
  testing with a constrained `0.5-3 degree` initial pitch.
- Documented both modes in `README.md`.

## Analytical Result

The computed gain matrix was:

```text
K = [[-27.83088906  -6.04751024  -1.          -3.20947983]]
```

The suspended sign test produced:

```text
theta = +1.0 deg -> i_des = +0.485741 A
theta = -1.0 deg -> i_des = -0.485741 A
```

Positive and negative pitch therefore produced opposite nonzero current
commands. A larger artificial tilt of `10 degrees` requested `+4.857406 A`,
which exceeds the DDSM115 simulated peak-current clamp of `2.7 A`.

## Suspended Simulation Test

Command used:

```bash
python3 scripts/lqr_control.py \
  --task Template-Twowheeledrobot-Standup-v0 \
  --test-mode lqr-model \
  --artificial-pitch-deg 1.0 \
  --headless \
  --no-plot \
  --log-csv ''
```

The first DDSM115 model step reported:

```text
i_des       = [+0.485741, +0.485741] A
i_cmd       = [+0.485741, +0.485741] A
tau_current = [+0.364305, +0.364305] Nm
tau_actual  = [+0.364305, +0.364305] Nm
```

Both free-spinning wheels accelerated in the positive corrective direction and
reached approximately `+199 rpm`, close to the modeled `200 rpm` no-load speed.

## Acceptance Result

- LQR gain matrix computed successfully: PASS
- Positive and negative pitch produce opposite current commands: PASS
- Current command passes through the DDSM115 motor model: PASS
- Wheels spin for a nonzero artificial pitch: PASS
- No floor-balancing result claimed while suspended: PASS

## Not Tested

At the time of the suspended-air test, the `lqr-floor` mode had not been run.

## First Floor-Contact Test: 0.5 Degrees

The first constrained floor-contact test was run at an initial model-state
pitch of `+0.5 degrees`. No larger initial angles were tested.

Before the run, the floor mode was corrected to use the robot's actual motion
geometry: the wheels drive along world `Y`, and pitch is rotation about the
wheel axle (`X`). Wheel contact-force sensing and the required CSV signals were
also added.

Startup checks confirmed:

- No base hold joint was present, so the robot was not suspended.
- Gravity was enabled.
- Ground friction was enabled with static coefficient `0.6` and dynamic
  coefficient `0.4`.
- Both wheels were touching the floor.
- Both wheels received the same desired current.
- Desired current passed through the DDSM115 current and torque model.
- Current saturation was logged.

Generated logs:

```text
outputs/lqr_floor_0p5deg_motors_disabled.csv
outputs/lqr_floor_0p5deg.csv
```

### Motors-Disabled Baseline

The motors-disabled baseline reached the `10 degree` stop threshold after
`0.30 s`:

```text
theta initial = +0.3782 deg
theta final   = +10.1331 deg
max wheel RPM = 33.75
```

### LQR-Enabled Result

The LQR-enabled run completed the full `5.00 s` test window:

```text
theta initial           = +0.3328 deg
theta range             = -0.4907 to +2.2864 deg
theta final             = +1.8973 deg
initial desired current = +0.242870 A per wheel
max requested current   = 6.5434 A
current saturation      = 30.1 percent of samples
max absolute wheel RPM  = 176.66
minimum contact force   = 1.846 N left, 7.037 N right
base Y displacement     = -0.3894 m
```

The first wheel motion was in the corrective direction and `theta` initially
moved toward zero. The LQR robot remained upright substantially longer than the
motors-disabled baseline, current was not permanently saturated, wheel speed
remained below the modeled `200 rpm` no-load speed, and wheel contact remained
continuous.

### Floor-Test Assessment

The constrained `0.5 degree` sign and survival test passed. The response is not
yet well settled: it oscillates, intermittently saturates, and drifts
significantly. No motor sign flip was made, and no `Q` or `R` tuning was
performed. The `1`, `2`, and `3 degree` tests should wait until this behavior is
reviewed.
