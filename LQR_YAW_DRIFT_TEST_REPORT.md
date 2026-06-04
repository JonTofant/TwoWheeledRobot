# LQR Floor-Contact Yaw Drift Test Report

## Scope

This test investigates yaw drift before any further LQR tuning. No `Q`, `R`, controller-gain, friction, collision, or geometry changes were made.

The floor-contact logger in `scripts/lqr_control.py` now records base yaw, world-frame yaw rate, all base-position components, exact left/right wheel, current, torque, and contact-force signals, plus:

```text
rpm_diff
current_diff
torque_diff
contact_force_diff
```

## Experiment

Three identical runs were executed with:

```text
Initial pitch = +0.5 degrees
Duration = 5.0 seconds
Seed = 42
LQR enabled
```

Generated CSV logs:

```text
outputs/lqr_floor_yaw_run1.csv
outputs/lqr_floor_yaw_run2.csv
outputs/lqr_floor_yaw_run3.csv
```

Generated plots and machine-readable summary:

```text
outputs/lqr_yaw_analysis/lqr_floor_yaw_run1_signals.png
outputs/lqr_yaw_analysis/lqr_floor_yaw_run2_signals.png
outputs/lqr_yaw_analysis/lqr_floor_yaw_run3_signals.png
outputs/lqr_yaw_analysis/repeatability_summary.csv
outputs/lqr_yaw_analysis/analysis.md
```

## Repeatability Results

All three CSV logs are byte-for-byte identical. Each run produced:

```text
Final yaw angle                     +15.0065 deg
Maximum absolute yaw angle           21.3453 deg
Maximum absolute yaw rate            63.8712 deg/s
Final wheel position                 -0.2593 m
Wheel-position change                 -0.2590 m
Final base displacement              (-0.0813, -0.3898, -0.0025) m
Maximum absolute theta                2.2864 deg
Maximum absolute RPM difference     184.0815 rpm
Maximum absolute current difference   0.0000 A
Maximum absolute torque difference    1.4908 Nm
Maximum contact-force difference     32.9161 N
```

Yaw direction is consistent across all runs, so this is Case A rather than stochastic contact behavior.

## Interpretation

1. **Yaw is not growing monotonically.** It accumulates and reverses during the repeated balancing recovery transients. For example, yaw change is approximately `-15.21 deg` at `0.5 s`, `-12.44 deg` at `2.0 s`, `-21.35 deg` near `3.65 s`, and `+15.01 deg` at `5.0 s`.

2. **Left and right wheel RPMs are not approximately equal.** The maximum absolute RPM difference is `184.08 rpm`, and yaw rate correlates strongly with signed RPM difference (`r = -0.672`).

3. **Left and right commanded currents are exactly equal, but actual motor torques are not.** The controller produces zero current difference for the entire run. Per-wheel torque-speed limiting then creates up to `1.49 Nm` of actual torque difference after the wheel speeds diverge.

4. **Left and right contact forces are not equal at the initial contact transient.** The first logged sample is approximately `63.61 N` left and `30.70 N` right. Across the run, however, signed contact-force difference has essentially no linear correlation with yaw rate (`r = 0.004`).

5. **Yaw drift correlates much more strongly with wheel-speed asymmetry than with contact-force asymmetry.** Torque asymmetry has a weaker yaw-rate correlation (`r = -0.191`) and appears partly downstream of the wheel-speed difference because the DDSM115 torque-speed curve is applied independently to each wheel.

## Conclusion

The remaining yaw behavior is not caused by unequal LQR current commands and should not be addressed by tuning `Q` or `R` yet. The most likely source is a deterministic physical-model asymmetry, most plausibly collision/contact geometry or a small left/right assembly asymmetry that seeds unequal wheel motion. The common-mode pitch controller has no yaw correction, and the per-wheel torque-speed limiter amplifies the resulting wheel-speed divergence into actual torque asymmetry.

The wheel-axis convention is unlikely to be the primary issue: both forward-positive wheel signals respond in the same general direction, and the mirrored joint effort signs are already handled explicitly. The next investigation should isolate left/right collision geometry and wheel contact placement before controller tuning.
