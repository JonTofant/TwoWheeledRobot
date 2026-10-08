# C pilot hardware standing failure, 7 October 2026

The fresh C seed-43 pilot failed the basic hardware standing check. It did not reach the automatic testbench. The telemetry records growing balance oscillations followed by forward wheel-speed runaway and a fall, with commanded velocity and yaw both zero. The supplied C ONNX reproduces all 356 active samples' network currents to float precision. This supports correct deployment inference and a policy/physical-plant balance failure. It does not establish which training change caused the regression.

## Evidence and provenance

- Hardware telemetry commit: `5b4b481`, `Results/paper/C_pilot_test.csv` in `JonTofant/TwoWheeledRobot_Embedded`, branch `feature/mdpi-actuators`.
- Firmware inspected: `44eb23a`, pushed during this audit. Full revision and capture SHA256 are in `analysis_summary.json`.
- Model: C selected iteration 1150, checkpoint SHA256 `4eec9bfce0ecd2e567a984f3e89baac0e1ed331fae75f31be8b066b256201540`.
- Deployment ONNX SHA256 `1a8e67c916bbddb7d871aeaadd24d9255207789afde0d2ae753a555062064431`.
- Recorded legacy policy ID is 2, range GRU. No ELF/model-hash sidecar was supplied, but recurrent replay strongly identifies the supplied C policy behavior across the full active interval. The inspected source is not a binary attestation of the flashed ELF.
- Raw capture copied byte-for-byte locally. No source, model, firmware or experiment settings were modified. No training, flashing or robot commands were performed.

## What happened

| Event or measurement | Recorded value |
| --- | ---: |
| Recording length / rows | 25.156 s / 1678 |
| Upright rearm | 14.390 s, sequence 663 |
| Pitch at rearm | +4.82 degrees |
| Fall latch | 19.726 s, sequence 1019 |
| Rearm to fall | **5.336 s** |
| Pitch at fall latch | +35.24 degrees |
| Peak wheel-odometry speed before fall | **1.171 m/s** |
| Zero-command velocity RMS during active interval | 0.416 m/s |
| Active-interval pitch RMS | 8.67 degrees |
| Wheel-odometry displacement during active interval | +1.104 m |
| Unwrapped yaw change during active interval | -33.19 degrees |
| Velocity command / yaw command | zero throughout |
| Automatic testbench stages started | **zero** |
| Physical fall-latch transitions after rearm | **one** |

The file begins with the robot already marked fallen. It is manually brought upright and control rearms inside the existing ±5-degree hysteresis. Pitch then oscillates from about +5 degrees to -8 degrees, +9 degrees, -11 degrees and back to +11 degrees. Forward wheel-speed peaks grow from roughly 0.30 to 0.48 to 1.17 m/s. The final pitch excursion crosses the firmware's 0.60 rad fall threshold and current commands are cut to zero.

The user's report supplies the context that no push occurred; the telemetry cannot itself measure an external hand force or identify the instant the robot was released. The 5.336 s interval is measured from firmware rearm, not from a separately recorded release event. Odometry displacement/speed can include slip or spinning wheels; they are not external motion-capture measurements.

`testbench_fall_count` remains zero because the testbench never ran. That counter does not mean zero physical falls. Aggregate `theta_*`, `current_rms_A` and `run_time_s` metrics are also zero/uninitialized in this file, so this analysis uses raw samples. After the fall, network observations/actions remain stale while wheel commands are zero; those stale fields are excluded from inference/tracking calculations.

![Standing failure](standing_failure.png)

## Deployment and firmware audit

Replaying each recorded 13-value observation through the exact supplied C STM32AI ONNX, with hidden state initially zero and retained between ticks, yields:

- Maximum absolute network-current difference: **7.15e-7 A**.
- RMS current difference: **1.86e-7 A**.
- Agreement across all 356 active samples, including the final pre-fall current output.

That agreement strongly rules out an incorrect C checkpoint, missing current scaling, a GRU gate conversion error, per-tick hidden-state clearing or a wrong hidden-state sequence as the cause of this capture. It verifies the input-to-network-output path, not motor torque, wiring truth, sensor calibration truth or the entire binary.

Recorded observation pitch/rate/roll/velocity/world-yaw-rate scaling matches the inspected firmware formulas to float precision. Previous-current observations exactly equal the preceding logical outputs divided by 2. Physical command destinations/signs exactly match the existing cross-wheel mapping. These signals are internally consistent; physical coordinate/calibration correctness still requires the established hardware reference.

Firmware commit 44eb23a adds a separate D network, selection ID 3, static shape checks and inference diagnostic counters. It does not change C normalization, current signs/destination swap, command slew, reference anti-windup, GRU reset/persistence, motor drivers, CyberGear gains, estimator or fall detector. Generated application init/process hooks remain inert; `ANN_Run()` owns scheduled inference. C generation reports the expected obs/h_in and current_a/h_out ports and 1332 bytes of activations.

Timing while C is active:

- Inference mean 2.925 ms, maximum 2.955 ms.
- Recorded policy/control-body maximum 3.308 ms.
- Control period 14.9992–15.0008 ms.
- Wheel-feedback age median 9.30 ms, maximum 12.71 ms.
- Consecutive sequence IDs, zero reported dropped frames, finite numerical data and IMU-valid flags throughout.

There is no evidence of a stalled inference or missed 15 ms release in this run. The control-body measurement does not include every asynchronous peripheral operation; it is not a complete MCU utilization estimate. Recorded wheel age is also not a direct measurement of total sensor-to-torque latency.

## Currents and offline diagnostic lead

Commands were not uniformly small. Immediately before the fall, logical C currents were **1.385 A / 1.128 A**. Reconstructed logical current feedback was about **0.479 A / 0.491 A**. The gap is observed at high wheel speed and cannot alone identify whether motor limits, response delay or another plant effect dominates. The file does not support applying an arbitrary output multiplier as a fix.

There is substantial every-other-tick current modulation, corresponding to about 33 Hz at a 66.7 Hz controller. During the 18.20–18.65 s window, mean absolute current change per adjacent tick is 0.438 A for C, versus 0.057 A when the original GRU is replayed on the same recorded observations. That is a diagnostic open-loop comparison: the original GRU receives C's observed physical trajectory and action-history inputs, so it does not establish that the original would have prevented this exact fall.

![Current variation](current_alternation.png)

Additional frozen-input probes initialize hidden state from the recorded history, hold physical observation channels constant, and feed the model's outputs back into its previous-action channels. At sequence 703, C exhibits a long alternating transient: mean adjacent-tick change is about 0.200 A over steps 200–299. A longer 1200-step probe shows eventual settling, with final mean change about 0.000016 A. Therefore this is **not proof of a permanent C limit cycle**. The original GRU has much smaller variation under the same probe.

D at sequence 928 produces a persistent two-tick cycle in this artificial frozen-input test (0.459 A mean adjacent change, near-zero two-step change). Holding its previous-action inputs constant removes that cycle. This demonstrates a learned action-feedback sensitivity for D at that input, not a measured D hardware failure. The recovered GRU also shows offline sensitivity at some tested inputs; it must remain separately identified from both the original reference and fresh C.

These diagnostics make observation/action-history sensitivity and balance damping under real sensor/motor timing useful next hypotheses. They do not identify one correction, the exploration floor, short training budget or architecture as the proven root cause. Raw replay and probe JSON are retained alongside the scripts for reproducibility.

## Historical context and decision

Earlier repository captures `gru_testbench.csv` and `range_gru_segmented.csv` show zero fallen frames while recording about 70.1 s and 103.9 s of running testbench respectively. They support the reported historical ability to complete substantial hardware testing. They are not matched repeats of today's standing trial and lack exact model/build identities in their CSVs, so their metrics should not be pooled into a formal new-versus-old estimate.

The new C is **not ready for a scored full hardware protocol**. Preserve this run as a failed standing/deployment trial, with zero protocol completion, rather than dropping it because the automatic bench did not start. Simulator selection gates passed, but the hardware standing gate failed. Current evidence contradicts an improvement claim for the new C on this robot. It neither invalidates the structural correctness fixes nor proves GRUs generally inferior.

Keep the original proven rollback as the working reference. Before another long retrain, use these measured inputs and timing to compare the old/new frozen policies and test the relevant plant/feedback hypotheses. A same-firmware, same-start old/new standing capture would strengthen the causal comparison. No additional training was launched during this audit.

## D capture compatibility

The newly pushed firmware implements D as `POLICY_RANGE_WIDE_MLP`, ID **3**, network `nn_arm_d_range_matchedsize_mlp`. Yesterday's handoff instructions and wrapper assumed D reused the B slot, ID 1. The new firmware's `Tools/paper_uart_capture.py` recognizes ID 3, but the packaged `capture_hardware_run.py` still expects ID 1 and would flag a correct D capture as a mismatch. Do not follow the old slot-selection instructions for this new firmware. This compatibility issue does not explain today's C failure, which used ID 2. Update the D handoff/capture mapping before using that wrapper for D.
