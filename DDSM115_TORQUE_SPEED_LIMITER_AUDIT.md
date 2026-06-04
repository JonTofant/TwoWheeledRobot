# DDSM115 Torque-Speed Limiter Audit

## Scope

This audit investigates the apparent case where a roughly `4 rpm` post-step wheel-speed difference was logged beside a roughly `1.49 Nm` torque-limit difference. No controller gains, motor constants, limiter equations, friction values, collision geometry, or physical parameters were changed.

## Implementation Audit

The limiter is implemented once, vectorwise, for both wheels in `StandupEnv._pre_physics_step`:

```python
self._wheel_velocity_raw = self.robot.data.joint_vel[:, self._wheel_ids].clone()
self._wheel_velocity_used = self._wheel_velocity_raw.clone()
self._wheel_omega_for_limiter = self._wheel_velocity_used.abs()
self._wheel_tau_speed_limit = DDSM115_TAU_PEAK * (
    1.0 - self._wheel_omega_for_limiter / DDSM115_NO_LOAD_SPEED
)
self._wheel_tau_speed_limit = self._wheel_tau_speed_limit.clamp(0.0, DDSM115_TAU_PEAK)
```

The wheel index tensor is constructed in explicit left/right order from the named joints:

```python
DDSM115_Levi
DDSM115_Desni
```

Audit findings:

1. Both wheels use the same vectorized limiter equation.
2. Both wheels use raw joint velocity in `rad/s`.
3. Both wheels use signed raw velocity followed by the same `abs()` operation.
4. Neither wheel uses filtered velocity, wheel position, or a different unit.
5. The limiter uses the current pre-physics-step wheel velocity, not an outdated value for control.

## Added Diagnostics

The environment now preserves the exact values used at the limiter calculation site:

```text
wheel_velocity_raw
wheel_velocity_used
omega_for_limiter
tau_speed_limit
tau_current
tau_actual
```

The CSV logger records left/right raw and used velocity in `rad/s`, used velocity in `rpm`, limiter omega, torque limit, current torque, and actual torque. It also prints every sample where the left/right torque-limit difference exceeds `0.2 Nm`.

## Suspicious Floor Sample Explained

At CSV time `2.955 s`, the old post-step state columns report:

```text
left_wheel_rpm  = -153.1479 rpm
right_wheel_rpm = -157.2048 rpm
```

Those values are read after the physics step. The torque limits in the same row were calculated before that physics step from:

```text
left_wheel_velocity_used_rpm  =  +14.2786 rpm
right_wheel_velocity_used_rpm = -163.3597 rpm

left_omega_for_limiter  =  1.495255 rad/s
right_omega_for_limiter = 17.106983 rad/s
```

Applying the intended equation gives:

```text
left_tau_limit  = 1.857214 Nm
right_tau_limit = 0.366403 Nm
```

Therefore, the `1.49 Nm` limit difference is consistent with the actual limiter inputs. The apparent `4 rpm` discrepancy came from comparing post-step RPM to pre-step limiter output during a contact transient where wheel velocity changed sharply within one `5 ms` physics step.

## Suspended Equal-Current Test

Command:

```bash
python3 scripts/lqr_control.py \
  --task Template-Twowheeledrobot-Standup-v0 \
  --test-mode free-spin \
  --left-motor-test-current -2.7 \
  --right-motor-test-current -2.7 \
  --free-spin-test-time-s 5.0 \
  --seed 42 \
  --headless \
  --no-plot \
  --log-csv outputs/ddsm115_torque_speed_audit_suspended.csv
```

Results:

```text
Final left RPM                       -199.8039 rpm
Final right RPM                      -199.4577 rpm
Maximum post-step RPM difference        3.3531 rpm
Maximum limiter omega difference        0.3511 rad/s
Maximum torque-limit difference          0.0335 Nm
Maximum limiter-equation residual       < 1e-7 Nm
Raw-to-used velocity residual             0.0 rad/s
RPM conversion residual                   0.0 rpm
```

Both suspended wheels accelerate and converge nearly identically. No sample exceeded the `0.2 Nm` torque-limit difference print threshold.

## Deliverables

```text
outputs/ddsm115_torque_speed_audit_suspended.csv
outputs/ddsm115_torque_speed_audit_floor.csv
outputs/ddsm115_torque_speed_audit/summary.csv
outputs/ddsm115_torque_speed_audit/ddsm115_torque_speed_audit_suspended.png
outputs/ddsm115_torque_speed_audit/ddsm115_torque_speed_audit_floor.png
scripts/analyze_ddsm115_torque_speed_audit.py
```

## Conclusion

The DDSM115 torque-speed limiter passes the audit. The observed torque-limit difference is physically and numerically explainable from the actual pre-step wheel velocities used by the limiter.

The previous yaw log combined pre-step motor-model values with post-step wheel-state values, so its RPM and torque-limit columns were not time-aligned. The new limiter-input RPM columns are the correct values to compare against `tau_speed_limit`.

The remaining concern is not the limiter equation but the large wheel-speed changes occurring within individual floor-contact physics steps. That behavior points back toward contact dynamics, collision geometry, solver behavior, or insufficiently resolved wheel-ground interaction before further LQR tuning.
