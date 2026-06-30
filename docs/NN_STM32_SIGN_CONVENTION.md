# NN STM32 Sign Convention

## 1. Summary

This document describes the sign convention that the current trained pure NN balance controller saw in simulation. It does not redefine or clean up the simulation convention.

Source of truth inspected:

- Environment: `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/pure_nn_balance_env.py`
- Components: `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/pure_nn_components.py`
- Config: `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/pure_nn_balance_env_cfg.py`
- Current exported policy found at: `logs/rsl_rl/pure_nn_balance_two_wheel/2026-06-29_13-23-00_stage5/exported/policy.pt`
- Current ONNX current export found at: `logs/rsl_rl/pure_nn_balance_two_wheel/2026-06-29_13-23-00_stage5/exported/policy_current.onnx`

The trained NN expects these normalized inputs:

```text
obs[0] = x_rel / 1.0
obs[1] = v_forward / 1.0
obs[2] = pitch / radians(25)
obs[3] = pitch_rate / 4.0
obs[4] = yaw_error / pi
obs[5] = yaw_rate / 4.0
obs[6] = previous_left_command_current / 2.0
obs[7] = previous_right_command_current / 2.0
```

The current simulation uses mirrored wheel signs:

```text
wheel_sign = [-1, +1]
left NN wheel quantity  = -left raw joint quantity
right NN wheel quantity = +right raw joint quantity
```

The current simulation applies motor torque signs as:

```text
left sim joint effort  = -left_command_current * Kt, after speed limiting
right sim joint effort = +right_command_current * Kt, after speed limiting
```

Therefore, in the simulation convention used during training:

```text
[+,+] is the NN forward-command pattern.
[-,-] is the NN backward-command pattern.
[+,-] is clockwise/right-yaw in the simulation yaw convention.
[-,+] is counter-clockwise/left-yaw in the simulation yaw convention.
```

Direct TorchScript policy check with crafted normalized observations and zero previous currents:

```text
policy: logs/rsl_rl/pure_nn_balance_two_wheel/2026-06-29_13-23-00_stage5/exported/policy.pt
theta=+5 deg -> raw action [ +0.438631, +0.367204 ], tanh current [ +0.825018 A, +0.703088 A ]
theta=-5 deg -> raw action [ -0.416263, -0.645781 ], tanh current [ -0.787556 A, -1.137646 A ]
```

This confirms that the exported policy currently responds to a +5 degree pitch input with a `[+,+]` forward command and to a -5 degree pitch input with a `[-,-]` reverse command.

Run the deterministic simulator report with:

```bash
python scripts/determine_nn_sign_convention.py --headless
```

## 2. Observation Mapping For STM32

Use the exact same normalized 8-value vector as simulation. The policy export does not add input normalization.

| obs index | NN meaning | simulation sign/source | STM32 formula | notes |
|---:|---|---|---|---|
| 0 | `x_rel` | `0.5 * ((joint_pos_L * -1) + (joint_pos_R * +1)) * R_WHEEL` | `obs[0] = x_rel_m / 1.0f` | Simulation uses the average of both wheels. `R_WHEEL = 0.05035 m`. |
| 1 | `v_forward` | `0.5 * ((joint_vel_L * -1) + (joint_vel_R * +1)) * R_WHEEL` | `obs[1] = v_forward_mps / 1.0f` | Simulation uses the average of both wheels. |
| 2 | `pitch` | `atan2(projected_gravity_b.y, -projected_gravity_b.z)` plus training noise/bias | `obs[2] = theta_rad / 0.436332313f` | For deployment, do not add training noise or random pitch bias. |
| 3 | `pitch_rate` | `-bno080.ang_vel_b.x` plus training noise | `obs[3] = theta_dot_radps / 4.0f` | `theta_dot_radps = -gyro_robot_x_radps` in the sim convention. |
| 4 | `yaw_error` | `wrap(yaw_from_root_quat_wxyz - yaw_reference)` | `obs[4] = yaw_error_rad / 3.141592654f` | At reset/start, set `yaw_reference = current_yaw`. |
| 5 | `yaw_rate` | `root_ang_vel_w.z` | `obs[5] = yaw_rate_radps / 4.0f` | Positive simulation yaw is left/CCW. Clockwise yaw is negative. |
| 6 | previous left current | `action_processor.command_current[:,0]` | `obs[6] = prev_left_command_current_A / 2.0f` | This is after tanh, delay, smoothing, gain, deadzone, current lag/limit; before sim joint-effort sign inversion. |
| 7 | previous right current | `action_processor.command_current[:,1]` | `obs[7] = prev_right_command_current_A / 2.0f` | Same stage as obs[6]. Not raw DDSM motor-side current if STM32 inverts one motor. |

STM32 wheel formulas matching simulation:

```c
const float R_WHEEL = 0.05035f;
const float WHEEL_SIGN_LEFT = -1.0f;
const float WHEEL_SIGN_RIGHT = +1.0f;

float phi_left_nn = WHEEL_SIGN_LEFT * phi_left_raw;
float phi_right_nn = WHEEL_SIGN_RIGHT * phi_right_raw;
float omega_left_nn = WHEEL_SIGN_LEFT * omega_left_raw;
float omega_right_nn = WHEEL_SIGN_RIGHT * omega_right_raw;

float x_rel_m = 0.5f * (phi_left_nn + phi_right_nn) * R_WHEEL;
float v_forward_mps = 0.5f * (omega_left_nn + omega_right_nn) * R_WHEEL;

obs[0] = x_rel_m;
obs[1] = v_forward_mps;
```

If STM32 needs a reset-relative position, subtract encoder references before averaging:

```c
float phi_left_rel_nn = phi_left_nn - phi_left_ref_nn;
float phi_right_rel_nn = phi_right_nn - phi_right_ref_nn;
float x_rel_m = 0.5f * (phi_left_rel_nn + phi_right_rel_nn) * R_WHEEL;
```

The current simulation does not subtract wheel position references inside `_state_terms`; it relies on reset/default joint positions being zero for the wheel joints.

## 3. Action Mapping For STM32

The deployed model path matters:

- `policy.pt` and `policy.onnx` output raw NN actions. STM32 must compute `current = tanh(action) * 2.0`.
- `policy_current.onnx` already appends `current = tanh(actor(obs)) * 2.0` and outputs current in amps.

Simulation action path:

| Stage | Left sign effect | Right sign effect |
|---|---|---|
| policy action | `action[0]` | `action[1]` |
| after tanh/current scaling | `tanh(action[0]) * i_max_a` | `tanh(action[1]) * i_max_a` |
| action delay | no sign flip | no sign flip |
| smoothing/slew limit | no sign flip unless it crosses through zero dynamically | no sign flip unless it crosses through zero dynamically |
| gain/bias/deadzone/current limit | no fixed sign flip; random bias can offset small commands during training | no fixed sign flip; random bias can offset small commands during training |
| current-loop lag | no sign flip | no sign flip |
| torque conversion | `current_left * Kt` | `current_right * Kt` |
| simulator joint effort | `-torque_left` | `+torque_right` |
| NN wheel velocity sign | `raw_left_velocity * -1` | `raw_right_velocity * +1` |
| robot-forward effect | positive action drives forward | positive action drives forward |

NN action pattern table:

| NN output/action pattern | simulated physical effect | STM32 raw DDSM115 command pattern |
|---|---|---|
| `[+,+]` | forward | `[-,+]` if STM32 raw DDSM sign matches simulator raw joint-effort sign |
| `[-,-]` | backward | `[+,-]` if STM32 raw DDSM sign matches simulator raw joint-effort sign |
| `[+,-]` | clockwise/right yaw, simulation `yaw_rate < 0` | `[-,-]` if STM32 raw DDSM sign matches simulator raw joint-effort sign |
| `[-,+]` | counter-clockwise/left yaw, simulation `yaw_rate > 0` | `[+,+]` if STM32 raw DDSM sign matches simulator raw joint-effort sign |

STM32 command formulas for `policy.pt` or raw actor ONNX:

```c
const float I_MAX_A = 2.0f;

float left_nn_current_A = tanhf(nn_action_0) * I_MAX_A;
float right_nn_current_A = tanhf(nn_action_1) * I_MAX_A;
```

STM32 command formulas for `policy_current.onnx`:

```c
float left_nn_current_A = onnx_output_0;
float right_nn_current_A = onnx_output_1;
```

Raw DDSM115 command formulas to reproduce simulation motor signs:

```c
const float DDSM_RAW_SIGN_LEFT = -1.0f;
const float DDSM_RAW_SIGN_RIGHT = +1.0f;

float I_left_ddsm_raw = DDSM_RAW_SIGN_LEFT * left_nn_current_A;
float I_right_ddsm_raw = DDSM_RAW_SIGN_RIGHT * right_nn_current_A;
```

Important: `obs[6]` and `obs[7]` must store `left_nn_current_A` and `right_nn_current_A`, not `I_left_ddsm_raw` and `I_right_ddsm_raw`.

## 4. Previous Current Mapping

In simulation, previous-current observations are built in `_get_observations` from:

```python
previous_current = self._action_processor.command_current.clone()
```

`command_current` is assigned in `CurrentActionProcessor.process()` after:

```text
raw action -> tanh * i_max -> optional action delay -> smoothing/slew -> gain/bias/deadzone/current limit -> current-loop lag
```

It is before this environment-level motor sign inversion:

```python
self._efforts_buf[:, 0] = -self._wheel_torque_cmd[:, 0]
self._efforts_buf[:, 1] = self._wheel_torque_cmd[:, 1]
```

Therefore:

```text
STM32 obs[6] should be previous left NN-side processed command current in amps.
STM32 obs[7] should be previous right NN-side processed command current in amps.
STM32 obs[6] and obs[7] should not use raw DDSM115 motor-side current after left/right sign mapping.
```

For the simplest deployment path matching default inference without training randomization:

```c
prev_left_command_current_A = left_nn_current_A;
prev_right_command_current_A = right_nn_current_A;

obs[6] = prev_left_command_current_A / 2.0f;
obs[7] = prev_right_command_current_A / 2.0f;
```

If STM32 implements action smoothing, slew limiting, or current-loop lag, store the final NN-side processed current after those filters but before DDSM raw sign mapping.

## 5. BNO/IMU Mapping

Simulation formulas:

```python
pitch = atan2(projected_gravity_b[:, 1], -projected_gravity_b[:, 2])
pitch_rate = -bno080.data.ang_vel_b[:, 0]
yaw_error = wrap_angle_rad(yaw_from_quat_wxyz(root_quat_w) - yaw_reference)
yaw_rate = root_ang_vel_w[:, 2]
```

STM32 formulas matching the trained NN convention:

```c
float wrap_pi(float a) {
    while (a > 3.141592654f) a -= 6.283185307f;
    while (a < -3.141592654f) a += 6.283185307f;
    return a;
}

float theta_rad = atan2f(gravity_robot_y, -gravity_robot_z);
float theta_dot_radps = -gyro_robot_x_radps;

float yaw_rad = yaw_from_bno_or_estimator_rad;
float yaw_error_rad = wrap_pi(yaw_rad - yaw_reference_rad);
float yaw_rate_radps = yaw_rate_sim_sign_radps;

obs[2] = theta_rad / 0.436332313f;
obs[3] = theta_dot_radps / 4.0f;
obs[4] = yaw_error_rad / 3.141592654f;
obs[5] = yaw_rate_radps / 4.0f;
```

Yaw sign required by the NN:

```text
In the trained simulation, counter-clockwise/left yaw is positive.
In the trained simulation, clockwise/right yaw is negative.
```

If the hardware BNO convention uses clockwise-positive yaw, STM32 must invert it before feeding the NN:

```c
float yaw_rate_sim_sign_radps = -yaw_rate_bno_clockwise_positive_radps;
float yaw_sim_sign_rad = -yaw_bno_clockwise_positive_rad;
float yaw_error_rad = wrap_pi(yaw_sim_sign_rad - yaw_reference_rad);
```

Pitch sign required by the NN:

```text
Forward lean is intended to be positive pitch in the observation.
Pitch rate observation is explicitly -gyro_robot_x in the simulation code.
```

## 6. Encoder Mapping

The NN saw wheel encoder signs from `_wheel_sign = [-1.0, 1.0]`.

STM32 formulas:

```c
const float R_WHEEL = 0.05035f;
const float TRACK_WIDTH = 0.382999941707f;

float phi_left_for_nn = -phi_left_raw;
float phi_right_for_nn = +phi_right_raw;
float omega_left_for_nn = -omega_left_raw;
float omega_right_for_nn = +omega_right_raw;

float phi_left_rel_for_nn = phi_left_for_nn - phi_left_ref_for_nn;
float phi_right_rel_for_nn = phi_right_for_nn - phi_right_ref_for_nn;

float x_for_nn = 0.5f * (phi_left_rel_for_nn + phi_right_rel_for_nn) * R_WHEEL;
float x_dot_for_nn = 0.5f * (omega_left_for_nn + omega_right_for_nn) * R_WHEEL;

float yaw_from_wheels_for_debug = ((phi_right_rel_for_nn - phi_left_rel_for_nn) * R_WHEEL) / TRACK_WIDTH;
float yaw_rate_from_wheels_for_debug = ((omega_right_for_nn - omega_left_for_nn) * R_WHEEL) / TRACK_WIDTH;
```

Use wheel-derived yaw only for debug consistency checks. The trained observation uses real yaw and real yaw rate, not wheel-derived yaw.

## 7. DDSM115 Command Mapping

To reproduce the simulation motor sign behavior, STM32 should keep a distinction between NN-side current and raw motor-side current:

```c
float left_nn_current_A;
float right_nn_current_A;

// If using policy.pt/policy.onnx actor output:
left_nn_current_A = tanhf(nn_action_0) * 2.0f;
right_nn_current_A = tanhf(nn_action_1) * 2.0f;

// If using policy_current.onnx, use output directly:
left_nn_current_A = onnx_current_output_0;
right_nn_current_A = onnx_current_output_1;

// Store previous-current observations on the NN side.
prev_left_command_current_A = left_nn_current_A;
prev_right_command_current_A = right_nn_current_A;

// Map to raw DDSM commands with the simulation motor signs.
I_left_ddsm_raw = -left_nn_current_A;
I_right_ddsm_raw = +right_nn_current_A;
```

For the NN's forward command `[+,+]`, the raw DDSM command pattern is opposite signs:

```text
NN-side forward current:  [ +, + ]
STM32 raw DDSM current:   [ -, + ]
```

This is expected because the left simulated wheel torque is inverted before applying joint effort.

## Verification Script

`scripts/determine_nn_sign_convention.py` prints:

- Code-traced action path table.
- Code-traced observation path table.
- Deterministic direct action tests for `[+,+]`, `[+,-]`, `[-,+]`, and `[-,-]`.
- Previous-current observation test showing `obs[6:8]` versus internal current variables.
- Pitch and yaw sign tests.
- Optional trained-policy pitch behavior tests using `--policy`.

Run:

```bash
python scripts/determine_nn_sign_convention.py --headless
```

or explicitly:

```bash
python scripts/determine_nn_sign_convention.py \
  --headless \
  --policy logs/rsl_rl/pure_nn_balance_two_wheel/2026-06-29_13-23-00_stage5/exported/policy.pt
```
