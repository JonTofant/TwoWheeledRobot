# Two-Wheeled Robot — Control Research Platform

A research platform for developing and validating control algorithms on a custom **two-wheeled inverted pendulum robot**. The simulation runs in NVIDIA Isaac Lab (GPU-accelerated PhysX). Validated controllers deploy directly to the physical robot via C firmware that mirrors the Python control code exactly.

---

## What This Project Does

The robot is inherently unstable — like a pencil balanced on its tip. To stay upright it must continuously lean into its direction of motion, using wheel torque to counteract falling.

Three control layers stack on top of each other:

```
┌─────────────────────────────────────────────────────────────┐
│                3. RL Gain Scheduler  (optional)              │
│   Watches robot state + trigger diagnostics, adjusts the     │
│   four most impactful gains in real-time via a trained       │
│   neural network  [32 → 32 → 4 outputs]                     │
│   Goal: maximise inter-event time while maintaining balance  │
└────────────────────────┬────────────────────────────────────┘
                         │ T_min_bal, T_min_yaw, K_max_bal, K_max_yaw
┌────────────────────────▼────────────────────────────────────┐
│         2. PETASMC Controller  (core balance law)            │
│                                                              │
│   Sensors ──► Sliding surfaces  s_bal, s_yaw                │
│                      │                                       │
│           ┌──────────▼──────────┐                           │
│           │  Periodic trigger?  │                           │
│           │  t_since ≥ T_min    │  ← floor rate (default   │
│           │  AND |Δs| > η|s̃|+ε │    20 ms = 50 Hz)        │
│           └──────┬──────┬───────┘                           │
│             YES  │      │  NO                               │
│          compute │      │ hold last output (ZOH)            │
│          new u   │      │                                    │
│           └──────▼──────┘                                    │
│           current_L = u_bal − u_yaw                         │
│           current_R = u_bal + u_yaw                         │
└────────────────────────┬────────────────────────────────────┘
                         │ wheel torque = current × Kt
┌────────────────────────▼────────────────────────────────────┐
│                1. Physical Robot                             │
│   2× DDSM115 wheel motors   4× CyberGear leg motors         │
│   IMU (tilt + yaw rate)      Wheel encoders (speed + angle) │
└─────────────────────────────────────────────────────────────┘
```

---

## Hardware

```
           ┌──────────────────┐
           │  Platform + IMU  │
      ┌────┤  Battery + MCU   ├────┐
      │ CG │                  │ CG │  ← 4× CyberGear legs (MIT mode)
      └────┤                  ├────┘     hold upright stance
           └────────┬─────────┘
      ○─────────────┴─────────────○
    Left                         Right
   DDSM115                     DDSM115
  (current)                   (current)
```

| Component | Details |
|-----------|---------|
| Wheels | 2× DDSM115, current-controlled, ±8 A, Kt = 0.75 Nm/A |
| Legs | 4× Xiaomi CyberGear in MIT mode (kp + kd + torque FF) |
| IMU | BNO080 — gravity projection + 3-axis angular velocity |
| Wheel radius | 0.0505 m (calibrated) |

> **Left wheel note:** The USD mesh is mirrored. The translation layer in `control.py` negates the left wheel current so forward = positive on both sides. Do not change this.

---

## Control Algorithm: PETASMC

**Periodic Event-Triggered Adaptive Sliding Mode Control**

The algorithm answers a key hardware question: *how often does the microcontroller actually need to write a new current command to the motors?* Fixed-rate control wastes CAN bus bandwidth at equilibrium. Pure event-triggered control can burst to maximum rate during a disturbance. PETASMC combines both:

- **Periodic floor** — never updates faster than `T_min` (default 20 ms = 50 Hz). This matches the prior fixed-rate tuning and bounds worst-case CAN traffic.
- **Event trigger** — can update early (within the window) if the surface changes enough. An impulse at t = 7 ms fires at t = 7 ms, not t = 20 ms.
- **Adaptive gain** — scales current with how far the robot is from equilibrium. Gentle near upright, strong during recovery.

### Sliding Surfaces

The controller watches two error surfaces. When a surface grows, the robot is moving away from the desired state.

**Balance surface** — drives tilt, velocity, and position to zero:
```
s_bal = θ  +  α·θ̇  +  β·(v_cmd − v)  +  γ·(x_cmd − x_odom)
         ↑      ↑              ↑                   ↑
       tilt  tilt rate    velocity error      position error
```

**Heading surface** — drives yaw to the commanded angle:
```
s_yaw = (ψ_cmd − ψ)  +  λ·(ψ̇_cmd − ψ̇)
              ↑                  ↑
         heading error      yaw rate error
```

Both set-points use **implicit hold**: while a key is held the set-point advances (error ≈ 0, rate-following mode). Releasing the key freezes the set-point and hold engages automatically — no mode switch needed.

### Adaptive Gain

```
K(|s|) = K_min + (K_max − K_min) · |s| / (|s| + σ)
```

| Situation | |s| | Gain |
|-----------|-----|------|
| Near upright | ≈ 0 | → K_min (gentle, no chattering) |
| Moderate lean | ~ σ | midpoint |
| Falling | → ∞ | → K_max (maximum recovery) |

### Periodic Event Trigger

```
can_trigger  = time_since_last ≥ T_min          ← periodic floor
event_fires  = |s(t) − s̃| > η·|s̃| + ε        ← surface threshold
trigger      = can_trigger  AND  event_fires
```

`s̃` is the surface value captured at the last trigger. `ε > 0` guarantees a minimum inter-event time (Zeno-free: `T_min ≥ ε / max|ṡ|`).

| Condition | Trigger rate | Meaning |
|-----------|-------------|---------|
| Quiet, balanced | 1 / T_min = 50 Hz max | CAN traffic bounded |
| Active disturbance | up to 200 Hz (sensor rate) | Fast reaction within window |

### Wheel Currents

```
u_bal = K_bal(|s_bal|) · tanh(s_bal / φ)      [A]
u_yaw = K_yaw(|s_yaw|) · tanh(s_yaw / φ_yaw)  [A]

current_left  = u_bal − u_yaw     →  torque_left  = current × Kt
current_right = u_bal + u_yaw     →  torque_right = current × Kt
```

---

## RL Gain Scheduler

Training a PPO policy to schedule the four most impactful PETASMC gains in real-time. The PETASMC structure is preserved — the RL policy only adjusts parameters, it does not replace the control law.

**Why this is interesting academically:**
- The SMC surfaces provide Lyapunov stability guarantees regardless of gain values
- The RL provides *optimal* gain scheduling that a human cannot tune manually
- The result runs at 50 Hz on an STM32F446RE (network: ~1 540 float32 weights ≈ 6 KB)

**Observation (10 dims):**

| # | Signal | Description |
|---|--------|-------------|
| 0 | θ | Tilt angle (rad) |
| 1 | θ̇ | Tilt rate (rad/s) |
| 2 | ψ̇ | Yaw rate (rad/s) |
| 3 | v | Forward wheel velocity (m/s) |
| 4 | v_cmd | Commanded velocity (m/s) |
| 5 | s_bal | Balance surface value |
| 6 | s_yaw | Heading surface value |
| 7 | IET_bal / T_min_bal | How sparse the balance trigger is (1 = at floor) |
| 8 | IET_yaw / T_min_yaw | Same for heading |
| 9 | T_min_bal (normalised) | Current balance floor setting |

**Action (4 dims, mapped from [-1, 1]):**

| # | Gain | Range |
|---|------|-------|
| 0 | T_min_bal | 5 – 40 ms |
| 1 | T_min_yaw | 5 – 40 ms |
| 2 | K_max_bal | 0.8 – 2.5 A |
| 3 | K_max_yaw | 0.05 – 0.30 A |

**Reward:**
```
r = 2.0 · exp(−θ²/0.01)          balance quality
  + 1.0 · exp(−(v−v_cmd)²/0.25)  velocity tracking
  + 0.1 · (1 − trig_bal)         sparsity bonus per held step
  + 0.5                           alive bonus
  − 200   (once, at fall)         termination penalty
```

**Domain randomisation per episode:** ±15% mass, random initial tilt ±2.3°, random velocity command ±0.3 m/s, random push impulses every ~4 s, IMU noise on all observations.

---

## Running

### Manual control (keyboard teleop)
```bash
# From TwoWheeledRobot/
python scripts/zero_agent.py --task Template-Twowheeledrobot-Direct-v0

# If no GUI (Docker): run this on the host first
xhost +local:docker
```
**W/S** = forward / back · **A/D** = turn left / right

The live plotter and SMC tuner open automatically.

### Train the RL gain scheduler
```bash
python scripts/rsl_rl/train.py \
    --task Template-Twowheeledrobot-Petasmc-v0 \
    --headless --num_envs 2048
```
Logs saved to `logs/rsl_rl/petasmc_gain_scheduler/`.

### Play back a trained policy
```bash
python scripts/rsl_rl/play.py \
    --task Template-Twowheeledrobot-Petasmc-v0 \
    --num_envs 1
```

---

## Tuning Reference

All tuneable parameters are constants at the top of `control.py` and can also be adjusted live via the SMC Tuner panel.

### Balance Surface

| Parameter | Value | Effect |
|-----------|-------|--------|
| `SMC_ALPHA` | 0.08 s | Tilt-rate weight. Increase → more damping. Too high → sluggish / hunting. |
| `SMC_BETA` | 0.60 s/m | Velocity-error weight. Increase → tighter speed tracking, steeper lean. |
| `SMC_PHI` | 0.20 rad | tanh boundary-layer width. Increase → smoother but slower near equilibrium. |
| `ET_GAMMA` | 0.00 m⁻¹ | Position-hold weight (odometry). **Disabled** — enable only after balance + yaw are stable. |

### Balance Adaptive Gain

| Parameter | Value | Effect |
|-----------|-------|--------|
| `ET_K_MAX_BAL` | 1.6 A | Maximum recovery current. Increase if robot cannot recover from large tilts. |
| `ET_K_MIN_BAL` | 0.70 A | Minimum holding current. Increase if robot drifts without converging. |
| `ET_K_SIGMA_BAL` | 0.15 rad | Half-saturation point. Decrease → gain rises faster from K_min. |

### Heading Surface

| Parameter | Value | Effect |
|-----------|-------|--------|
| `SMC_YAW_LAMBDA` | 0.15 s | Yaw-rate weight. Was 0.30 — caused yaw torque to dominate balance at moderate spin. |
| `SMC_YAW_PHI` | 0.20 rad | Heading boundary-layer width. |

### Heading Adaptive Gain

| Parameter | Value | Effect |
|-----------|-------|--------|
| `ET_K_MAX_YAW` | 0.15 A | Max differential current. Must stay well below `ET_K_MIN_BAL` or balance loses. |
| `ET_K_MIN_YAW` | 0.02 A | Minimum heading correction. |
| `ET_K_SIGMA_YAW` | 0.10 rad | Half-saturation point for yaw gain. |

### Event Trigger Thresholds

| Parameter | Value | Effect |
|-----------|-------|--------|
| `ET_ETA_BAL` | 0.20 | Relative balance threshold. Decrease → more frequent triggers. |
| `ET_EPS_BAL` | 0.008 rad | Absolute floor. Must be above IMU noise (~0.003 rad). |
| `ET_ETA_YAW` | 0.05 | Near-continuous heading trigger. Low to prevent ZOH overshoot oscillation. |
| `ET_EPS_YAW` | 0.003 rad | Absolute floor for heading. |

### Periodic Floor (PETASMC)

| Parameter | Value | Effect |
|-----------|-------|--------|
| `ET_MIN_IET_BAL` | 0.020 s | Minimum time between balance updates (50 Hz floor). |
| `ET_MIN_IET_YAW` | 0.020 s | Minimum time between heading updates. |

---

## Diagnostics Plotter

The live plotter opens automatically during manual control. Panels:

| Panel | What to look for |
|-------|-----------------|
| Tilt | Settling to 0°, no sustained oscillation |
| Tilt rate | Small and quiet at equilibrium |
| Yaw rate | Near 0 at equilibrium |
| Torques | Symmetric (τ_L ≈ τ_R) and small at equilibrium |
| Velocity | vel_fwd tracks vel_cmd |
| Surfaces | s_bal, s_yaw converging toward 0 |
| Triggers | Dense near disturbance, sparse at equilibrium |
| IET | Grows at equilibrium — confirms self-quiescence |
| Position | Returns to 0 when vel_cmd = 0 (after enabling ET_GAMMA) |

---

## Key Invariants

- `ET_K_MAX_YAW` ≪ `ET_K_MIN_BAL` — yaw torque must be a small fraction of balance budget
- `ET_EPS_BAL` > 3 × IMU noise — prevents noise from causing Zeno triggering
- Left wheel torque sign negated in translation layer — matches mirrored USD mesh
- CyberGear legs initialised to IK equilibrium at reset — prevents violent jump on first step

---

## Project Structure

```
TwoWheeledRobot/
├── README.md
├── CLAUDE.md                          — AI assistant context
├── compute_lqr_gains.py               — Offline LQR gain design (scipy CARE)
├── source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/
│   ├── control.py                     ← PETASMC controller (primary file)
│   ├── petasmc_rl_env.py              ← RL gain-scheduling environment
│   ├── petasmc_rl_env_cfg.py          ← RL env configuration
│   ├── twowheeledrobot_env.py         ← Manual control environment
│   ├── twowheeledrobot_env_cfg.py
│   ├── kinematics.py                  — CyberGear stance angle IK
│   ├── plotter.py                     — Live diagnostic plotter
│   ├── tuner.py                       — Live SMC parameter tuner
│   ├── robot_cfg.py                   — USD spawn + actuator config
│   ├── sim_params.py                  — PhysX parameters
│   └── agents/
│       ├── rsl_rl_ppo_cfg.py          — PPO config (manual env)
│       └── rsl_rl_petasmc_cfg.py      — PPO config (RL gain scheduler)
├── RealImplementationCode/            — C firmware (mirrors control.py exactly)
└── scripts/
    ├── zero_agent.py                  — Run manual control sim
    └── rsl_rl/, skrl/, sb3/           — RL training + playback scripts
```

---

## Firmware Correspondence

`control.py` is designed to translate 1:1 to the C firmware:

| Python | C firmware |
|--------|-----------|
| `current_left / current_right` | `DDSM115setCurrent()` |
| `desired_angle` | `motor->desired_angle` (after `setMechanicalZero()`) |
| CyberGear kp / kd | `CyberGearMotorList` init values in `cybergear.c` |
| IMU axes 1, 1, 2 | Confirmed from live hardware debug |
