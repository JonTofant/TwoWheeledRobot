# Two-Wheeled Robot — Isaac Lab Control Research

A research platform for developing and validating control algorithms for a custom two-wheeled inverted-pendulum robot. Simulation runs in NVIDIA Isaac Lab (GPU-accelerated PhysX). Target: deploy validated controllers to the physical robot.

---

## Hardware

| Component | Details |
|-----------|---------|
| Wheels | 2× DDSM115 current-controlled motors, ±8 A, Kt = 0.75 Nm/A |
| Legs | 4× Xiaomi CyberGear in MIT mode (kp/kd + torque feedforward) |
| IMU | Gravity projection + 3-axis angular velocity in body frame |
| Wheel radius | 0.0505 m (calibrated) |

**Left wheel USD mesh is mirrored** — the translation layer in `control.py` negates the left current to get forward = positive convention. Do not change that negation.

---

## Running the Simulation

```bash
# From the TwoWheeledRobot/ directory:
python scripts/zero_agent.py --task Template-Twowheeledrobot-Direct-v0

# If no GUI appears (Docker):
xhost +local:docker   # run on the host first
```

Keyboard teleop: **W/S** = forward/back, **A/D** = turn left/right.

---

## Control Algorithm: Event-Triggered Adaptive SMC (ETASMC)

All control logic lives in `control.py`. The algorithm has three coupled layers:

### 1. Sliding Surfaces

**Balance + velocity + position (common mode — equal current to both wheels):**
```
x_odom  = 0.5 · R · (−φ_L + φ_R)          wheel odometry, direct encoder read
x_cmd   advances at vel_cmd · dt            freezes when vel_cmd = 0 → position-hold
s_bal = θ + α·θ̇ + β·(v_cmd − v) + γ·(x_cmd − x_odom)
```

**Heading (differential mode — yaw torque via wheel current difference):**
```
ψ̂  = ∫ ψ̇ dt       simulated compass from IMU yaw rate
s_yaw = e_heading + λ·(ψ̇_cmd − ψ̇)
```

Both set-points use implicit hold logic: while commanding motion the set-point advances (error ≈ 0); releasing the command freezes it and hold engages automatically.

### 2. Adaptive Gain

```
K(|s|) = K_min + (K_max − K_min) · |s| / (|s| + σ)
```
- Near equilibrium (|s| → 0): gain → K_min (gentle, no chattering)
- Large disturbance (|s| → ∞): gain → K_max (fast recovery)
- Continuous and monotone — no switched-gain discontinuities

### 3. Event Trigger (zero-order hold)

```
trigger = |s(t) − s̃| > η · |s̃| + ε
```
Where `s̃` is the surface value at the last trigger. Between triggers the last computed output is held (ZOH). Near equilibrium, |s| changes slowly → fewer triggers (**self-quiescence**). The `ε > 0` term guarantees a positive minimum inter-event time (**Zeno-free**: T_min ≥ ε / max|ṡ|).

### Wheel Currents

```
u_bal = K_bal(|s_bal|) · tanh(s_bal / SMC_PHI)       [A]  — updated on trigger
u_yaw = K_yaw(|s_yaw|) · tanh(s_yaw / SMC_YAW_PHI)  [A]  — updated on trigger

current_left  = u_bal − u_yaw
current_right = u_bal + u_yaw
torque = current × Kt = current × 0.75 Nm/A
```

---

## Tuning Reference

All tuneable parameters are constants at the top of `control.py`. The table below describes which state each parameter primarily affects and what changing it does.

### Balance Surface Shape

| Parameter | Current value | Effect |
|-----------|--------------|--------|
| `SMC_ALPHA` | 0.08 s | Weight of **tilt rate** θ̇ in s_bal. Increase → more damping on tilt oscillations. Too high → sluggish, can cause hunting. |
| `SMC_BETA` | 0.60 s/m | Weight of **velocity error** in s_bal. Increase → tighter velocity tracking, steeper required lean angle. Too high → robot runs to chase velocity set-point. |
| `SMC_PHI` | 0.20 rad | **Boundary-layer width** for balance tanh. Increase → smoother control near equilibrium (less chattering) but slower response. Decrease → snappier but more chattering. |
| `ET_GAMMA` | 0.00 m⁻¹ | Weight of **position error** (wheel odometry) in s_bal. Currently disabled. Enable (try 0.05) only after balance + yaw are stable; spinning corrupts x_odom. |

### Balance Adaptive Gain

| Parameter | Current value | Effect |
|-----------|--------------|--------|
| `ET_K_MAX_BAL` | 1.6 A | **Maximum** wheel current for balance. Upper bound on recovery authority. Increase if robot cannot recover from large tilts. Hardware limit: 8 A. |
| `ET_K_MIN_BAL` | 0.70 A | **Minimum** wheel current near equilibrium. Must be large enough to hold upright against gravity + CoM offset. Increase if robot drifts without converging. |
| `ET_K_SIGMA_BAL` | 0.15 rad | Surface magnitude at which gain is halfway between K_min and K_max. Decrease → gain rises faster from K_min; increase → wider gentle region. |

### Heading Surface Shape

| Parameter | Current value | Effect |
|-----------|--------------|--------|
| `SMC_YAW_LAMBDA` | 0.15 s | Weight of **yaw rate error** ψ̇ in s_yaw. Increase → more damping on yaw oscillations. Was 0.30 — caused yaw torque to dominate balance at moderate spin rates. |
| `SMC_YAW_PHI` | 0.20 rad | **Boundary-layer width** for heading tanh. Same trade-off as SMC_PHI but for yaw. |

### Heading Adaptive Gain

| Parameter | Current value | Effect |
|-----------|--------------|--------|
| `ET_K_MAX_YAW` | 0.15 A | **Maximum differential current** for heading correction. Critical: keep well below ET_K_MIN_BAL so balance always dominates the torque budget. Was 0.50 A — starved balance. |
| `ET_K_MIN_YAW` | 0.02 A | Minimum heading correction near equilibrium. |
| `ET_K_SIGMA_YAW` | 0.10 rad | Half-saturation point for yaw gain schedule. |

### Event Trigger Thresholds

| Parameter | Current value | Effect |
|-----------|--------------|--------|
| `ET_ETA_BAL` | 0.20 | **Relative** balance trigger threshold. Decrease → more frequent balance events (approaches fixed-rate at 0). Increase → sparser updates, more ZOH hold time. |
| `ET_EPS_BAL` | 0.008 rad | **Absolute** floor for balance trigger. Must be above IMU noise (~0.003 rad). Governs minimum inter-event time (Zeno guarantee). |
| `ET_ETA_YAW` | 0.05 | Relative heading trigger threshold. Set low (near-continuous) to prevent ZOH overshoot oscillation in the fast yaw channel. |
| `ET_EPS_YAW` | 0.003 rad | Absolute floor for heading trigger. |

---

## Tuning Procedure

Work through these stages in order. Do not move to the next stage until the current one is stable.

### Stage 1 — Balance only (ET_GAMMA = 0, ignore yaw drift)

Goal: robot stands upright, tilt ≈ 0°, zero forward drift.

Watch in plotter: **tilt (deg)** and **torque L/R**.

1. If robot falls forward/backward within 2 s: increase `ET_K_MIN_BAL` (0.70 → 0.90 A).
2. If torques chatter at high frequency near equilibrium: increase `SMC_PHI` (0.20 → 0.30 rad) or decrease `ET_K_MIN_BAL`.
3. If robot oscillates (tilt swings ±5° repeatedly): decrease `SMC_ALPHA` (0.08 → 0.05 s) or decrease `ET_K_MAX_BAL`.
4. If robot drifts forward/back with no oscillation: this is a CoM offset. Acceptable at this stage.
5. Verify `tau_L ≈ tau_R` (symmetric torques). Large asymmetry means yaw dominates — reduce `ET_K_MAX_YAW` further.

### Stage 2 — Heading stability

Goal: robot stands upright AND does not spin.

Watch in plotter: **yaw_rate**, **tau_L vs tau_R**, **s_yaw**.

1. If robot spins slowly and never corrects: check sign of yaw correction (see firmware notes below).
2. If yaw oscillates ±1 rad/s: `ET_K_MAX_YAW` is still too high — reduce it.
3. If yaw drift is slow but acceptable: robot is near stable. Proceed to stage 3.
4. Target: `|tau_L − tau_R| < 0.2 Nm` at steady state.

### Stage 3 — Position hold

Goal: robot returns to starting position after being pushed.

1. Set `ET_GAMMA = 0.05` m⁻¹.
2. Push robot ~0.2 m forward; watch `x_odom` in plotter — should return toward 0.
3. If balance destabilises: reduce `ET_GAMMA` (0.05 → 0.02).
4. If robot oscillates in position: increase `SMC_BETA` slightly or reduce `ET_GAMMA`.

### Stage 4 — Event rate verification

Goal: confirm self-quiescence (events slow down at equilibrium).

Watch in plotter: **IET_bal**, **IET_yaw**.

- At equilibrium: IET_bal should be > 100 ms (vs 20 ms fixed-rate baseline).
- During disturbance: IET_bal should drop toward 20–40 ms.
- IET should never be < 2 × dt = 40 ms (Zeno check).

---

## Diagnostics Plotter

The live plotter opens automatically (`enabled=True` in `twowheeledrobot_env.py`). Panels:

| Panel | Signals | What to look for |
|-------|---------|-----------------|
| Tilt | tilt (deg), theta_des | Settling to 0°, no sustained oscillation |
| Tilt rate | θ̇ (rad/s) | Should be small and quiet at equilibrium |
| Yaw rate | ψ̇ (rad/s) | Should be near 0 at equilibrium |
| Torques | τ_L, τ_R (Nm) | Should be symmetric and small at equilibrium |
| Velocity | vel_cmd, vel_fwd (m/s) | vel_fwd should track vel_cmd and return to 0 |
| Surfaces | s_bal, s_yaw (rad) | Should converge toward 0 |
| Triggers | trig_bal, trig_yaw | Dense near disturbance, sparse at equilibrium |
| IET | IET_bal, IET_yaw (s) | Increases at equilibrium (self-quiescence) |
| Position | x_odom (m) | Returns to 0 when vel_cmd = 0 (Stage 3+) |

---

## Key Invariants (do not violate)

- `ET_K_MAX_YAW << ET_K_MIN_BAL` — yaw torque budget must be a small fraction of balance
- `ET_EPS_BAL > 3 × IMU_noise` — prevents Zeno triggering from sensor noise
- Left wheel torque sign negation in the translation layer — matches mirrored USD mesh convention
- CyberGear legs initialized to IK equilibrium at reset — prevents violent jump on first step

---

## Project Structure

```
TwoWheeledRobot/
├── README.md
├── CLAUDE.md                          — AI assistant context
├── compute_lqr_gains.py               — Offline LQR gain design (scipy CARE)
├── source/TwoWheeledRobot/TwoWheeledRobot/tasks/
│   └── direct/twowheeledrobot/
│       ├── control.py                 ← PRIMARY: ETASMC controller
│       ├── kinematics.py              — CyberGear stance angle IK
│       ├── plotter.py                 — Isaac Lab live diagnostic plotter
│       ├── robot_cfg.py               — USD spawn + actuator config
│       ├── sim_params.py              — PhysX parameters (friction, CoM offsets)
│       └── twowheeledrobot_env.py     — Isaac Lab direct env (calls control.py)
├── RealImplementationCode/            — C firmware reference (mirrors control.py)
└── scripts/
    ├── zero_agent.py                  — Run sim with control.py (no RL)
    └── rsl_rl/, skrl/, sb3/           — RL training/inference scripts
```

---

## IMU Axis Convention

```python
IMU_TILT_GRAVITY_AXIS = 1   # projected_gravity_b[:, 1] — forward tilt (positive = leaning forward)
IMU_TILT_RATE_AXIS    = 1   # ang_vel_b[:, 1]           — tilt rate
IMU_YAW_RATE_AXIS     = 2   # ang_vel_b[:, 2]           — yaw rate (positive = CCW from above)
```
