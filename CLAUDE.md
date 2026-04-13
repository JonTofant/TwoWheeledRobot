# Two-Wheeled Robot — Project Context

## What This Is

A research project targeting academic publication. The goal is to develop and validate control algorithms (Sliding Mode Control, LQR, and eventually RL policies) for a custom two-wheeled inverted pendulum robot, using NVIDIA Isaac Lab as the simulation testbed, with the intent of deploying to a physical robot.

## Hardware

A physical robot has been built. The C firmware (`RealImplementationCode/`) is functional but policies/controllers have not yet been transferred from simulation to hardware. The simulation is the active development environment.

**Robot hardware:**
- **Wheels:** DDSM115 current-controlled motors, ±8 A range, Kt = 0.75 Nm/A
- **Legs:** 4× Xiaomi CyberGear motors in MIT mode (kp/kd position+velocity+torque feedforward)
- **Sensor:** IMU (gravity projection + angular velocity in body frame)
- **Wheel radius:** 0.0505 m (calibrated)

**Left wheel USD mesh is mirrored** — negate left current/torque to get consistent forward = positive convention. This is handled in `control.py`'s translation layer; do not change that negation.

## Software Stack

- **Framework:** NVIDIA Isaac Lab (GPU-accelerated PhysX simulation)
- **Physics:** 200 Hz, GPU-parallelized environments
- **RL frameworks available:** RSL RL, SKRL, RL Games, Stable Baselines 3
- **Language:** Python 3.11, PyTorch tensors throughout

## Current Development Focus

Three active areas of work:

1. **Tuning SMC gains** — The sliding mode controller balance/velocity/heading parameters in `control.py` are the primary tuning target. See gain table below.
2. **Leg / posture control** — CyberGear legs currently hold a fixed upright stance (`desired_angle = 0.0`). Active leg control (crouching, posture adaptation) is next.
3. **Sim-to-real transfer** — Bridging the gap between Isaac Lab simulation and the physical hardware. The control law in `control.py` is designed to mirror the C firmware exactly (same variable names, same conventions).

## The Main File: `control.py`

**Path:** `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/control.py`

This is the primary file to edit. It contains:
- `RobotController` class with `compute()` called every physics step
- Full SMC implementation (balance + heading surfaces)
- CyberGear stance offset logic (via `kinematics.cybergear_stance_angles`)
- A `last` dict populated each step for the real-time plotter

**Run the simulation:**
```bash
# From inside the TwoWheeledRobot/ directory:
python scripts/zero_agent.py --task Template-Twowheeledrobot-Direct-v0
# If no GUI: run `xhost +local:docker` on the host first
```

## Control Algorithm: Sliding Mode Control (SMC)

Two coupled surfaces share one control loop:

### Balance + Velocity Surface (common mode — equal current to both wheels)
```
s_bal = θ + SMC_ALPHA·θ̇ + SMC_BETA·(v_cmd − v)
u_bal = SMC_K · tanh(s_bal / SMC_PHI)    [A]
```

### Heading Surface (differential mode — yaw via current difference)
```
ψ̂  = ∫ ψ̇ dt           (integrated yaw from IMU)
s_yaw = e_heading + SMC_YAW_LAMBDA·(ψ̇_cmd − ψ̇)
u_yaw = SMC_YAW_K · tanh(s_yaw / SMC_YAW_PHI)   [A]
```

### Wheel Currents
```
current_left  = u_bal − u_yaw
current_right = u_bal + u_yaw
```

### Heading-Hold Logic
While `|yaw_rate_cmd| > 0`, `_heading_cmd` advances at `yaw_rate_cmd` → `e_heading ≈ 0` (rate command).
When the turn key is released, `_heading_cmd` freezes → compass/heading-hold engages automatically.

### Current SMC Gains
| Parameter | Value | Meaning |
|-----------|-------|---------|
| `SMC_ALPHA` | 0.08 s | Tilt-rate weight (~pendulum natural period / 2π) |
| `SMC_BETA` | 0.60 s/m | Velocity-error weight |
| `SMC_K` | 1.3 A | Max current per wheel |
| `SMC_PHI` | 0.25 rad | Boundary-layer width (keeps tanh linear near equilibrium) |
| `SMC_YAW_LAMBDA` | 0.30 s | Yaw-rate weight in heading surface |
| `SMC_YAW_K` | 0.6 A | Differential gain |
| `SMC_YAW_PHI` | 0.20 rad | Boundary-layer for heading surface |

## Project Structure

```
TwoWheeledRobot/
├── CLAUDE.md                          ← you are here
├── README.md
├── compute_lqr_gains.py               # Offline LQR gain design (scipy CARE solver)
├── source/TwoWheeledRobot/TwoWheeledRobot/tasks/
│   ├── direct/twowheeledrobot/
│   │   ├── control.py                 # ← PRIMARY FILE: SMC controller
│   │   ├── kinematics.py              # CyberGear stance angle computation
│   │   ├── robot_cfg.py               # USD spawn + actuator config
│   │   ├── sim_params.py              # PhysX parameters
│   │   └── twowheeledrobot_env.py     # Isaac Lab direct env (calls control.py)
│   └── manager_based/twowheeledrobot/ # Alternative RL manager-based env
├── RealImplementationCode/            # C firmware reference (mirrors control.py)
├── scripts/
│   ├── zero_agent.py                  # Run sim with control.py (no RL)
│   ├── rsl_rl/{train,play}.py
│   ├── skrl/{train,play}.py
│   └── sb3/{train,play}.py
└── docs/ColectedUSD_v2/               # Robot USD model files
```

## Firmware Correspondence

`control.py` is designed to be a 1:1 translation of the C firmware:
- `DDSM115setCurrent()` → `current_left / current_right` [A]
- `motor->desired_angle` → `desired_angle` [rad], 0 = upright after `setMechanicalZero()`
- CyberGear kp/kd values match `cybergear.c` `CyberGearMotorList` initialization

## IMU Axis Mapping (confirmed from live debug)
```python
IMU_TILT_GRAVITY_AXIS = 1   # projected_gravity_b[:, 1] — forward tilt
IMU_TILT_RATE_AXIS    = 1   # ang_vel_b[:, 1]           — tilt rate
IMU_YAW_RATE_AXIS     = 2   # ang_vel_b[:, 2]           — yaw rate
```
