# 2026-11-03 — Payload-mass estimator on top of the hardware balance GRU

Branch `experiment/payload-mass-estimator` (from `feature/mdpi-retrain` @ 3074b62). Sim only;
nothing here has touched hardware.

## Question

Put an object (a book, anything) on the platform at an arbitrary spot while the robot
balances. Can a separate network learn its mass, with accuracy that does not depend on where
it was placed?

## Setup

- **Balance controller, frozen:** the hardware-deployed range+GRU arm C, seed 42,
  `ExportedPolicy/fixedstance_range_gru_2026-08-10/.../model_775.pt`. Rebuilt from the
  checkpoint in `scripts/payload_mass/frozen_policy.py` with a per-env hidden state, because
  the archived `exported/policy.pt` holds a batch-1 hidden buffer. It matches that TorchScript
  export to 0.0 over 300 random steps. It runs on this branch's plant, with the
  firmware-faithful observation semantics (`prev_current_obs_source=command`,
  `obs_delay_mode=sensors`).
- **Payload model** (`nn_drive_payload_env.py`, task `Template-Twowheeledrobot-NNDrivePayload-v0`):
  - The object is a rigid box fused to `Platform_Group` through the PhysX mass/COM/inertia
    tensors, including box inertia and the parallel-axis terms.
  - It is placed at a random time and is optionally removed later.
  - **Rigid mass, not a point force.** A point force gets the gravity torque right but omits
    the object's inertia. 1 kg about 0.18 m above the axle adds roughly the robot's own pitch
    inertia, so a point-force model would teach the estimator the wrong dynamics.
  - The deck outline and height are measured from the USD bound at env construction: deck
    0.227 m lateral × 0.200 m fore/aft, centred over the axle, with its top 0.141 m above the
    platform body origin.
  - `body_mass_scale_range` is narrowed to ±3% (from ±15%) in this env only. The robot's own
    mass is known on hardware (it can be weighed), and ±15% would be ±0.57 kg of label
    ambiguity. ASSUMED.
- **Estimator** (`estimator.py`): a single-layer GRU at the 66.7 Hz control rate, followed
  by Linear(H→32), ReLU, Linear(32→3). It outputs mass (kg) and the deck position (x, y).
  Position is an auxiliary target trained alongside mass. Input standardization is folded
  into the model. With legs, H = 32 and the model has 6,627 parameters (GRU 5,472, head
  1,155), about 26 KB as fp32. That is under a third of the balance GRU's 23,618. Two input
  sets:
  - `imu_wheels`: the policy's own 13 observations plus its 2 wheel-current commands. This is
    only what the firmware already computes.
  - `imu_wheels_legs`: the above plus the 4 CyberGear angles and 4 torque estimates (MIT-mode
    CAN feedback). Sensor model: 14-bit encoder quantization, 0.5 mrad of noise, 0.03 Nm of
    torque noise, ±0.05 Nm per-joint per-episode torque offset. All ASSUMED, no bench data.
- **Data:** 4 rounds × 4096 episodes × 20 s of stage-2 joystick driving, 35% of it pure
  station keeping. Mass is U(0.05, 2.0) kg, absent in 15% of episodes. The object goes on at
  2–6 s and comes off in 25% of episodes. Rounds 0–2 train, round 3 validates.
- **Training:** about 35 s on the GPU. Full episodes alternate with random mid-episode crops
  that start from a zero hidden state.

## Results

### 1. How much can the balance GRU carry? (`results/balance_tolerance.json`)

Fall rate over the deck grid. Fore/aft offsets are ±4/±8 cm, lateral offsets ±9 cm.

| payload | all placements (hold / drive) | centred | notes |
|---|---|---|---|
| ≤ 1.0 kg | 0.0% / 0.8% | 0% | |
| 1.5 kg | 3.3% / 3.3% | 0% | only at ±8 cm fore/aft |
| 2.0 kg | 20% / 20% | 0% | ±8 cm fore/aft falls 25–100% |
| 3.0 kg | 43% / 41% | 0% | all ±8 cm fore/aft cells fall |

A fore/aft offset makes the policy hold a permanent lean, about 1.1 deg per cm at 1 kg and
14 deg at 2 kg / 8 cm. That lean is the failure mode. Lateral placement barely matters.
**The estimator is trained and evaluated on 0–2 kg.**

### 2. Closed-loop estimator accuracy (`results/eval.json`, `figures/`)

Unseen seed. 8 masses (0–2 kg) × 25 deck placements × 6 repeats per mode. The object is on
from 3 s to 33 s. Fallen envs (~3%, heavy loads at the fore/aft edges) are excluded.

| estimator | mode | MAE | rel. err. (loaded) | p95 | placement spread (std of cell means) | worst-cell bias | empty / after removal |
|---|---|---|---|---|---|---|---|
| **imu_wheels_legs, h32 (6.6k params)** | hold | **14 g** | **4.0%** | 40 g | **≤ 15 g** | 63 g (2 kg) | 1 g / 2 g |
| | drive | 15 g | 4.2% | 43 g | ≤ 11 g | 44 g (2 kg) | 1 g / 2 g |
| imu_wheels, h64 (17.7k params) | hold | 133 g | 30% | 392 g | ~100 g | 504 g | 5 g / 42 g |
| | drive | 138 g | 30% | 406 g | ~110 g | 504 g | 5 g / 29 g |

- **The task is solved, and placement-immune, with leg feedback.** The estimate steps to the
  new value within about 0.1 s of placement (sim mass steps are instant; a hand placement
  will be slower) and returns to zero on removal. See `figures/traces_*.png`.
- **The deployed signals alone are not enough.** At equilibrium a balanced robot carries the
  load straight through the wheels with zero net torque, so mass only shows up dynamically.
  The IMU/wheel network detects *that* something was placed but cannot size it, and more
  time does not help. Training it longer at 3× the parameters only took validation error from
  220 g to 120 g.
- Leg channels in sim: about 0.23 Nm and 7.8 mrad per kg per joint, linear, with the
  antisymmetric fl+/fr−/bl−/br+ sign pattern.

### 3. Where is it? Closed-loop location accuracy (`results/eval.json`, `figures/location_*.png`)

The same network also outputs the object's position on the deck: the object COM in x/y,
relative to the deck centre. This comes from the auxiliary offset head, which is trained
alongside mass. It is scored on the same run as section 2: the settled estimate over
[place + 5 s, remove], loaded and upright envs only. For scale, always answering "centre"
would be off by about 5.5 cm on this grid.

| payload | legs: lateral MAE | legs: fore/aft MAE | legs: worst-cell bias | IMU/wheels: lateral | IMU/wheels: fore/aft |
|---|---|---|---|---|---|
| 0.10 kg | 4.3 cm | 3.7 cm | 8.5 cm | 5.5 cm | 2.6 cm |
| 0.25 kg | 1.9 cm | 2.1 cm | 4.1 cm | 5.3 cm | 1.5 cm |
| 0.50 kg | 1.0 cm | 1.0 cm | 2.0 cm | 4.5 cm | 1.0 cm |
| 0.75 kg | 0.7 cm | 0.8 cm | 1.4 cm | 4.0 cm | 0.7 cm |
| 1.0 kg | 0.6 cm | 0.6 cm | 1.0 cm | 3.3 cm | 0.6 cm |
| 1.5 kg | 0.5 cm | 0.5 cm | 0.9 cm | 2.7 cm | 0.5 cm |
| 2.0 kg | 0.5 cm | 0.5 cm | 1.2 cm | 2.9 cm | 0.5 cm |

These figures are for hold mode. Drive mode is within 0.3 cm in every row, except at
0.1 kg, where it is about 0.4 cm better.

- **With leg feedback, an object of 0.5 kg or more is located to about 1 cm in both axes.**
  The four leg torques act like the four load cells of a scale: their sum gives the mass and
  their imbalance gives the position.
- **Error scales roughly with 1/mass.** Position is a load moment divided by the mass, so
  light objects barely move the legs and noise dominates. At 0.1 kg the answer is close to a
  guess; `figures/location_hold.png` shows the scatter.
- **IMU/wheels alone gets fore/aft but not lateral.** A fore/aft offset forces a permanent
  lean, which the IMU and wheels see. A lateral offset only loads one side of the legs, so
  without leg feedback it is nearly invisible. That gap is the 3–5 cm lateral error.
- **What it gives:** the 2D centre of mass of everything on the deck. It does not give the
  height, footprint or shape, and two objects read as one object at their combined COM.
- **Untested:** location with the object already on at power-on, and under the 3× leg-sensor
  errors. Location depends on *differences* between the four joints, so per-joint torque
  offsets and gearbox hysteresis should hurt it more than they hurt mass.

### 4. Robustness of the leg estimator (3 reps per cell, `results/eval_*.{json,log}`)

| test | MAE | rel. err. | note |
|---|---|---|---|
| nominal | 14 g | 4.0% | |
| objects 1.75× larger (up to 14 cm tall) | 30 g | 7% | higher COM; out of training range |
| leg torque offset ±0.15 Nm, noise 0.09 Nm, angle noise 1.5 mrad (3× trained) | 46 g | 15% | empty deck reads 30 g |
| **object already on at power-on** | **78 g** | **29%** | 0.1 kg reads as 0.01, 0.25 as 0.09; ≥0.75 kg within ~5% |

**The estimator is effectively taring.** It needs to see the empty deck once, because a
±0.05 Nm per-joint torque offset is about ±0.2 kg-equivalent and cannot be told apart from
load without a reference. This is an information limit, not a training bug: the training
data already includes mid-episode starts with the object on. On hardware this means arming
with an empty deck, or adding an explicit tare.

## Hardware risks, in order

1. **CyberGear torque feedback realism.** Sim legs are ideal PD joints. The real 7.75:1
   planetary gearbox has stiction and hysteresis, which may be comparable to the
   0.23 Nm/kg signal.
   - *Decisive 10-minute bench check:* on the robot, logging only CyberGear torque and angle,
     put known masses (0, 0.25, 0.5, 1, 2 kg) on the centre and the corners, a few times
     each. Look for a linear, repeatable reading.
   - If the reading is linear and repeatable, deploy. If it is hysteretic, add that to
     `leg_features` and retrain (35 s).
2. **The firmware must read CyberGear angle and torque feedback** every tick. I have not
   checked whether the fixed-stance firmware already does.
3. The tare requirement from section 4.

## Reproduce

```bash
# in the isaac-lab-dev container, from /workspace/TwoWheeledRobot
/isaac-sim/python.sh scripts/payload_mass/sweep_balance_tolerance.py --headless
/isaac-sim/python.sh scripts/payload_mass/collect_rollouts.py --headless --rounds 4 --seed 100
/isaac-sim/python.sh scripts/payload_mass/train_estimator.py --features imu_wheels_legs --hidden 32
/isaac-sim/python.sh scripts/payload_mass/train_estimator.py --features imu_wheels --hidden 64 --iters 12000 --crop 900
/isaac-sim/python.sh scripts/payload_mass/evaluate_estimator.py --headless \
  --estimators outputs/payload_mass/estimator_imu_wheels_legs_h32.pt outputs/payload_mass/estimator_imu_wheels_h64.pt
/isaac-sim/python.sh scripts/payload_mass/export_estimator_onnx.py --estimator outputs/payload_mass/estimator_imu_wheels_legs_h32.pt
```

The robustness runs add `--place_s 0.0`, `--size_scale 1.75`, or
`--cfg 'leg_torque_bias_nm_range=(-0.15,0.15)' leg_torque_noise_std_nm=0.09 leg_pos_noise_std_rad=0.0015`.

## MCU contract for the ONNX (`model/estimator_imu_wheels_legs_h32.onnx`)

- **Inputs:**
  - `features` (1, 23), in raw units, in this order:
    - the 13 normalized policy observations, exactly as fed to the balance GRU;
    - the 2 wheel-current commands (A), logical left/right;
    - 4 CyberGear angles (rad), fl fr bl br;
    - 4 CyberGear torques (Nm), fl fr bl br.
  - `h_in` (1, 1, 32).
- **Outputs:** `mass_kg` (1, 1), `offset_xy_m` (1, 2) and `h_out`. `offset_xy_m` is the
  object position relative to the deck centre in the sim body frame: lateral (x), then
  fore/aft (y, forward = -y). Ignore it below about 0.25 kg (section 3). Feed `h_out` back each tick
  and zero it on arm. The graph matches PyTorch to 2e-7.
- **Caveat:** the leg sign and zero conventions are the sim's. They must be checked against
  the CAN feedback before use.
