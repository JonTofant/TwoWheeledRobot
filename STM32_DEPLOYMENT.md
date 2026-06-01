# STM32 Deployment Notes

These notes describe the cleaned stand-up policy interface. The Isaac Lab task is:

```text
Template-Twowheeledrobot-Standup-v0
```

## Policy Contract

The trained actor takes 18 normalized `float32` observations and returns 6 actions in `[-1, 1]`.

Observations:

```text
0..2    projected gravity in body frame
3..5    body angular velocity / 10 rad/s
6..9    CyberGear joint extension fraction
10..11  DDSM115 wheel velocity / 15.7 rad/s
12..17  previous action
```

If the STM32 sends roll/pitch/yaw instead of projected gravity, convert roll and pitch to projected gravity before inference. Yaw does not affect gravity direction and is not used by this policy.

CyberGear extension fractions are normalized joint positions in this order:

```text
front_left, front_right, back_left, back_right
```

The two mirrored joints use opposite signs so that positive normalized value means "more extended" on every leg. The conversion used by the Python UART runner is:

```text
cg_norm = ((cg_angle * [1,-1,-1,1] + 10deg) / 100deg) * 2 - 1
```

Actions:

```text
0..3  CyberGear target angles
4     left DDSM115 current command
5     right DDSM115 current command
```

The simulation maps wheel actions to current with `wheel_current_max` from `standup_env_cfg.py` and then torque with `DDSM115_KT` from `sim_params.py`. The PhysX actuator hard limit remains 2.0 Nm, but the policy command envelope is lower: 1.6 Nm, or about 2.13 A at 0.75 Nm/A.

## Sign Convention

The left wheel USD is mirrored, so simulation negates left wheel torque:

```c
torque_left  = -current_left  * DDSM115_KT;
torque_right =  current_right * DDSM115_KT;
```

Keep the firmware-side motor direction mapping consistent with the physical wiring, not blindly with the USD. The important external behavior is that positive wheel action should help the learned policy perform the same maneuver on hardware as in simulation.

## Files To Keep Aligned

- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/standup_env.py`
- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/standup_env_cfg.py`
- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/sim_params.py`
- `source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/agents/rsl_rl_standup_cfg.py`
- `RealImplementationCode/`

When changing observation normalization, action scaling, motor signs, or the actor network shape, update the STM32 inference code in the same commit.

## Export

Train:

```bash
python scripts/rsl_rl/train.py \
  --task Template-Twowheeledrobot-Standup-v0 \
  --headless --num_envs 4096
```

Play/export from a checkpoint:

```bash
python scripts/rsl_rl/play.py \
  --task Template-Twowheeledrobot-Standup-v0 \
  --num_envs 1
```

Check exported policies under `logs/rsl_rl/standup_two_wheel/<run>/exported/`.

## UART Runner

Host-side runner:

```bash
python scripts/uart_policy_runner.py \
  --policy logs/rsl_rl/standup_two_wheel/<run>/exported/policy.pt \
  --port /dev/ttyACM0 \
  --baud 115200
```

STM32 sends JSON lines:

```json
{"roll":0.0,"pitch":1.57,"yaw":0.0,"gyro":[0.0,0.0,0.0],"cg":[0.0,0.0,0.0,0.0],"ddsm":[0.0,0.0]}
```

Host sends JSON lines:

```json
{"cg_target":[0.0,0.0,0.0,0.0],"wheel_current":[0.0,0.0],"action":[0.0,0.0,0.0,0.0,0.0,0.0]}
```

DDSM115 velocity is part of the policy observation. Send `[left, right]` wheel angular velocity in rad/s using the same sign convention as simulation.
