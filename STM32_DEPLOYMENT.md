# STM32 Deployment Notes

These notes describe the cleaned stand-up policy interface. The Isaac Lab task is:

```text
Template-Twowheeledrobot-Standup-v0
```

## Policy Contract

The trained actor takes 16 normalized `float32` observations and returns 6 actions in `[-1, 1]`.

Observations:

```text
0..2    projected gravity in body frame
3..5    body angular velocity / 10 rad/s
6..9    CyberGear joint extension fraction
10..15  previous action
```

Actions:

```text
0..3  CyberGear target angles
4     left DDSM115 current command
5     right DDSM115 current command
```

The simulation maps wheel actions to current with `wheel_current_max` from `standup_env_cfg.py` and then torque with `DDSM115_KT` from `sim_params.py`.

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
