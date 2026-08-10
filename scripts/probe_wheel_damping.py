#!/usr/bin/env python3
"""Read back per-env wheel joint damping immediately after reset.

Verifies that NNDriveEnv._randomize_cybergear_gains no longer clobbers the
wheel-column viscous-damping randomization applied by
PureNNBalanceEnv._apply_pure_nn_physical_randomization earlier in the same
reset (see nn_drive_env.py::_randomize_cybergear_gains). Expected damping
after a correct reset falls inside cfg.wheel_viscous_damping_range; the bug
this probes for silently restores WHEEL_INTERNAL_DAMPING (0.2 Nm*s/rad).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_EXTENSION_SOURCE_PATH = Path(__file__).resolve().parents[1] / "source" / "TwoWheeledRobot"
if _EXTENSION_SOURCE_PATH.is_dir():
    sys.path.insert(0, str(_EXTENSION_SOURCE_PATH))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", type=str, default="Template-Twowheeledrobot-NNDrive-v0")
parser.add_argument("--num_envs", type=int, default=64)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
import TwoWheeledRobot.tasks  # noqa: F401

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg, agent_cfg):
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.terrain_mode = "flat"
    env = gym.make(args_cli.task, cfg=env_cfg)
    u = env.unwrapped
    with torch.inference_mode():
        env.reset()
        # Second reset of the same envs exercises the steady-state reset path
        # (not just first-construction defaults), which is what the bug hits.
        env.reset()
        wheel_cols = [u._left_wheel_ids[0], u._right_wheel_ids[0]]
        damping = u.robot.data.joint_damping[:, wheel_cols]
        lo, hi = u.cfg.wheel_viscous_damping_range
        print(f"\nwheel_viscous_damping_range (cfg) = ({lo}, {hi})")
        print(f"post-reset wheel damping: mean={damping.mean().item():.5f} "
              f"min={damping.min().item():.5f} max={damping.max().item():.5f}")
        in_range = ((damping >= lo) & (damping <= hi)).float().mean().item()
        print(f"fraction of (env, wheel) values inside expected range: {in_range:.3f}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
