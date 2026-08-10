#!/usr/bin/env python3
"""Confirm which way the robot actually moves in world coordinates for a
negative-velocity command, and check it against the body-frame "velocity"
used for reward/tracking.

CLAUDE.md documents forward = -Y in the USD world frame. This probe settles
whether a user-visible "drives forward on negative command" report is that
axis convention showing up (world Y increases while the robot correctly
moves backward in its own reference frame) versus an actual sign bug.
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
parser.add_argument("--checkpoint", type=Path, required=True)
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--num_steps", type=int, default=100)
parser.add_argument("--v-cmd", type=float, default=-0.15)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
import TwoWheeledRobot.tasks  # noqa: F401
from rsl_rl.runners import OnPolicyRunner

from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg, agent_cfg):
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.terrain_mode = "flat"
    env_cfg.forced_command_mode = "fixed"
    env_cfg.forced_velocity_cmd_mps = args_cli.v_cmd
    env_cfg.forced_yaw_rate_cmd_radps = 0.0
    env_cfg.curriculum_stage = 1

    env = gym.make(args_cli.task, cfg=env_cfg)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    agent_cfg.device = env_cfg.sim.device
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(str(args_cli.checkpoint))
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    u = env.unwrapped

    with torch.inference_mode():
        reset_result = env.reset()
        obs = reset_result[0] if isinstance(reset_result, tuple) else reset_result
        if isinstance(obs, dict):
            obs = obs["policy"]
        y0 = u.robot.data.root_pos_w[:, 1].clone()

        for _ in range(args_cli.num_steps):
            actions = policy(obs)
            step_out = env.step(actions)
            obs = step_out[0]
            if isinstance(obs, dict):
                obs = obs["policy"]

        y1 = u.robot.data.root_pos_w[:, 1].clone()
        dy = (y1 - y0).mean().item()
        _, velocity, _, _, _, _ = u._state_terms()

    print(f"\ncommanded v_cmd (body-frame, forward positive) = {args_cli.v_cmd:+.3f} m/s")
    print(f"measured body-frame 'velocity' (reward/obs convention), mean = {velocity.mean().item():+.4f} m/s")
    print(f"world-frame Y displacement over {args_cli.num_steps} steps, mean = {dy:+.4f} m")
    print("(CLAUDE.md convention: forward = -Y, so a negative v_cmd correctly executed")
    print(" should move the robot in +Y -- i.e. dy > 0 here is EXPECTED, not a bug.)")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
