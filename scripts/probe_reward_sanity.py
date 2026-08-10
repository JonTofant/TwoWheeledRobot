#!/usr/bin/env python3
"""Step with random actions and check reward components stay finite and sane.

No trained checkpoint needed/used -- this only needs to confirm the new
observation (21-wide) and reward (yaw_err_cos-based) plumbing doesn't produce
NaN/Inf or blow up, across both station-keeping and driving commands.
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
parser.add_argument("--num_envs", type=int, default=32)
parser.add_argument("--num_steps", type=int, default=300)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import gymnasium as gym
import TwoWheeledRobot.tasks  # noqa: F401

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg, agent_cfg):
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.terrain_mode = "flat"
    env_cfg.curriculum_stage = 3  # nonzero yaw/velocity commands so driving is exercised
    env = gym.make(args_cli.task, cfg=env_cfg)
    u = env.unwrapped
    print(f"observation_space={u.cfg.observation_space} action_space={u.cfg.action_space}")

    with torch.inference_mode():
        env.reset()
        obs_shape_seen = None
        all_finite = True
        min_reward, max_reward = float("inf"), float("-inf")
        component_ranges = {}
        for step in range(args_cli.num_steps):
            actions = torch.empty(u.num_envs, u.cfg.action_space, device=u.device).uniform_(-1.0, 1.0)
            step_out = env.step(actions)
            obs = step_out[0]
            if isinstance(obs, dict):
                obs = obs["policy"]
            obs_shape_seen = tuple(obs.shape)
            if not torch.isfinite(obs).all():
                all_finite = False
                print(f"step {step}: NON-FINITE OBS")
            components = u._last_reward_components
            for key, val in components.items():
                if not torch.isfinite(val).all():
                    all_finite = False
                    print(f"step {step}: NON-FINITE reward component '{key}'")
                lo, hi = val.min().item(), val.max().item()
                clo, chi = component_ranges.get(key, (float("inf"), float("-inf")))
                component_ranges[key] = (min(clo, lo), max(chi, hi))
            total = components["total"]
            min_reward = min(min_reward, total.min().item())
            max_reward = max(max_reward, total.max().item())

    print(f"\nobs shape per step: {obs_shape_seen} (expected (*, {u.cfg.observation_space}))")
    print(f"all finite throughout {args_cli.num_steps} steps: {all_finite}")
    print(f"total reward range over run: [{min_reward:.4f}, {max_reward:.4f}]")
    print(f"\n{'component':22}{'min':>10}{'max':>10}")
    for key, (lo, hi) in sorted(component_ranges.items()):
        print(f"{key:22}{lo:10.4f}{hi:10.4f}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
