#!/usr/bin/env python3
"""Check that rew_hold_world_drift fires during station-keeping drift and is
gated off during commanded driving, using an already-trained checkpoint."""

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
parser.add_argument("--num_envs", type=int, default=32)
parser.add_argument("--num_steps", type=int, default=400)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import gymnasium as gym
import TwoWheeledRobot.tasks  # noqa: F401
from rsl_rl.runners import OnPolicyRunner

from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config


def run(env_cfg, agent_cfg, v_cmd, label):
    env_cfg.forced_command_mode = "fixed"
    env_cfg.forced_velocity_cmd_mps = v_cmd
    env_cfg.forced_yaw_rate_cmd_radps = 0.0
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
        drift_sum = torch.zeros(u.num_envs, device=u.device)
        for _ in range(args_cli.num_steps):
            actions = policy(obs)
            step_out = env.step(actions)
            obs = step_out[0]
            if isinstance(obs, dict):
                obs = obs["policy"]
            drift_sum += u._last_reward_components["hold_world_drift"]
        world_drift = torch.linalg.vector_norm(u.robot.data.root_pos_w[:, :2] - u._spawn_pos_xy, dim=1)

    print(f"\n[{label}] v_cmd={v_cmd:+.2f}")
    print(f"  final world_drift: mean={world_drift.mean().item():.4f} max={world_drift.max().item():.4f}")
    print(f"  mean per-step hold_world_drift reward component: {drift_sum.mean().item() / args_cli.num_steps:.5f}")
    env.close()


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg, agent_cfg):
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.terrain_mode = "flat"
    env_cfg.curriculum_stage = 1
    run(env_cfg, agent_cfg, 0.0, "station_keeping (hold gate should be ON)")
    run(env_cfg, agent_cfg, 0.3, "driving (hold gate should be OFF)")


if __name__ == "__main__":
    main()
    simulation_app.close()
