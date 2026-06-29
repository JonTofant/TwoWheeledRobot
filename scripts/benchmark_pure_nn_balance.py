#!/usr/bin/env python3
"""Benchmark a pure NN balance policy across required disturbance scenarios."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_EXTENSION_SOURCE_PATH = Path(__file__).resolve().parents[1] / "source" / "TwoWheeledRobot"
if _EXTENSION_SOURCE_PATH.is_dir():
    sys.path.insert(0, str(_EXTENSION_SOURCE_PATH))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Benchmark pure NN balance policy.")
parser.add_argument("--task", type=str, default="Template-Twowheeledrobot-PureNNBalance-v0")
parser.add_argument("--policy", required=True, type=Path, help="TorchScript actor exported by play.py")
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--num_steps", type=int, default=400, help="400 steps = 8 s at 50 Hz")
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import math

import gymnasium as gym
import torch

from isaaclab.envs import DirectMARLEnv, DirectMARLEnvCfg, DirectRLEnvCfg, ManagerBasedRLEnvCfg, multi_agent_to_single_agent
from isaaclab_tasks.utils.hydra import hydra_task_config

import isaaclab_tasks  # noqa: F401
import TwoWheeledRobot.tasks  # noqa: F401
from TwoWheeledRobot.tasks.direct.twowheeledrobot.pure_nn_components import pitch_from_projected_gravity


def _scenario_metrics(policy: torch.nn.Module, env, steps: int) -> dict[str, float]:
    unwrapped = env.unwrapped
    reset_out = env.reset()
    obs_dict = reset_out[0] if isinstance(reset_out, tuple) else reset_out
    obs = obs_dict["policy"] if isinstance(obs_dict, dict) else obs_dict
    dt = unwrapped.step_dt
    alive = torch.ones(unwrapped.num_envs, dtype=torch.bool, device=unwrapped.device)
    survival = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    pitch_hist = []
    rate_hist = []
    current_hist = []
    pos_hist = []
    recovery_time = torch.full((unwrapped.num_envs,), float("nan"), device=unwrapped.device)
    stable_count = torch.zeros(unwrapped.num_envs, device=unwrapped.device, dtype=torch.long)
    disturbance_seen = torch.zeros(unwrapped.num_envs, dtype=torch.bool, device=unwrapped.device)

    for step in range(steps):
        with torch.inference_mode():
            actions = policy(obs)
            step_out = env.step(actions)
            if len(step_out) == 5:
                obs, _, terminated, truncated, _ = step_out
                dones = terminated | truncated
            else:
                obs, _, dones, _ = step_out
        pitch = pitch_from_projected_gravity(unwrapped.bno080.data.projected_gravity_b)
        pitch_rate = -unwrapped.bno080.data.ang_vel_b[:, 0]
        pos = 0.5 * unwrapped.robot.data.joint_pos[:, unwrapped._wheel_ids].mul(unwrapped._wheel_sign).sum(dim=1) * 0.05035
        current = unwrapped._action_processor.command_current
        pitch_hist.append(pitch)
        rate_hist.append(pitch_rate)
        current_hist.append(current)
        pos_hist.append(pos)
        survival = torch.where(alive, torch.full_like(survival, (step + 1) * dt), survival)
        alive &= ~dones

        disturbance_step = torch.clamp(unwrapped.episode_length_buf - 1, min=0)
        disturbance_active = unwrapped._disturbance.active_for_recovery(disturbance_step)
        disturbance_seen |= disturbance_active
        measuring_recovery = disturbance_seen & ~disturbance_active
        stable = (pitch.abs() < math.radians(2.0)) & (pitch_rate.abs() < 0.2)
        stable_count = torch.where(measuring_recovery & stable, stable_count + 1, torch.zeros_like(stable_count))
        recovered = stable_count >= int(math.ceil(0.5 / dt))
        recovery_time = torch.where(recovered & torch.isnan(recovery_time), torch.full_like(recovery_time, step * dt), recovery_time)

    pitch_t = torch.stack(pitch_hist)
    rate_t = torch.stack(rate_hist)
    current_t = torch.stack(current_hist)
    pos_t = torch.stack(pos_hist)
    return {
        "survival_time_s": survival.mean().item(),
        "max_pitch_deg": (pitch_t.abs().max() * 180.0 / math.pi).item(),
        "rms_pitch_deg": (torch.sqrt(pitch_t.pow(2).mean()) * 180.0 / math.pi).item(),
        "rms_pitch_rate": torch.sqrt(rate_t.pow(2).mean()).item(),
        "rms_current_a": torch.sqrt(current_t.pow(2).mean()).item(),
        "max_current_a": current_t.abs().max().item(),
        "final_position_error_m": pos_t[-1].abs().mean().item(),
        "recovery_time_s": torch.nanmean(recovery_time).item(),
    }


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, _agent_cfg):
    env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    policy = torch.jit.load(str(args_cli.policy), map_location=env_cfg.sim.device).eval()
    scenarios = [
        ("none", 1),
        ("human_push", 3),
        ("double_human_push", 3),
        ("payload", 4),
        ("payload_push", 5),
        ("sine_diagnostic", 5),
        ("slope", 5),
        ("randomized_motor", 1),
    ]
    for name, stage in scenarios:
        env_cfg.curriculum_stage = stage
        env_cfg.benchmark_disturbance_kind = "none" if name == "randomized_motor" else name
        env = gym.make(args_cli.task, cfg=env_cfg)
        if isinstance(env.unwrapped, DirectMARLEnv):
            env = multi_agent_to_single_agent(env)
        metrics = _scenario_metrics(policy, env, args_cli.num_steps)
        env.close()
        print(f"\n[{name}]")
        for key, value in metrics.items():
            print(f"{key}: {value:.6g}")


if __name__ == "__main__":
    main()
    simulation_app.close()
