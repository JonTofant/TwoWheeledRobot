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
parser.add_argument("--include-sine-diagnostic", action="store_true", help="Include optional sine diagnostic benchmark.")
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
from TwoWheeledRobot.tasks.direct.twowheeledrobot.pure_nn_components import yaw_from_quat_wxyz


def _set_initial_pitch_bin(env, low_deg: float, high_deg: float) -> torch.Tensor:
    unwrapped = env.unwrapped
    env_ids = torch.arange(unwrapped.num_envs, device=unwrapped.device, dtype=torch.long)
    n = len(env_ids)
    abs_pitch = torch.empty(n, device=unwrapped.device).uniform_(math.radians(low_deg), math.radians(high_deg))
    sign = torch.where(torch.rand(n, device=unwrapped.device) < 0.5, -1.0, 1.0)
    pitch = abs_pitch * sign

    root_state = unwrapped.robot.data.default_root_state[env_ids].clone()
    root_state[:, :3] += unwrapped.scene.env_origins[env_ids]
    root_state[:, 2] = unwrapped.scene.env_origins[env_ids, 2] + unwrapped.cfg.spawn_upright_z
    root_state[:, 3] = torch.cos(0.5 * pitch)
    root_state[:, 4] = -torch.sin(0.5 * pitch)
    root_state[:, 5:7] = 0.0
    root_state[:, 7:] = 0.0
    unwrapped.robot.write_root_pose_to_sim(root_state[:, :7], env_ids)
    unwrapped.robot.write_root_velocity_to_sim(root_state[:, 7:], env_ids)
    unwrapped._spawn_pos_xy[env_ids] = root_state[:, :2]
    unwrapped._yaw_reference[env_ids] = yaw_from_quat_wxyz(root_state[:, 3:7])

    joint_pos = unwrapped.robot.data.default_joint_pos[env_ids].clone()
    joint_vel = unwrapped.robot.data.default_joint_vel[env_ids].clone()
    joint_pos[:, unwrapped._cg_ids] = 0.0
    joint_vel[:, unwrapped._cg_ids] = 0.0
    joint_vel[:, unwrapped._wheel_ids] = 0.0
    unwrapped.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
    unwrapped.robot.set_joint_position_target(joint_pos, env_ids=env_ids)

    unwrapped._prev_actions[env_ids] = 0.0
    unwrapped._cur_actions[env_ids] = 0.0
    unwrapped._obs_now[env_ids] = 0.0
    unwrapped._obs_delay[env_ids] = 0.0
    unwrapped._obs_delay_samples[env_ids] = 0
    unwrapped._pitch_bias[env_ids] = 0.0
    unwrapped._action_processor.reset(env_ids)
    if hasattr(unwrapped, "_fall_counter"):
        unwrapped._fall_counter[env_ids] = 0
        unwrapped._last_fall[env_ids] = False
        unwrapped._last_physics_broken[env_ids] = False
        unwrapped._last_invalid_state[env_ids] = False
        unwrapped._last_timeout[env_ids] = False
        unwrapped._last_terminal_penalty[env_ids] = 0.0
        unwrapped._termination_update_step[env_ids] = -1
    return pitch


def _terminal_reason_summary(reason: torch.Tensor) -> str:
    labels = {0: "none", 1: "timeout", 2: "fall", 3: "physics_broken", 4: "invalid_state"}
    counts = []
    for code, label in labels.items():
        count = int((reason == code).sum().item())
        if count > 0:
            counts.append(f"{label}:{count}")
    return ",".join(counts) if counts else "none:0"


def _scenario_metrics(policy: torch.nn.Module, env, steps: int, pitch_bin: tuple[float, float]) -> dict[str, float | str]:
    unwrapped = env.unwrapped
    env.reset()
    _set_initial_pitch_bin(env, *pitch_bin)
    obs = unwrapped._get_observations()["policy"]
    dt = unwrapped.step_dt
    alive = torch.ones(unwrapped.num_envs, dtype=torch.bool, device=unwrapped.device)
    survival = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    pitch_hist = []
    velocity_hist = []
    current_hist = []
    terminal_reason = torch.zeros(unwrapped.num_envs, device=unwrapped.device, dtype=torch.long)

    for step in range(steps):
        with torch.inference_mode():
            actions = policy(obs)
            step_out = env.step(actions)
            if len(step_out) == 5:
                obs, _, terminated, truncated, _ = step_out
                dones = terminated | truncated
            else:
                obs, _, dones, _ = step_out
        _, velocity, pitch, _, _, _ = unwrapped._state_terms()
        current = unwrapped._action_processor.command_current
        pitch_hist.append(pitch)
        velocity_hist.append(velocity)
        current_hist.append(current)
        survival = torch.where(alive, torch.full_like(survival, (step + 1) * dt), survival)
        newly_done = alive & dones
        reason_step = torch.zeros_like(terminal_reason)
        if hasattr(unwrapped, "_last_timeout"):
            reason_step = torch.where(unwrapped._last_timeout, torch.ones_like(reason_step), reason_step)
            reason_step = torch.where(unwrapped._last_fall, torch.full_like(reason_step, 2), reason_step)
            reason_step = torch.where(unwrapped._last_physics_broken, torch.full_like(reason_step, 3), reason_step)
            reason_step = torch.where(unwrapped._last_invalid_state, torch.full_like(reason_step, 4), reason_step)
        terminal_reason = torch.where(newly_done, reason_step, terminal_reason)
        alive &= ~dones

    pitch_t = torch.stack(pitch_hist)
    velocity_t = torch.stack(velocity_hist)
    current_t = torch.stack(current_hist)
    return {
        "survival_time_s": survival.mean().item(),
        "fall_rate": (terminal_reason == 2).float().mean().item(),
        "max_pitch_deg": (pitch_t.abs().max() * 180.0 / math.pi).item(),
        "rms_pitch_deg": (torch.sqrt(pitch_t.pow(2).mean()) * 180.0 / math.pi).item(),
        "rms_velocity_mps": torch.sqrt(velocity_t.pow(2).mean()).item(),
        "final_velocity_mps": velocity_t[-1].mean().item(),
        "rms_current_a": torch.sqrt(current_t.pow(2).mean()).item(),
        "max_current_a": current_t.abs().max().item(),
        "terminal_reason": _terminal_reason_summary(terminal_reason),
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
        ("randomized_motor", 1),
    ]
    if args_cli.include_sine_diagnostic:
        scenarios.append(("sine_diagnostic", 5))
    pitch_bins = [(0.0, 5.0), (5.0, 10.0), (10.0, 15.0), (15.0, 20.0)]
    for name, stage in scenarios:
        env_cfg.curriculum_stage = stage
        env_cfg.benchmark_disturbance_kind = "none" if name == "randomized_motor" else name
        print(f"\n[{name}]")
        for pitch_bin in pitch_bins:
            env = gym.make(args_cli.task, cfg=env_cfg)
            if isinstance(env.unwrapped, DirectMARLEnv):
                env = multi_agent_to_single_agent(env)
            metrics = _scenario_metrics(policy, env, args_cli.num_steps, pitch_bin)
            env.close()
            print(f"pitch_bin_deg: {pitch_bin[0]:.0f}-{pitch_bin[1]:.0f}")
            for key, value in metrics.items():
                if isinstance(value, str):
                    print(f"{key}: {value}")
                else:
                    print(f"{key}: {value:.6g}")


if __name__ == "__main__":
    main()
    simulation_app.close()
