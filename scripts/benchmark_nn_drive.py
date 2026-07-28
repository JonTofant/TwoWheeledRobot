#!/usr/bin/env python3
"""Benchmark an NN drive policy: station keeping, command tracking, disturbances.

Runs the exported TorchScript actor (from scripts/rsl_rl/play.py) through fixed
command scenarios and reports survival, fall rate, and tracking quality. Use
--terrain generator to benchmark on the bumps/slopes terrain mix.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_EXTENSION_SOURCE_PATH = Path(__file__).resolve().parents[1] / "source" / "TwoWheeledRobot"
if _EXTENSION_SOURCE_PATH.is_dir():
    sys.path.insert(0, str(_EXTENSION_SOURCE_PATH))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Benchmark NN drive policy.")
parser.add_argument("--task", type=str, default="Template-Twowheeledrobot-NNDrive-v0")
parser.add_argument("--policy", required=True, type=Path, help="TorchScript actor exported by play.py")
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--num_steps", type=int, default=1000, help="1000 steps = 15 s at 66.7 Hz")
parser.add_argument("--terrain", type=str, default="flat", choices=["flat", "generator"])
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import math

import gymnasium as gym
import torch
import TwoWheeledRobot.tasks  # noqa: F401
from TwoWheeledRobot.tasks.direct.twowheeledrobot.pure_nn_components import roll_from_projected_gravity

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config

# name, curriculum_stage, forced disturbance kind, v_cmd (m/s), w_cmd (rad/s)
SCENARIOS = [
    ("station_keeping", 1, "none", 0.0, 0.0),
    ("drive_forward", 1, "none", 0.4, 0.0),
    ("drive_backward", 1, "none", -0.4, 0.0),
    ("turn_in_place", 1, "none", 0.0, 1.2),
    ("drive_and_turn", 1, "none", 0.3, 0.8),
    ("push_while_still", 3, "human_push", 0.0, 0.0),
    ("push_while_driving", 3, "human_push", 0.35, 0.0),
    ("payload_while_still", 4, "payload", 0.0, 0.0),
    ("payload_while_driving", 4, "payload", 0.35, 0.0),
]


def _scenario_metrics(policy: torch.nn.Module, env, steps: int, settle_steps: int) -> dict[str, float | str]:
    # The whole scenario (reset + stepping) runs under inference_mode: stepping
    # turns env-internal buffers into inference tensors, and a later reset
    # outside inference mode would fail on their in-place updates.
    with torch.inference_mode():
        return _scenario_metrics_impl(policy, env, steps, settle_steps)


def _scenario_metrics_impl(policy: torch.nn.Module, env, steps: int, settle_steps: int) -> dict[str, float | str]:
    unwrapped = env.unwrapped
    obs, _ = env.reset()
    if isinstance(obs, dict):
        obs = obs["policy"]
    dt = unwrapped.step_dt
    alive = torch.ones(unwrapped.num_envs, dtype=torch.bool, device=unwrapped.device)
    survival = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    fall = torch.zeros(unwrapped.num_envs, dtype=torch.bool, device=unwrapped.device)
    vel_err_sq_sum = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    yaw_rate_err_sq_sum = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    pitch_sq_sum = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    roll_sq_sum = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    current_sq_sum = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    pos_err_abs_max = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    pos_err_abs_final = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    speed_abs_sum = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    world_drift_final = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    tracked_steps = 0

    for step in range(steps):
        with torch.inference_mode():
            actions = policy(obs)
            step_out = env.step(actions)
            if len(step_out) == 5:
                obs, _, terminated, truncated, _ = step_out
                dones = terminated | truncated
            else:
                obs, _, dones, _ = step_out
                terminated = dones
            if isinstance(obs, dict):
                obs = obs["policy"]
        x_rel, velocity, pitch, _, _, yaw_rate = unwrapped._state_terms()
        if step >= settle_steps:
            tracked_steps += 1
            vel_err_sq = (velocity - unwrapped._commands.v_cmd).pow(2)
            yaw_rate_err_sq = (yaw_rate - unwrapped._commands.w_cmd).pow(2)
            vel_err_sq_sum += torch.where(alive, vel_err_sq, torch.zeros_like(velocity))
            yaw_rate_err_sq_sum += torch.where(alive, yaw_rate_err_sq, torch.zeros_like(yaw_rate))
            roll = roll_from_projected_gravity(unwrapped.bno080.data.projected_gravity_b)
            pitch_sq_sum += torch.where(alive, pitch.pow(2), torch.zeros_like(pitch))
            roll_sq_sum += torch.where(alive, roll.pow(2), torch.zeros_like(roll))
            current_sq_sum += torch.where(
                alive, unwrapped._action_processor.command_current.pow(2).mean(dim=1), torch.zeros_like(velocity)
            )
            # Ground truth, immune to any reference/clamp change: how fast the
            # robot actually moved, and how far it actually ended up from spawn.
            speed_abs_sum += torch.where(alive, velocity.abs(), torch.zeros_like(velocity))
            world_drift = torch.linalg.vector_norm(
                unwrapped.robot.data.root_pos_w[:, :2] - unwrapped._spawn_pos_xy, dim=1
            )
            world_drift_final = torch.where(alive, world_drift, world_drift_final)
            # pos_err is measured against the command reference. Since reference
            # anti-windup bounds it to +-cmd_pos_err_clamp_m by construction, it
            # can no longer be read as "how far the robot drove away" — use
            # world_drift_m/achieved_speed_mps for that.
            pos_err = unwrapped._commands.position_error_raw(x_rel)
            pos_err_abs_max = torch.where(alive, torch.maximum(pos_err_abs_max, pos_err.abs()), pos_err_abs_max)
            pos_err_abs_final = torch.where(alive, pos_err.abs(), pos_err_abs_final)
        survival = torch.where(alive, torch.full_like(survival, (step + 1) * dt), survival)
        # The in-step reset clears _last_fall for done envs before env.step()
        # returns, so use the terminated flag (fall | physics_broken | invalid).
        fall = fall | (alive & terminated.view(-1).to(alive.device))
        alive &= ~dones

    denom = max(tracked_steps, 1)
    return {
        "survival_time_s": survival.mean().item(),
        "fall_rate": fall.float().mean().item(),
        "rms_vel_err_mps": torch.sqrt(vel_err_sq_sum / denom).mean().item(),
        "rms_yaw_rate_err_radps": torch.sqrt(yaw_rate_err_sq_sum / denom).mean().item(),
        "rms_pitch_deg": (torch.sqrt(pitch_sq_sum / denom).mean() * 180.0 / math.pi).item(),
        "rms_roll_deg": (torch.sqrt(roll_sq_sum / denom).mean() * 180.0 / math.pi).item(),
        "max_pos_err_m": pos_err_abs_max.mean().item(),
        # Error against the command reference. Bounded by anti-windup, so read
        # world_drift_m for actual displacement, not this.
        "final_pos_err_m": pos_err_abs_final.mean().item(),
        "achieved_speed_mps": (speed_abs_sum / denom).mean().item(),
        "world_drift_m": world_drift_final.mean().item(),
        "rms_current_a": torch.sqrt(current_sq_sum / denom).mean().item(),
    }


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, _agent_cfg):
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.terrain_mode = args_cli.terrain
    env_cfg.forced_command_mode = "fixed"
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    policy = torch.jit.load(str(args_cli.policy), map_location=env_cfg.sim.device).eval()

    # Build the environment once: env.close() tears down the simulation app in
    # this Isaac build, so scenarios are switched by mutating the live cfg
    # (curriculum stage, forced disturbance kind, forced commands are all
    # re-read at env.reset()).
    env = gym.make(args_cli.task, cfg=env_cfg)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    settle_steps = int((env_cfg.cmd_settle_s + 1.0) / env.unwrapped.step_dt)

    for name, stage, dist_kind, v_cmd, w_cmd in SCENARIOS:
        live_cfg = env.unwrapped.cfg
        live_cfg.curriculum_stage = stage
        live_cfg.benchmark_disturbance_kind = dist_kind
        live_cfg.forced_velocity_cmd_mps = v_cmd
        live_cfg.forced_yaw_rate_cmd_radps = w_cmd
        metrics = _scenario_metrics(policy, env, args_cli.num_steps, settle_steps)
        print(
            f"\n[{name}] v_cmd={v_cmd:+.2f} m/s, w_cmd={w_cmd:+.2f} rad/s, "
            f"disturbance={dist_kind}, terrain={args_cli.terrain}"
        )
        for key, value in metrics.items():
            print(f"{key}: {value:.6g}" if not isinstance(value, str) else f"{key}: {value}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
