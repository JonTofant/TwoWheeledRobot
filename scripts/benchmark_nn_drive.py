#!/usr/bin/env python3
"""Deterministically benchmark an NN drive checkpoint or TorchScript policy.

Metrics are accumulated only while each environment is alive and use a
per-environment sample count. Termination causes are reported separately.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from nn_drive_benchmark_contract import SCENARIOS, benchmark_payload

_EXTENSION_SOURCE_PATH = Path(__file__).resolve().parents[1] / "source" / "TwoWheeledRobot"
if _EXTENSION_SOURCE_PATH.is_dir():
    sys.path.insert(0, str(_EXTENSION_SOURCE_PATH))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Benchmark NN drive policy.")
parser.add_argument("--task", type=str, default="Template-Twowheeledrobot-NNDrive-v0")
policy_group = parser.add_mutually_exclusive_group(required=True)
policy_group.add_argument("--policy", type=Path, help="TorchScript actor exported by play.py")
policy_group.add_argument("--checkpoint", type=Path, help="Raw RSL-RL model_*.pt checkpoint")
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--num_steps", type=int, default=1000, help="1000 steps = 15 s at 66.7 Hz")
parser.add_argument("--terrain", type=str, default="flat", choices=["flat", "generator"])
parser.add_argument("--seed", type=int, default=42, help="Fixed environment and policy seed")
parser.add_argument(
    "--scenarios",
    nargs="+",
    choices=tuple(SCENARIOS),
    default=list(SCENARIOS),
    help="Named scenarios to run (default: all).",
)
parser.add_argument("--json-output", type=Path, default=None, help="Write machine-readable results to this path")
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import json
import math
import random

import gymnasium as gym
import torch
import TwoWheeledRobot.tasks  # noqa: F401
from rsl_rl.runners import OnPolicyRunner
from TwoWheeledRobot.tasks.direct.twowheeledrobot.pure_nn_components import roll_from_projected_gravity

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)

from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config


def _scenario_metrics(policy, env, steps: int, settle_steps: int) -> dict[str, float]:
    # The whole scenario (reset + stepping) runs under inference_mode: stepping
    # turns env-internal buffers into inference tensors, and a later reset
    # outside inference mode would fail on their in-place updates.
    with torch.inference_mode():
        return _scenario_metrics_impl(policy, env, steps, settle_steps)


def _scenario_metrics_impl(policy, env, steps: int, settle_steps: int) -> dict[str, float]:
    unwrapped = env.unwrapped
    reset_result = env.reset()
    obs = reset_result[0] if isinstance(reset_result, tuple) else reset_result
    if isinstance(obs, dict):
        obs = obs["policy"]
    dt = unwrapped.step_dt
    alive = torch.ones(unwrapped.num_envs, dtype=torch.bool, device=unwrapped.device)
    survival = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    fall = torch.zeros(unwrapped.num_envs, dtype=torch.bool, device=unwrapped.device)
    physics_broken = torch.zeros_like(fall)
    invalid_state = torch.zeros_like(fall)
    timeout = torch.zeros_like(fall)
    vel_err_sq_sum = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    yaw_rate_err_sq_sum = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    pitch_sq_sum = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    roll_sq_sum = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    current_sq_sum = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    pos_err_abs_max = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    pos_err_abs_final = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    speed_abs_sum = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    velocity_sum = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    yaw_rate_sum = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    world_drift_final = torch.zeros(unwrapped.num_envs, device=unwrapped.device)
    sample_count = torch.zeros(unwrapped.num_envs, device=unwrapped.device)

    for step in range(steps):
        with torch.inference_mode():
            actions = policy(obs)
            step_out = env.step(actions)
            if len(step_out) == 5:
                obs, _, terminated, truncated, _ = step_out
                dones = terminated | truncated
            else:
                obs, _, dones, _ = step_out
            dones = dones.view(-1).to(device=alive.device, dtype=torch.bool)
            if isinstance(obs, dict):
                obs = obs["policy"]
        x_rel, velocity, pitch, _, _, yaw_rate = unwrapped._state_terms()
        alive_after_step = alive & ~dones
        if step >= settle_steps:
            sample_count += alive_after_step.float()
            vel_err_sq = (velocity - unwrapped._commands.v_cmd).pow(2)
            yaw_rate_err_sq = (yaw_rate - unwrapped._commands.w_cmd).pow(2)
            vel_err_sq_sum += torch.where(alive_after_step, vel_err_sq, torch.zeros_like(velocity))
            yaw_rate_err_sq_sum += torch.where(alive_after_step, yaw_rate_err_sq, torch.zeros_like(yaw_rate))
            roll = roll_from_projected_gravity(unwrapped.bno080.data.projected_gravity_b)
            pitch_sq_sum += torch.where(alive_after_step, pitch.pow(2), torch.zeros_like(pitch))
            roll_sq_sum += torch.where(alive_after_step, roll.pow(2), torch.zeros_like(roll))
            current_sq_sum += torch.where(
                alive_after_step,
                unwrapped._action_processor.command_current.pow(2).mean(dim=1),
                torch.zeros_like(velocity),
            )
            # Ground truth, immune to any reference/clamp change: how fast the
            # robot actually moved, and how far it actually ended up from spawn.
            speed_abs_sum += torch.where(alive_after_step, velocity.abs(), torch.zeros_like(velocity))
            velocity_sum += torch.where(alive_after_step, velocity, torch.zeros_like(velocity))
            yaw_rate_sum += torch.where(alive_after_step, yaw_rate, torch.zeros_like(yaw_rate))
            world_drift = torch.linalg.vector_norm(
                unwrapped.robot.data.root_pos_w[:, :2] - unwrapped._spawn_pos_xy, dim=1
            )
            world_drift_final = torch.where(alive_after_step, world_drift, world_drift_final)
            # pos_err is measured against the command reference. Since reference
            # anti-windup bounds it to +-cmd_pos_err_clamp_m by construction, it
            # can no longer be read as "how far the robot drove away" — use
            # world_drift_m/achieved_speed_mps for that.
            pos_err = unwrapped._commands.position_error_raw(x_rel)
            pos_err_abs_max = torch.where(
                alive_after_step, torch.maximum(pos_err_abs_max, pos_err.abs()), pos_err_abs_max
            )
            pos_err_abs_final = torch.where(alive_after_step, pos_err.abs(), pos_err_abs_final)
        survival = torch.where(alive, torch.full_like(survival, (step + 1) * dt), survival)
        fall |= alive & unwrapped._last_step_fall
        physics_broken |= alive & unwrapped._last_step_physics_broken
        invalid_state |= alive & unwrapped._last_step_invalid_state
        timeout |= alive & unwrapped._last_step_timeout
        alive &= ~dones

    sampled = sample_count > 0
    denom = sample_count.clamp_min(1.0)

    def sampled_mean(values: torch.Tensor) -> float:
        if not torch.any(sampled):
            return 0.0
        return values[sampled].mean().item()

    termination = fall | physics_broken | invalid_state
    return {
        "survival_time_s": survival.mean().item(),
        "fall_rate": fall.float().mean().item(),
        "physics_broken_rate": physics_broken.float().mean().item(),
        "invalid_state_rate": invalid_state.float().mean().item(),
        "timeout_rate": timeout.float().mean().item(),
        "termination_rate": termination.float().mean().item(),
        "rms_vel_err_mps": sampled_mean(torch.sqrt(vel_err_sq_sum / denom)),
        "rms_yaw_rate_err_radps": sampled_mean(torch.sqrt(yaw_rate_err_sq_sum / denom)),
        "rms_pitch_deg": sampled_mean(torch.sqrt(pitch_sq_sum / denom)) * 180.0 / math.pi,
        "rms_roll_deg": sampled_mean(torch.sqrt(roll_sq_sum / denom)) * 180.0 / math.pi,
        "max_pos_err_m": sampled_mean(pos_err_abs_max),
        # Error against the command reference. Bounded by anti-windup, so read
        # world_drift_m for actual displacement, not this.
        "final_pos_err_m": sampled_mean(pos_err_abs_final),
        "mean_velocity_mps": sampled_mean(velocity_sum / denom),
        "mean_yaw_rate_radps": sampled_mean(yaw_rate_sum / denom),
        "achieved_speed_mps": sampled_mean(speed_abs_sum / denom),
        "world_drift_m": sampled_mean(world_drift_final),
        "rms_current_a": sampled_mean(torch.sqrt(current_sq_sum / denom)),
        "sampled_env_fraction": sampled.float().mean().item(),
    }


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    random.seed(args_cli.seed)
    torch.manual_seed(args_cli.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args_cli.seed)
    env_cfg.seed = args_cli.seed
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.terrain_mode = args_cli.terrain
    env_cfg.forced_command_mode = "fixed"
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    # Build the environment once: env.close() tears down the simulation app in
    # this Isaac build, so scenarios are switched by mutating the live cfg
    # (curriculum stage, forced disturbance kind, forced commands are all
    # re-read at env.reset()).
    env = gym.make(args_cli.task, cfg=env_cfg)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    policy_path = args_cli.policy or args_cli.checkpoint
    if args_cli.checkpoint is not None:
        agent_cfg.seed = args_cli.seed
        agent_cfg.device = env_cfg.sim.device
        env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
        runner.load(str(args_cli.checkpoint))
        policy = runner.get_inference_policy(device=env.unwrapped.device)
    else:
        policy = torch.jit.load(str(args_cli.policy), map_location=env_cfg.sim.device).eval()
    settle_steps = int((env_cfg.cmd_settle_s + 1.0) / env.unwrapped.step_dt)

    results: dict[str, dict[str, float]] = {}
    for name in args_cli.scenarios:
        stage, dist_kind, v_cmd, w_cmd = SCENARIOS[name]
        live_cfg = env.unwrapped.cfg
        live_cfg.curriculum_stage = stage
        live_cfg.benchmark_disturbance_kind = dist_kind
        live_cfg.forced_velocity_cmd_mps = v_cmd
        live_cfg.forced_yaw_rate_cmd_radps = w_cmd
        metrics = _scenario_metrics(policy, env, args_cli.num_steps, settle_steps)
        results[name] = metrics
        print(
            f"\n[{name}] v_cmd={v_cmd:+.2f} m/s, w_cmd={w_cmd:+.2f} rad/s, "
            f"disturbance={dist_kind}, terrain={args_cli.terrain}"
        )
        for key, value in metrics.items():
            print(f"{key}: {value:.6g}")
    payload = benchmark_payload(
        policy=str(policy_path),
        seed=args_cli.seed,
        terrain=args_cli.terrain,
        num_envs=args_cli.num_envs,
        num_steps=args_cli.num_steps,
        scenarios=results,
    )
    if args_cli.json_output is not None:
        args_cli.json_output.parent.mkdir(parents=True, exist_ok=True)
        args_cli.json_output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"\nWrote benchmark JSON: {args_cli.json_output}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
