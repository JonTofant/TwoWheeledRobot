#!/usr/bin/env python3
"""Diagnose why NN drive policies fall while turning.

Sweeps the commanded yaw rate and, for each, reports whether the robot can
physically deliver it (achieved yaw rate, wheel-torque saturation), how it
fails (pitch-axis fall vs sideways tilt), and how long it lasts. Run it against
two policies to compare.

The question this answers: is sustained turning a physical limit, a
training-distribution gap, or a reward problem?

    python scripts/diagnose_turn_failure.py --policy <exported policy.pt> --headless
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_EXTENSION_SOURCE_PATH = Path(__file__).resolve().parents[1] / "source" / "TwoWheeledRobot"
if _EXTENSION_SOURCE_PATH.is_dir():
    sys.path.insert(0, str(_EXTENSION_SOURCE_PATH))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Diagnose NN drive turning failures.")
parser.add_argument("--task", type=str, default="Template-Twowheeledrobot-NNDrive-v0")
parser.add_argument("--policy", required=True, type=Path, help="TorchScript actor exported by play.py")
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--num_steps", type=int, default=1000, help="1000 steps = 15 s at 66.7 Hz")
parser.add_argument("--terrain", type=str, default="flat", choices=["flat", "generator"])
parser.add_argument(
    "--yaw-rates",
    type=float,
    nargs="+",
    default=[0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.6, 2.0],
    help="commanded yaw rates (rad/s) to sweep",
)
parser.add_argument("--v-cmd", type=float, default=0.0, help="forward command held during the sweep")
parser.add_argument(
    "--pin-yaw-ref",
    action="store_true",
    help="hold the heading reference at the actual heading so yaw_err never accumulates",
)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import math

import gymnasium as gym
import torch
import TwoWheeledRobot.tasks  # noqa: F401
from TwoWheeledRobot.tasks.direct.twowheeledrobot.pure_nn_components import (
    roll_from_projected_gravity,
    yaw_from_quat_wxyz,
)

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config


def _sweep_point(policy: torch.nn.Module, env, steps: int, settle_steps: int, pin_yaw_ref: bool) -> dict[str, float]:
    # Reset + stepping share one inference_mode block: stepping turns env
    # buffers into inference tensors, and a later reset outside inference mode
    # would fail on their in-place updates.
    with torch.inference_mode():
        return _sweep_point_impl(policy, env, steps, settle_steps, pin_yaw_ref)


def _sweep_point_impl(
    policy: torch.nn.Module, env, steps: int, settle_steps: int, pin_yaw_ref: bool
) -> dict[str, float]:
    u = env.unwrapped
    dev = u.device
    obs, _ = env.reset()
    if isinstance(obs, dict):
        obs = obs["policy"]

    n = u.num_envs
    alive = torch.ones(n, dtype=torch.bool, device=dev)
    survival = torch.zeros(n, device=dev)
    fell = torch.zeros(n, dtype=torch.bool, device=dev)
    # Which threshold was crossed on the step the env terminated: the pitch
    # (fore/aft balance) limit or the total-tilt (sideways) limit.
    fell_by_pitch = torch.zeros(n, dtype=torch.bool, device=dev)
    fell_by_tilt = torch.zeros(n, dtype=torch.bool, device=dev)
    yaw_rate_sum = torch.zeros(n, device=dev)
    roll_sq_sum = torch.zeros(n, device=dev)
    pitch_sq_sum = torch.zeros(n, device=dev)
    cur_sq_sum = torch.zeros(n, device=dev)
    derate_clip_sum = torch.zeros(n, device=dev)
    tracked = 0

    pitch_limit = math.radians(u.cfg.fall_pitch_threshold_deg)
    tilt_limit = math.radians(u.cfg.fall_total_tilt_threshold_deg)

    # env.step() resets done envs in-step, so state read after it returns is the
    # post-reset (upright) state. Fall-cause attribution must use the values
    # captured before the step that terminated.
    prev_pitch = torch.zeros(n, device=dev)
    prev_tilt = torch.zeros(n, device=dev)

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

        if pin_yaw_ref:
            # Hold the heading reference at the actual heading so yaw_err can
            # never accumulate. Isolates "cannot turn" from "cannot cope with an
            # accumulated/wrapping heading error".
            u._commands.yaw_ref = yaw_from_quat_wxyz(u.robot.data.root_quat_w).clone()

        _, _, pitch, _, _, yaw_rate = u._state_terms()
        roll = roll_from_projected_gravity(u.bno080.data.projected_gravity_b)
        tilt = u._last_total_tilt

        if step >= settle_steps:
            tracked += 1
            yaw_rate_sum += torch.where(alive, yaw_rate, torch.zeros_like(yaw_rate))
            roll_sq_sum += torch.where(alive, roll.pow(2), torch.zeros_like(roll))
            pitch_sq_sum += torch.where(alive, pitch.pow(2), torch.zeros_like(pitch))
            cur_sq_sum += torch.where(
                alive, u._action_processor.command_current.pow(2).mean(dim=1), torch.zeros_like(yaw_rate)
            )
            # Torque-speed derate is clipping when the requested current torque
            # exceeds what the motor can deliver at the current wheel speed.
            clipped = (u._wheel_tau_current.abs() > u._wheel_tau_speed_limit).any(dim=1).float()
            derate_clip_sum += torch.where(alive, clipped, torch.zeros_like(yaw_rate))

        survival = torch.where(alive, torch.full_like(survival, (step + 1) * u.step_dt), survival)
        newly_done = alive & terminated.view(-1).to(dev)
        fell |= newly_done
        fell_by_pitch |= newly_done & (prev_pitch.abs() > pitch_limit)
        fell_by_tilt |= newly_done & (prev_tilt > tilt_limit)
        alive &= ~dones
        prev_pitch, prev_tilt = pitch.clone(), tilt.clone()

    d = max(tracked, 1)
    n_fell = max(int(fell.sum().item()), 1)
    return {
        "fall_rate": fell.float().mean().item(),
        "survival_s": survival.mean().item(),
        "achieved_yaw_radps": (yaw_rate_sum / d).mean().item(),
        "rms_roll_deg": (torch.sqrt(roll_sq_sum / d).mean() * 180.0 / math.pi).item(),
        "rms_pitch_deg": (torch.sqrt(pitch_sq_sum / d).mean() * 180.0 / math.pi).item(),
        "rms_current_a": torch.sqrt(cur_sq_sum / d).mean().item(),
        "derate_clip_frac": (derate_clip_sum / d).mean().item(),
        "fell_by_pitch_frac": (fell_by_pitch.sum().item() / n_fell),
        "fell_by_tilt_frac": (fell_by_tilt.sum().item() / n_fell),
    }


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, _agent_cfg):
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.terrain_mode = args_cli.terrain
    env_cfg.forced_command_mode = "fixed"
    env_cfg.benchmark_disturbance_kind = "none"
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    policy = torch.jit.load(str(args_cli.policy), map_location=env_cfg.sim.device).eval()

    env = gym.make(args_cli.task, cfg=env_cfg)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    settle_steps = int((env_cfg.cmd_settle_s + 1.0) / env.unwrapped.step_dt)

    cols = [
        "fall_rate",
        "survival_s",
        "achieved_yaw_radps",
        "rms_roll_deg",
        "rms_pitch_deg",
        "rms_current_a",
        "derate_clip_frac",
        "fell_by_pitch_frac",
        "fell_by_tilt_frac",
    ]
    print(
        f"\npolicy={args_cli.policy}  v_cmd={args_cli.v_cmd:+.2f} m/s  "
        f"terrain={args_cli.terrain}  pin_yaw_ref={args_cli.pin_yaw_ref}"
    )
    print("w_cmd " + "".join(f"{c:>20s}" for c in cols))
    for w_cmd in args_cli.yaw_rates:
        live_cfg = env.unwrapped.cfg
        live_cfg.curriculum_stage = 5
        live_cfg.forced_velocity_cmd_mps = args_cli.v_cmd
        live_cfg.forced_yaw_rate_cmd_radps = w_cmd
        m = _sweep_point(policy, env, args_cli.num_steps, settle_steps, args_cli.pin_yaw_ref)
        print(f"{w_cmd:5.2f} " + "".join(f"{m[c]:20.3f}" for c in cols))
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
