# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Run a deterministic LQR-style balance controller in Isaac Sim.

This script follows the launch and Hydra task-config pattern used by
``scripts/rsl_rl/play.py`` but intentionally does not import or instantiate any
RSL-RL runner or vec-env wrapper.  It directly reads the StandupEnv robot state
and writes the six-dimensional action format expected by the environment.
"""

from __future__ import annotations

"""Launch Isaac Sim Simulator first."""

import argparse
import sys
from pathlib import Path

_EXTENSION_SOURCE_PATH = Path(__file__).resolve().parents[1] / "source" / "TwoWheeledRobot"
if _EXTENSION_SOURCE_PATH.is_dir():
    sys.path.insert(0, str(_EXTENSION_SOURCE_PATH))

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Run a deterministic LQR-style controller for the two-wheeled robot.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment.")
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")
parser.add_argument("--k-pitch", type=float, default=2.0, help="Pitch error feedback gain.")
parser.add_argument("--k-pitch-rate", type=float, default=0.35, help="Pitch angular-rate feedback gain.")
parser.add_argument("--k-wheel-vel", type=float, default=0.08, help="Wheel velocity damping feedback gain.")
parser.add_argument("--k-roll", type=float, default=0.0, help="Optional differential roll feedback gain.")
parser.add_argument("--k-roll-rate", type=float, default=0.0, help="Optional differential roll-rate feedback gain.")
parser.add_argument(
    "--cg-neutral-action",
    type=float,
    default=-1.0,
    help="Neutral CyberGear action for channels [0:4]; -1 maps to mechanical zero/retracted.",
)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import time
from typing import Any

import gymnasium as gym
import torch
import TwoWheeledRobot.tasks  # noqa: F401

from isaaclab.envs import DirectMARLEnvCfg, DirectRLEnvCfg, ManagerBasedRLEnvCfg

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config


def _as_done_tensor(value: Any, device: torch.device) -> torch.Tensor:
    """Convert Gymnasium done outputs to a boolean tensor on the env device."""
    if isinstance(value, torch.Tensor):
        return value.to(device=device, dtype=torch.bool)
    return torch.as_tensor(value, device=device, dtype=torch.bool)


def _reset_done_envs(env: gym.Env, dones: torch.Tensor) -> None:
    """Reset completed environments when the backend exposes indexed resets.

    Isaac Lab direct environments normally perform the per-environment reset as
    part of ``env.step``.  This helper keeps the controller explicit and also
    supports backends that expose a public or private indexed reset entry point.
    """
    if not torch.any(dones):
        return

    unwrapped = env.unwrapped
    env_ids = (
        torch.tensor([0], device=dones.device, dtype=torch.long)
        if dones.ndim == 0
        else torch.nonzero(dones, as_tuple=False).squeeze(-1)
    )

    if hasattr(unwrapped, "reset_idx"):
        unwrapped.reset_idx(env_ids)
    elif hasattr(unwrapped, "_reset_idx"):
        unwrapped._reset_idx(env_ids)  # noqa: SLF001 - Isaac Lab indexed reset hook.
    else:
        env.reset()


def _compute_lqr_actions(env_unwrapped: Any) -> torch.Tensor:
    """Build the six-dimensional action tensor from raw simulator state."""
    projected_gravity_b = env_unwrapped.bno080.data.projected_gravity_b
    ang_vel_b = env_unwrapped.bno080.data.ang_vel_b
    wheel_vel = env_unwrapped.robot.data.joint_vel[:, env_unwrapped._wheel_ids]
    cg_pos = env_unwrapped.robot.data.joint_pos[:, env_unwrapped._cg_ids]

    # Simple balance state: gravity projection approximates small roll/pitch
    # error around the upright pose, angular velocity damps body motion,
    # CyberGear positions keep the measured leg state available for tuning, and
    # wheel velocity feedback discourages runaway wheel spin.
    balance_state = torch.cat(
        (
            projected_gravity_b[:, 0:2],
            ang_vel_b[:, 0:2],
            cg_pos,
            wheel_vel,
        ),
        dim=1,
    )
    roll_error = balance_state[:, 0]
    pitch_error = balance_state[:, 1]
    roll_rate = balance_state[:, 2]
    pitch_rate = balance_state[:, 3]
    left_wheel_vel = balance_state[:, 8]
    right_wheel_vel = balance_state[:, 9]
    mean_wheel_vel = 0.5 * (left_wheel_vel + right_wheel_vel)

    forward_current = -(
        args_cli.k_pitch * pitch_error
        + args_cli.k_pitch_rate * pitch_rate
        + args_cli.k_wheel_vel * mean_wheel_vel
    )
    turn_current = -(args_cli.k_roll * roll_error + args_cli.k_roll_rate * roll_rate)

    left_current = (forward_current - turn_current).clamp(-1.0, 1.0)
    right_current = (forward_current + turn_current).clamp(-1.0, 1.0)

    cg_actions = torch.full(
        (env_unwrapped.num_envs, 4),
        args_cli.cg_neutral_action,
        device=env_unwrapped.device,
        dtype=torch.float32,
    ).clamp(-1.0, 1.0)
    wheel_actions = torch.stack((left_current, right_current), dim=1)
    return torch.cat((cg_actions, wheel_actions), dim=1)


@hydra_task_config(args_cli.task, None)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, _agent_cfg: Any | None):
    """Run the deterministic balance controller."""
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
    env_cfg.seed = args_cli.seed if args_cli.seed is not None else env_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    env = gym.make(args_cli.task, cfg=env_cfg)
    dt = env.unwrapped.step_dt

    env.reset()
    timestep = 0
    print("[INFO]: LQR controller running. Press Ctrl+C to stop.")

    while simulation_app.is_running():
        start_time = time.time()
        with torch.inference_mode():
            actions = _compute_lqr_actions(env.unwrapped)
            _, _, terminated, truncated, _ = env.step(actions)
            dones = _as_done_tensor(terminated, env.unwrapped.device) | _as_done_tensor(truncated, env.unwrapped.device)
            _reset_done_envs(env, dones)

        timestep += 1
        if timestep % 100 == 0:
            proj_grav = env.unwrapped.bno080.data.projected_gravity_b[0].detach().cpu().tolist()
            wheel_action = actions[0, 4:6].detach().cpu().tolist()
            print(
                f"[step {timestep:5d}]  "
                f"proj_grav=[{proj_grav[0]:+.3f}, {proj_grav[1]:+.3f}, {proj_grav[2]:+.3f}]  "
                f"wheel_action=[{wheel_action[0]:+.3f}, {wheel_action[1]:+.3f}]"
            )


        sleep_time = dt - (time.time() - start_time)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

    env.close()


if __name__ == "__main__":
    main()  # type: ignore[reportCallIssue]
    simulation_app.close()
