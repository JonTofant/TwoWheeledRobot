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
parser.add_argument(
    "--num_envs",
    type=int,
    default=1,
    help="Accepted for compatibility; the LQR controller always runs one environment.",
)
parser.add_argument(
    "--task",
    type=str,
    default="Template-Twowheeledrobot-Standup-v0",
    help="Name of the task.",
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment.")
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()

args_cli.headless = False
args_cli.num_envs = 1

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

    # ---------------------------------------------------------------------
    # USER CONTROLLER SECTION
    # ---------------------------------------------------------------------
    # This block is intentionally written like a small C control loop.
    # Change signs, gains, and the u calculation here. Everything below this
    # block only translates your wheel currents into the Isaac Lab action.

    # 1) Raw states from the simulator. These are scalar tensors for env_0.
    raw_roll = projected_gravity_b[0, 0]
    raw_pitch = projected_gravity_b[0, 1]
    raw_roll_rate = ang_vel_b[0, 0]
    raw_pitch_rate = ang_vel_b[0, 1]
    raw_left_wheel_velocity = wheel_vel[0, 0]
    raw_right_wheel_velocity = wheel_vel[0, 1]

    # 2) State sign transforms. Flip any value between +1.0 and -1.0 here.
    roll = +1.0 * raw_roll
    pitch = +1.0 * raw_pitch
    roll_rate = +1.0 * raw_roll_rate
    pitch_rate = +1.0 * raw_pitch_rate
    left_wheel_velocity = +1.0 * raw_left_wheel_velocity
    right_wheel_velocity = +1.0 * raw_right_wheel_velocity
    wheel_velocity = 0.5 * (left_wheel_velocity + right_wheel_velocity)

    # 3) Gains. Edit these numbers directly while tuning.
    K_PITCH = 2.0
    K_PITCH_RATE = 0.35
    K_WHEEL_VELOCITY = 0.08
    K_ROLL = 0.0
    K_ROLL_RATE = 0.0

    # 4) Controller calculation. Put your LQR / PID equation here.
    u_forward = -(
        K_PITCH * pitch
        + K_PITCH_RATE * pitch_rate
        + K_WHEEL_VELOCITY * wheel_velocity
    )
    u_turn = -(K_ROLL * roll + K_ROLL_RATE * roll_rate)

    # 5) Translate controller output to left/right wheel current commands.
    # If both wheels move backward when they should move forward, flip both
    # WHEEL_ACTION_SIGN values. If turning is reversed, flip the sign of u_turn.
    LEFT_WHEEL_ACTION_SIGN = +1.0
    RIGHT_WHEEL_ACTION_SIGN = +1.0

    left_current = LEFT_WHEEL_ACTION_SIGN * (u_forward - u_turn)
    right_current = RIGHT_WHEEL_ACTION_SIGN * (u_forward + u_turn)
    # ---------------------------------------------------------------------
    # END USER CONTROLLER SECTION
    # ---------------------------------------------------------------------

    left_current = left_current.clamp(-1.0, 1.0).reshape(1)
    right_current = right_current.clamp(-1.0, 1.0).reshape(1)

    # The CyberGear joints use asymmetric mechanical limits, so a single
    # normalized action value does not correspond to 0 rad on every joint.
    # Convert the zero-angle target directly into the normalized action space
    # used by StandupEnv so the legs are commanded to stay at 0 rad.
    cg_zero_targets = torch.zeros_like(env_unwrapped._cg_joint_lo)
    cg_actions = (
        2.0 * (cg_zero_targets - env_unwrapped._cg_joint_lo)
        / (env_unwrapped._cg_joint_hi - env_unwrapped._cg_joint_lo)
        - 1.0
    ).clamp(-1.0, 1.0)
    wheel_actions = torch.stack((left_current, right_current), dim=1)
    return torch.cat((cg_actions, wheel_actions), dim=1)


@hydra_task_config(args_cli.task, None)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, _agent_cfg: Any | None):
    """Run the deterministic balance controller."""
    env_cfg.scene.num_envs = 1
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
