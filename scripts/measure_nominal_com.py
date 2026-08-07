#!/usr/bin/env python3
"""Measure the nominal whole-robot COM relative to the wheel axis.

The fore/aft (Y) component decides whether "drive forward" and "drive backward"
are the same problem. If the whole-robot COM sits off the wheel axis in Y, the
robot has a permanent lean direction and the two are NOT equivalent — which
would be a property of the USD asset, not of the policy, and would need
declaring in the paper.

The lateral (X) component is the roll-axis equivalent and is reported alongside,
since the same argument applies to left/right.

Run with randomization disabled so this measures the asset, not a draw:
    env.com_offset_y_range_m=[0,0] env.com_offset_z_range_m=[0,0] \
    env.body_mass_scale_range=[1,1] env.reset_pitch_range_deg=0 \
    env.reset_pitch_rate_range_radps=0 env.reset_velocity_range_mps=0
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
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
import TwoWheeledRobot.tasks  # noqa: F401

import isaaclab.utils.math as math_utils

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg, agent_cfg):
    env_cfg.scene.num_envs = 2
    env_cfg.terrain_mode = "flat"
    env = gym.make(args_cli.task, cfg=env_cfg)
    u = env.unwrapped
    with torch.inference_mode():
        env.reset()
        for _ in range(3):  # let the drop settle so poses are the resting ones
            env.step(torch.zeros(u.num_envs, u.cfg.action_space, device=u.device))

        names = u.robot.body_names
        masses = u.robot.root_physx_view.get_masses()[0].to(u.device)
        coms_local = u.robot.root_physx_view.get_coms()[0][:, :3].to(u.device)
        body_pos_w = u.robot.data.body_pos_w[0]
        body_quat_w = u.robot.data.body_quat_w[0]

        # Each body's COM in world = body origin + R(body quat) * local COM.
        com_w = body_pos_w + math_utils.quat_apply(body_quat_w, coms_local)
        total_mass = masses.sum()
        robot_com_w = (masses.unsqueeze(1) * com_w).sum(dim=0) / total_mass

        wheel_rows = [i for i, n in enumerate(names) if "DDSM115" in n]
        wheel_axis_w = body_pos_w[wheel_rows].mean(dim=0)

        print(f"\ntotal mass = {total_mass.item():.4f} kg over {len(names)} bodies")
        print(f"\n{'body':28}{'mass_kg':>9}{'com_w_x':>10}{'com_w_y':>10}{'com_w_z':>10}")
        for i, n in enumerate(names):
            print(
                f"{n[:28]:28}{masses[i].item():9.4f}"
                f"{com_w[i, 0].item():10.4f}{com_w[i, 1].item():10.4f}{com_w[i, 2].item():10.4f}"
            )

        offset = robot_com_w - wheel_axis_w
        print(f"\nwheel axis (world)  x={wheel_axis_w[0].item():+.5f} "
              f"y={wheel_axis_w[1].item():+.5f} z={wheel_axis_w[2].item():+.5f}")
        print(f"robot COM  (world)  x={robot_com_w[0].item():+.5f} "
              f"y={robot_com_w[1].item():+.5f} z={robot_com_w[2].item():+.5f}")
        print("\nCOM offset from the wheel axis:")
        print(f"  lateral  X = {offset[0].item() * 1000:+8.2f} mm   (roll axis)")
        print(f"  fore/aft Y = {offset[1].item() * 1000:+8.2f} mm   (pitch axis — the one in question)")
        print(f"  height   Z = {offset[2].item() * 1000:+8.2f} mm   (pendulum length)")

        height = offset[2].item()
        if abs(height) > 1e-6:
            import math

            lean = math.degrees(math.atan2(offset[1].item(), height))
            print(f"\nimplied resting lean about the pitch axis = {lean:+.3f} deg")
            print(f"trim sensitivity = {math.degrees(math.atan2(0.001, height)):.3f} deg per mm of fore/aft offset")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
