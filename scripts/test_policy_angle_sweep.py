#!/usr/bin/env python3
"""Feed synthetic angle sweeps into the stand-up policy and print commands.

This is a host-side sanity check for the UART policy runner. It does not need
the robot or Isaac Sim. It loads an exported JIT policy.pt, creates fake UART
states with changing roll/pitch angles, and prints the resulting CyberGear
targets and wheel current commands.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import torch

from uart_policy_runner import RobotState, actions_to_commands, build_observation


def deg(value: float) -> float:
    return math.radians(value)


def default_sweep() -> list[tuple[str, float, float]]:
    """Representative roll/pitch poses in degrees."""
    return [
        ("upright", 0.0, 0.0),
        ("small roll right", -8.0, 0.0),
        ("small roll left", 8.0, 0.0),
        ("small pitch fwd", 0.0, 10.0),
        ("small pitch back", 0.0, -10.0),
        ("right side", -90.0, 0.0),
        ("left side", 90.0, 0.0),
        ("fallen forward", 0.0, 90.0),
        ("fallen backward", 0.0, -90.0),
        ("diagonal fall", -55.0, 35.0),
        ("upside down", 180.0, 0.0),
    ]


def make_state(roll_deg: float, pitch_deg: float, gyro_scale: float) -> RobotState:
    """Create a fake UART state.

    The gyro is a small synthetic value in the same direction as the angle so
    the policy sees motion instead of a perfectly static impossible pose.
    """
    roll = deg(roll_deg)
    pitch = deg(pitch_deg)
    gyro = [
        deg(roll_deg) * gyro_scale,
        deg(pitch_deg) * gyro_scale,
        0.0,
    ]
    return RobotState(
        roll=roll,
        pitch=pitch,
        yaw=0.0,
        gyro=gyro,
        cg=[0.0, 0.0, 0.0, 0.0],
        ddsm=[0.0, 0.0],
    )


def fmt(values: list[float], precision: int = 3) -> str:
    return "[" + ", ".join(f"{value:+.{precision}f}" for value in values) + "]"


def main() -> int:
    parser = argparse.ArgumentParser(description="Print policy outputs for a roll/pitch sweep.")
    parser.add_argument("--policy", required=True, help="Path to exported JIT policy.pt")
    parser.add_argument(
        "--gyro-scale",
        type=float,
        default=0.0,
        help="Synthetic gyro = angle_rad * gyro_scale. Default keeps gyro at zero.",
    )
    parser.add_argument(
        "--carry-prev-action",
        action="store_true",
        help="Feed each output action into the next observation's previous-action field.",
    )
    args = parser.parse_args()

    policy_path = Path(args.policy)
    policy = torch.jit.load(str(policy_path), map_location="cpu")
    policy.eval()

    prev_action = torch.zeros(6, dtype=torch.float32)

    print(f"policy: {policy_path}")
    print("angles are deg, cg_target is rad, wheel_current is A")
    print()
    print(
        f"{'case':<18} {'roll':>7} {'pitch':>7}  "
        f"{'action':<48} {'cg_target':<38} {'wheel_current'}"
    )
    print("-" * 140)

    for name, roll_deg, pitch_deg in default_sweep():
        state = make_state(roll_deg, pitch_deg, args.gyro_scale)
        obs = build_observation(state, prev_action)

        with torch.inference_mode():
            action = policy(obs).squeeze(0).detach().cpu().to(torch.float32).clamp(-1.0, 1.0)

        cg_target, wheel_current = actions_to_commands(action)
        print(
            f"{name:<18} {roll_deg:>+7.1f} {pitch_deg:>+7.1f}  "
            f"{fmt(action.tolist()):<48} {fmt(cg_target):<38} {fmt(wheel_current)}"
        )

        if args.carry_prev_action:
            prev_action = action

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
