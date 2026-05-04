#!/usr/bin/env python3
"""Run the exported stand-up policy against a robot over UART.

STM32 -> host, one JSON object per line:
    {
      "roll": 0.0, "pitch": 1.57, "yaw": 0.0,
      "gyro": [0.0, 0.0, 0.0],
      "cg": [0.0, 0.0, 0.0, 0.0],
      "ddsm": [0.0, 0.0]
    }

Host -> STM32, one JSON object per line:
    {
      "cg_target": [fl, fr, bl, br],
      "wheel_current": [left, right],
      "action": [a0, a1, a2, a3, a4, a5]
    }

Angles are radians by default. Pass --degrees if the STM32 packet uses degrees.
The exported policy is expected to be the JIT file produced by scripts/rsl_rl/play.py.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from typing import Any

import serial
import torch


LO_EXT = math.radians(10.0)
LIM = math.pi / 2.0
CG_JOINT_LO = torch.tensor([-LO_EXT, -LIM, -LIM, -LO_EXT], dtype=torch.float32)
CG_JOINT_HI = torch.tensor([LIM, LO_EXT, LO_EXT, LIM], dtype=torch.float32)
CG_EXT_SIGN = torch.tensor([1.0, -1.0, -1.0, 1.0], dtype=torch.float32)
CG_NORM_SPAN = LO_EXT + LIM
WHEEL_CURRENT_MAX_A = 8.0


@dataclass
class RobotState:
    roll: float
    pitch: float
    yaw: float
    gyro: list[float]
    cg: list[float]
    ddsm: list[float]


def projected_gravity_from_roll_pitch(roll: float, pitch: float) -> torch.Tensor:
    """Convert roll/pitch to Isaac-style projected_gravity_b.

    StandupEnv trained with projected_gravity_b where upright is [0, 0, -1],
    roll shows up mostly in x, and pitch shows up mostly in y. Yaw does not
    affect gravity direction.

    If hardware signs do not match simulation, fix the sign at the UART/input
    layer before trusting the policy on the real robot.
    """
    sr = math.sin(roll)
    cr = math.cos(roll)
    sp = math.sin(pitch)
    cp = math.cos(pitch)
    return torch.tensor([-sr * cp, sp, -cr * cp], dtype=torch.float32)


def normalize_cg_positions(cg_pos: torch.Tensor) -> torch.Tensor:
    """Map CyberGear angles to the policy's zero-centered extension fractions."""
    cg_pos = torch.clamp(cg_pos, CG_JOINT_LO, CG_JOINT_HI)
    cg_shifted = cg_pos * CG_EXT_SIGN + LO_EXT
    return cg_shifted / CG_NORM_SPAN * 2.0 - 1.0


def actions_to_commands(actions: torch.Tensor) -> tuple[list[float], list[float]]:
    """Map policy actions in [-1, 1] to CyberGear targets and wheel currents."""
    actions = torch.clamp(actions, -1.0, 1.0)
    act01 = 0.5 * (actions[:4] + 1.0)
    cg_targets = CG_JOINT_LO + act01 * (CG_JOINT_HI - CG_JOINT_LO)
    wheel_current = [
        float(actions[4] * WHEEL_CURRENT_MAX_A),
        float(actions[5] * WHEEL_CURRENT_MAX_A),
    ]
    return cg_targets.tolist(), wheel_current


def parse_state(line: bytes, use_degrees: bool) -> RobotState:
    msg: dict[str, Any] = json.loads(line.decode("utf-8"))
    roll = float(msg["roll"])
    pitch = float(msg["pitch"])
    yaw = float(msg.get("yaw", msg.get("jaw", 0.0)))
    gyro = [float(x) for x in msg["gyro"]]
    cg = [float(x) for x in msg["cg"]]
    ddsm = [float(x) for x in msg.get("ddsm", [0.0, 0.0])]

    if use_degrees:
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        gyro = [math.radians(x) for x in gyro]
        cg = [math.radians(x) for x in cg]

    if len(gyro) != 3:
        raise ValueError("gyro must contain [wx, wy, wz]")
    if len(cg) != 4:
        raise ValueError("cg must contain [front_left, front_right, back_left, back_right]")
    if len(ddsm) != 2:
        raise ValueError("ddsm must contain [left_velocity, right_velocity]")

    return RobotState(roll=roll, pitch=pitch, yaw=yaw, gyro=gyro, cg=cg, ddsm=ddsm)


def build_observation(state: RobotState, prev_action: torch.Tensor) -> torch.Tensor:
    grav_3d = projected_gravity_from_roll_pitch(state.roll, state.pitch)
    ang_vel_norm = torch.tensor(state.gyro, dtype=torch.float32) / 10.0
    cg_norm = normalize_cg_positions(torch.tensor(state.cg, dtype=torch.float32))
    obs = torch.cat([grav_3d, ang_vel_norm, cg_norm, prev_action], dim=0)
    return torch.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0).clamp(-10.0, 10.0).unsqueeze(0)


def run(args: argparse.Namespace) -> int:
    policy = torch.jit.load(args.policy, map_location="cpu")
    policy.eval()

    prev_action = torch.zeros(6, dtype=torch.float32)
    next_tick = time.monotonic()

    with serial.Serial(args.port, args.baud, timeout=args.timeout) as uart:
        print(f"[uart_policy_runner] opened {args.port} @ {args.baud}", file=sys.stderr)
        while True:
            line = uart.readline()
            if not line:
                if args.fail_silent:
                    continue
                print("[uart_policy_runner] UART timeout", file=sys.stderr)
                continue

            try:
                state = parse_state(line, args.degrees)
                obs = build_observation(state, prev_action)
                with torch.inference_mode():
                    action = policy(obs).squeeze(0).detach().cpu().to(torch.float32)
                action = torch.clamp(action, -1.0, 1.0)
                cg_targets, wheel_current = actions_to_commands(action)
                prev_action = action

                response = {
                    "cg_target": cg_targets,
                    "wheel_current": wheel_current,
                    "action": action.tolist(),
                }
                if args.echo_ddsm:
                    response["ddsm"] = state.ddsm
                uart.write((json.dumps(response, separators=(",", ":")) + "\n").encode("utf-8"))

                if args.rate_hz > 0.0:
                    next_tick += 1.0 / args.rate_hz
                    sleep_s = next_tick - time.monotonic()
                    if sleep_s > 0.0:
                        time.sleep(sleep_s)
                    else:
                        next_tick = time.monotonic()
            except Exception as exc:
                print(f"[uart_policy_runner] bad packet: {exc}; raw={line!r}", file=sys.stderr)
                if args.safe_on_error:
                    safe = {
                        "cg_target": [0.0, 0.0, 0.0, 0.0],
                        "wheel_current": [0.0, 0.0],
                        "action": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    }
                    uart.write((json.dumps(safe, separators=(",", ":")) + "\n").encode("utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the stand-up policy over UART.")
    parser.add_argument("--policy", required=True, help="Path to exported JIT policy.pt")
    parser.add_argument("--port", required=True, help="Serial port, for example /dev/ttyACM0 or COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.1)
    parser.add_argument("--rate-hz", type=float, default=50.0, help="Optional host-side loop cap. Use 0 to disable.")
    parser.add_argument("--degrees", action="store_true", help="UART angles and gyro are degrees / deg/s.")
    parser.add_argument("--echo-ddsm", action="store_true", help="Echo DDSM velocities in the response for debugging.")
    parser.add_argument("--safe-on-error", action="store_true", help="Send zero-safe command after malformed packets.")
    parser.add_argument("--fail-silent", action="store_true", help="Do not print UART timeout messages.")
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
