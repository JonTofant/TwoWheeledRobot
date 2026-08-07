"""Shared deterministic benchmark scenarios and curriculum gate rules."""

from __future__ import annotations

import math
from typing import Any

# name: (curriculum stage, forced disturbance, velocity command, yaw-rate command)
SCENARIOS: dict[str, tuple[int, str, float, float]] = {
    "station_keeping": (1, "none", 0.0, 0.0),
    "drive_forward_slow": (1, "none", 0.10, 0.0),
    "drive_backward_slow": (1, "none", -0.10, 0.0),
    "drive_forward": (2, "none", 0.30, 0.0),
    "drive_backward": (2, "none", -0.30, 0.0),
    "turn_in_place": (2, "none", 0.0, 0.40),
    "drive_and_turn": (2, "none", 0.25, 0.35),
    "push_while_still": (3, "human_push", 0.0, 0.0),
    "push_while_driving": (3, "human_push", 0.30, 0.0),
    "payload_while_still": (4, "payload", 0.0, 0.0),
    "payload_while_driving": (4, "payload", 0.35, 0.0),
}

STAGE_GATE_SCENARIOS: dict[int, tuple[str, ...]] = {
    1: ("station_keeping", "drive_forward_slow", "drive_backward_slow"),
    2: ("station_keeping", "drive_forward", "drive_backward", "turn_in_place", "drive_and_turn"),
    3: (
        "station_keeping",
        "drive_forward",
        "drive_backward",
        "turn_in_place",
        "drive_and_turn",
        "push_while_still",
        "push_while_driving",
    ),
    4: tuple(SCENARIOS),
    5: tuple(SCENARIOS),
}


def checkpoint_score(results: dict[str, dict[str, float]]) -> float:
    """Lower-is-better score used only after the hard stage gate passes."""
    scores = []
    for name, metrics in results.items():
        _, _, velocity_cmd, yaw_rate_cmd = SCENARIOS[name]
        score = (
            100.0 * metrics["termination_rate"]
            + 0.10 * metrics["rms_pitch_deg"]
            + 0.20 * metrics["rms_roll_deg"]
            + 4.0 * metrics["rms_vel_err_mps"]
            + metrics["rms_yaw_rate_err_radps"]
        )
        if velocity_cmd == 0.0 and yaw_rate_cmd == 0.0:
            score += 2.0 * metrics["world_drift_m"] + 4.0 * abs(metrics["mean_velocity_mps"])
        scores.append(score)
    return sum(scores) / max(len(scores), 1)


def stage_gate_failures(stage: int, results: dict[str, dict[str, float]]) -> list[str]:
    """Return concrete reasons a checkpoint is unsafe to promote."""
    failures: list[str] = []
    required = STAGE_GATE_SCENARIOS[stage]
    missing = [name for name in required if name not in results]
    if missing:
        return [f"missing scenarios: {', '.join(missing)}"]

    for name in required:
        metrics = results[name]
        _, _, velocity_cmd, yaw_rate_cmd = SCENARIOS[name]
        non_finite = [key for key, value in metrics.items() if not math.isfinite(value)]
        if non_finite:
            failures.append(f"{name}: non-finite metrics: {', '.join(sorted(non_finite))}")
            continue
        if metrics["sampled_env_fraction"] < 0.90:
            failures.append(
                f"{name}: sampled_env_fraction {metrics['sampled_env_fraction']:.3f} < 0.900"
            )
        if metrics["termination_rate"] > 0.10:
            failures.append(f"{name}: termination_rate {metrics['termination_rate']:.3f} > 0.100")
        if metrics["rms_pitch_deg"] > 8.0:
            failures.append(f"{name}: rms_pitch_deg {metrics['rms_pitch_deg']:.2f} > 8.00")
        if metrics["rms_roll_deg"] > 5.0:
            failures.append(f"{name}: rms_roll_deg {metrics['rms_roll_deg']:.2f} > 5.00")

        if velocity_cmd == 0.0 and yaw_rate_cmd == 0.0:
            if abs(metrics["mean_velocity_mps"]) > 0.10:
                failures.append(f"{name}: |mean_velocity_mps| {abs(metrics['mean_velocity_mps']):.3f} > 0.100")
            if metrics["world_drift_m"] > 0.50:
                failures.append(f"{name}: world_drift_m {metrics['world_drift_m']:.3f} > 0.500")
        if velocity_cmd != 0.0:
            max_vel_error = max(0.12, 0.65 * abs(velocity_cmd))
            if metrics["rms_vel_err_mps"] > max_vel_error:
                failures.append(f"{name}: rms_vel_err_mps {metrics['rms_vel_err_mps']:.3f} > {max_vel_error:.3f}")
            if metrics["mean_velocity_mps"] * velocity_cmd <= 0.0:
                failures.append(f"{name}: mean velocity has the wrong sign or is zero")
        if yaw_rate_cmd != 0.0:
            max_yaw_error = max(0.25, 0.65 * abs(yaw_rate_cmd))
            if metrics["rms_yaw_rate_err_radps"] > max_yaw_error:
                failures.append(
                    f"{name}: rms_yaw_rate_err_radps {metrics['rms_yaw_rate_err_radps']:.3f} > {max_yaw_error:.3f}"
                )
            if metrics["mean_yaw_rate_radps"] * yaw_rate_cmd <= 0.0:
                failures.append(f"{name}: mean yaw rate has the wrong sign or is zero")
    return failures


def benchmark_payload(
    *,
    policy: str,
    seed: int,
    terrain: str,
    num_envs: int,
    num_steps: int,
    scenarios: dict[str, dict[str, float]],
) -> dict[str, Any]:
    return {
        "policy": policy,
        "seed": seed,
        "terrain": terrain,
        "num_envs": num_envs,
        "num_steps": num_steps,
        "scenarios": scenarios,
    }
