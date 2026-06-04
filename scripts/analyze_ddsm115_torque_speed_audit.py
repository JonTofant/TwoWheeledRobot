#!/usr/bin/env python3
"""Validate DDSM115 torque-speed limiter inputs and generate audit plots."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

TAU_PEAK_NM = 2.0
NO_LOAD_SPEED_RAD_S = 200.0 * 2.0 * math.pi / 60.0

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("csv", nargs="+", type=Path)
parser.add_argument("--output-dir", type=Path, default=Path("outputs/ddsm115_torque_speed_audit"))
args = parser.parse_args()


def load(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise ValueError(f"Empty CSV: {path}")
    return {name: np.asarray([float(row[name]) for row in rows]) for name in rows[0]}


def max_abs(values: np.ndarray) -> float:
    return float(np.max(np.abs(values)))


args.output_dir.mkdir(parents=True, exist_ok=True)
summary_rows: list[dict[str, str]] = []
for path in args.csv:
    data = load(path)
    expected_left = np.clip(
        TAU_PEAK_NM * (1.0 - data["left_omega_for_limiter_rad_s"] / NO_LOAD_SPEED_RAD_S),
        0.0,
        TAU_PEAK_NM,
    )
    expected_right = np.clip(
        TAU_PEAK_NM * (1.0 - data["right_omega_for_limiter_rad_s"] / NO_LOAD_SPEED_RAD_S),
        0.0,
        TAU_PEAK_NM,
    )
    equation_residual = max(
        max_abs(expected_left - data["left_tau_speed_limit_nm"]),
        max_abs(expected_right - data["right_tau_speed_limit_nm"]),
    )
    raw_used_residual = max(
        max_abs(data["left_wheel_velocity_raw_rad_s"] - data["left_wheel_velocity_used_rad_s"]),
        max_abs(data["right_wheel_velocity_raw_rad_s"] - data["right_wheel_velocity_used_rad_s"]),
    )
    rpm_conversion_residual = max(
        max_abs(data["left_wheel_velocity_used_rpm"] - data["left_wheel_velocity_used_rad_s"] * 60.0 / (2.0 * math.pi)),
        max_abs(data["right_wheel_velocity_used_rpm"] - data["right_wheel_velocity_used_rad_s"] * 60.0 / (2.0 * math.pi)),
    )
    time = data["time"]
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 10))
    axes[0].plot(time, data["left_wheel_rpm"], label="left post-step forward RPM")
    axes[0].plot(time, data["right_wheel_rpm"], label="right post-step forward RPM")
    axes[0].set_ylabel("Post-step RPM")
    axes[1].plot(time, data["left_wheel_velocity_used_rpm"], label="left limiter raw RPM")
    axes[1].plot(time, data["right_wheel_velocity_used_rpm"], label="right limiter raw RPM")
    axes[1].set_ylabel("Limiter-input RPM")
    axes[2].plot(time, data["left_tau_speed_limit_nm"], label="left torque limit")
    axes[2].plot(time, data["right_tau_speed_limit_nm"], label="right torque limit")
    axes[2].set_ylabel("Torque limit [Nm]")
    axes[2].set_xlabel("Time [s]")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    fig.suptitle(path.stem)
    fig.tight_layout()
    plot_path = args.output_dir / f"{path.stem}.png"
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)
    summary_rows.append({
        "csv": str(path),
        "plot": str(plot_path),
        "max_equation_residual_nm": f"{equation_residual:.12g}",
        "max_raw_used_residual_rad_s": f"{raw_used_residual:.12g}",
        "max_rpm_conversion_residual": f"{rpm_conversion_residual:.12g}",
        "max_post_step_rpm_difference": f"{max_abs(data['right_wheel_rpm'] - data['left_wheel_rpm']):.9f}",
        "max_limiter_omega_difference_rad_s": f"{max_abs(data['right_omega_for_limiter_rad_s'] - data['left_omega_for_limiter_rad_s']):.9f}",
        "max_tau_limit_difference_nm": f"{max_abs(data['right_tau_speed_limit_nm'] - data['left_tau_speed_limit_nm']):.9f}",
        "final_left_post_step_rpm": f"{data['left_wheel_rpm'][-1]:.9f}",
        "final_right_post_step_rpm": f"{data['right_wheel_rpm'][-1]:.9f}",
    })

summary_path = args.output_dir / "summary.csv"
with summary_path.open("w", newline="") as file:
    writer = csv.DictWriter(file, fieldnames=list(summary_rows[0]))
    writer.writeheader()
    writer.writerows(summary_rows)
print(f"Wrote {summary_path}")
