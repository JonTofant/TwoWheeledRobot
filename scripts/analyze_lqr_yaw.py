#!/usr/bin/env python3
"""Generate repeatability metrics and static plots for LQR floor yaw tests."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("csv", nargs="+", type=Path, help="LQR floor-contact CSV logs")
parser.add_argument("--output-dir", type=Path, default=Path("outputs/lqr_yaw_analysis"))
args = parser.parse_args()


def load(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise ValueError(f"Empty CSV: {path}")
    return {name: np.asarray([float(row[name]) for row in rows]) for name in rows[0]}


def max_abs(values: np.ndarray) -> float:
    return float(np.max(np.abs(values)))


def corr(a: np.ndarray, b: np.ndarray) -> float:
    if np.std(a) < 1.0e-12 or np.std(b) < 1.0e-12:
        return 0.0
    return float(np.corrcoef(a, b)[0, 1])


def plot_run(path: Path, data: dict[str, np.ndarray], output_dir: Path) -> Path:
    time = data["time"]
    fig, axes = plt.subplots(6, 1, sharex=True, figsize=(11, 18))
    pairs = [
        (("theta", "theta_dot"), ("theta", "theta_dot"), "Pitch [rad, rad/s]"),
        (("base_yaw", "base_yaw_rate"), ("yaw", "yaw_rate"), "Yaw [rad, rad/s]"),
        (("left_wheel_rpm", "right_wheel_rpm"), ("left", "right"), "Wheel speed [rpm]"),
        (("i_cmd_left", "i_cmd_right"), ("left", "right"), "Current [A]"),
        (("tau_actual_left", "tau_actual_right"), ("left", "right"), "Torque [Nm]"),
        (("left_contact_force", "right_contact_force"), ("left", "right"), "Contact force [N]"),
    ]
    for axis, (signals, labels, ylabel) in zip(axes, pairs, strict=True):
        for signal, label in zip(signals, labels, strict=True):
            axis.plot(time, data[signal], label=label, linewidth=1.1)
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle(path.stem)
    fig.tight_layout()
    output_path = output_dir / f"{path.stem}_signals.png"
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path


args.output_dir.mkdir(parents=True, exist_ok=True)
summary_rows: list[dict[str, str]] = []
report_lines = ["# LQR Floor Yaw Repeatability Analysis", ""]
for run_number, path in enumerate(args.csv, start=1):
    data = load(path)
    yaw = np.unwrap(data["base_yaw"])
    yaw_delta = yaw - yaw[0]
    yaw_rate = data["base_yaw_rate"]
    theta = data["theta"]
    rpm_diff = data["rpm_diff"]
    torque_diff = data["torque_diff"]
    contact_diff = data["contact_force_diff"]
    current_diff = data["current_diff"]
    final_position = data["position"][-1]
    position_delta = final_position - data["position"][0]
    base_dx = data["base_position_x"][-1] - data["base_position_x"][0]
    base_dy = data["base_position_y"][-1] - data["base_position_y"][0]
    base_dz = data["base_position_z"][-1] - data["base_position_z"][0]
    plot_path = plot_run(path, data, args.output_dir)
    row = {
        "run": str(run_number),
        "csv": str(path),
        "plot": str(plot_path),
        "final_yaw_deg": f"{math.degrees(yaw_delta[-1]):.9f}",
        "max_abs_yaw_deg": f"{math.degrees(max_abs(yaw_delta)):.9f}",
        "max_abs_yaw_rate_deg_s": f"{math.degrees(max_abs(yaw_rate)):.9f}",
        "final_position_m": f"{final_position:.9f}",
        "position_change_m": f"{position_delta:.9f}",
        "final_base_x_m": f"{base_dx:.9f}",
        "final_base_y_m": f"{base_dy:.9f}",
        "final_base_z_m": f"{base_dz:.9f}",
        "max_abs_theta_deg": f"{math.degrees(max_abs(theta)):.9f}",
        "max_abs_rpm_diff": f"{max_abs(rpm_diff):.9f}",
        "max_abs_current_diff_a": f"{max_abs(current_diff):.9f}",
        "max_abs_torque_diff_nm": f"{max_abs(torque_diff):.9f}",
        "max_abs_contact_force_diff_n": f"{max_abs(contact_diff):.9f}",
        "corr_yaw_rate_rpm_diff": f"{corr(yaw_rate, rpm_diff):.9f}",
        "corr_yaw_rate_torque_diff": f"{corr(yaw_rate, torque_diff):.9f}",
        "corr_yaw_rate_contact_force_diff": f"{corr(yaw_rate, contact_diff):.9f}",
    }
    summary_rows.append(row)
    report_lines.extend([
        f"## Run {run_number}", "",
        f"- CSV: `{path}`",
        f"- Plot: `{plot_path}`",
        f"- Final yaw: `{row['final_yaw_deg']} deg`",
        f"- Maximum absolute yaw: `{row['max_abs_yaw_deg']} deg`",
        f"- Maximum absolute yaw rate: `{row['max_abs_yaw_rate_deg_s']} deg/s`",
        f"- Final position change: `{row['final_position_m']} m`",
        f"- Final base displacement: `({row['final_base_x_m']}, {row['final_base_y_m']}, {row['final_base_z_m']}) m`",
        f"- Maximum absolute theta: `{row['max_abs_theta_deg']} deg`",
        f"- Maximum absolute RPM difference: `{row['max_abs_rpm_diff']} rpm`",
        f"- Maximum absolute torque difference: `{row['max_abs_torque_diff_nm']} Nm`",
        f"- Maximum absolute contact-force difference: `{row['max_abs_contact_force_diff_n']} N`",
        "",
    ])

summary_path = args.output_dir / "repeatability_summary.csv"
with summary_path.open("w", newline="") as file:
    writer = csv.DictWriter(file, fieldnames=list(summary_rows[0]))
    writer.writeheader()
    writer.writerows(summary_rows)

final_yaws = np.asarray([float(row["final_yaw_deg"]) for row in summary_rows])
directions = np.sign(final_yaws[np.abs(final_yaws) > 1.0e-9])
consistent = len(directions) > 0 and np.all(directions == directions[0])
report_lines.extend([
    "## Repeatability", "",
    f"Yaw direction consistent across runs: `{'yes' if consistent else 'no'}`", "",
    "## Correlations", "",
    "Correlations below compare yaw rate with the signed asymmetry signals.", "",
])
for row in summary_rows:
    report_lines.append(
        f"- Run {row['run']}: RPM `{row['corr_yaw_rate_rpm_diff']}`, torque `{row['corr_yaw_rate_torque_diff']}`, contact force `{row['corr_yaw_rate_contact_force_diff']}`"
    )
report_lines.append("")
report_path = args.output_dir / "analysis.md"
report_path.write_text("\n".join(report_lines))
print(f"Wrote {summary_path}")
print(f"Wrote {report_path}")
