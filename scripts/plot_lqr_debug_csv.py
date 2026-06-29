#!/usr/bin/env python3
"""Plot detailed LQR force/current decomposition from lqr_control.py CSV logs."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="CSV produced by scripts/lqr_control.py")
    parser.add_argument("--output", type=Path, default=None, help="Output PNG path")
    return parser.parse_args()


def require_columns(df: pd.DataFrame, columns: list[str]) -> None:
    missing = [column for column in columns if column not in df]
    if missing:
        raise KeyError(f"CSV is missing required columns: {', '.join(missing)}")


def main() -> int:
    args = parse_args()
    df = pd.read_csv(args.csv)
    time_col = "time_s" if "time_s" in df else "time"
    columns = [
        time_col,
        "theta", "theta_dot", "velocity", "yaw",
        "lqr_state_left_velocity_m_s", "lqr_state_right_velocity_m_s",
        "force_left_position_n", "force_left_velocity_n", "force_left_pitch_n", "force_left_pitch_rate_n", "force_left_lqr_physical_n",
        "force_right_position_n", "force_right_velocity_n", "force_right_pitch_n", "force_right_pitch_rate_n", "force_right_lqr_physical_n",
        "force_total_phys_n",
        "current_left_after_lqr_before_disturbance_a", "current_right_after_lqr_before_disturbance_a",
        "current_left_final_command_a", "current_right_final_command_a",
        "motor_current_saturated_left_a", "motor_current_saturated_right_a",
    ]
    require_columns(df, columns)
    t = df[time_col].astype(float)

    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(12, 14))

    axes[0].plot(t, df["theta"], label="theta")
    axes[0].plot(t, df["theta_dot"], label="theta_dot")
    axes[0].plot(t, df["velocity"], label="avg wheel velocity m/s")
    axes[0].plot(t, df["yaw"], label="yaw")
    axes[0].set_ylabel("state")
    axes[0].legend(loc="upper left")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, df["lqr_state_left_velocity_m_s"], label="left v state")
    axes[1].plot(t, df["lqr_state_right_velocity_m_s"], label="right v state")
    axes[1].set_ylabel("m/s")
    axes[1].legend(loc="upper left")
    axes[1].grid(True, alpha=0.3)

    for prefix, style in [("left", "-"), ("right", "--")]:
        axes[2].plot(t, df[f"force_{prefix}_position_n"], style, label=f"{prefix} position")
        axes[2].plot(t, df[f"force_{prefix}_velocity_n"], style, label=f"{prefix} velocity")
        axes[2].plot(t, df[f"force_{prefix}_pitch_n"], style, label=f"{prefix} pitch")
        axes[2].plot(t, df[f"force_{prefix}_pitch_rate_n"], style, label=f"{prefix} pitch_rate")
    axes[2].set_ylabel("force terms [N]")
    axes[2].legend(loc="upper left", ncol=2, fontsize=8)
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t, df["force_left_lqr_physical_n"], label="left wheel force")
    axes[3].plot(t, df["force_right_lqr_physical_n"], label="right wheel force")
    axes[3].plot(t, df["force_total_phys_n"], label="left+right total force", linewidth=2.0, color="black")
    axes[3].set_ylabel("total force [N]")
    axes[3].legend(loc="upper left")
    axes[3].grid(True, alpha=0.3)

    axes[4].plot(t, df["current_left_after_lqr_before_disturbance_a"], label="left LQR current")
    axes[4].plot(t, df["current_right_after_lqr_before_disturbance_a"], label="right LQR current")
    axes[4].plot(t, df["current_left_final_command_a"], "--", label="left final command")
    axes[4].plot(t, df["current_right_final_command_a"], "--", label="right final command")
    axes[4].plot(t, df["motor_current_saturated_left_a"], ":", label="left motor saturated")
    axes[4].plot(t, df["motor_current_saturated_right_a"], ":", label="right motor saturated")
    axes[4].set_ylabel("current [A]")
    axes[4].set_xlabel("time [s]")
    axes[4].legend(loc="upper left", ncol=2, fontsize=8)
    axes[4].grid(True, alpha=0.3)

    fig.tight_layout()
    output = args.output or args.csv.with_name(f"{args.csv.stem}_lqr_debug.png")
    fig.savefig(output, dpi=160)
    print(f"Wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
