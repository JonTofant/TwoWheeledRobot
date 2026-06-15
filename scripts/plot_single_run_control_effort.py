#!/usr/bin/env python3
"""Plot final current-command effort for one repeatability run CSV."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_lqr_disturbance_sweep as sweep


FINAL_COMMAND_COLUMN_CANDIDATES = [
    ("u_left_cmd_a", "u_right_cmd_a"),
    ("u_left_final_a", "u_right_final_a"),
    ("u_left_final", "u_right_final"),
    ("left_i_cmd_a", "right_i_cmd_a"),
    ("i_cmd_left", "i_cmd_right"),
    ("left_current_a", "right_current_a"),
]
RESIDUAL_COLUMN_CANDIDATES = [
    ("u_left_rl_a", "u_right_rl_a"),
    ("u_left_rl", "u_right_rl"),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path, help="Raw run CSV to plot.")
    parser.add_argument("--output", type=Path, default=None, help="Output PNG path for the current-command plot.")
    parser.add_argument("--title", default="Control Effort Over Time")
    parser.add_argument("--disturbance-start-s", type=float, default=None)
    parser.add_argument("--disturbance-samples", type=int, default=None)
    parser.add_argument("--sample-time-s", type=float, default=None)
    parser.add_argument("--residual-output", type=Path, default=None, help="Optional residual-current PNG output path.")
    return parser.parse_args()


def find_pair(df: pd.DataFrame, candidates: list[tuple[str, str]], required: bool) -> tuple[str, str] | None:
    for left, right in candidates:
        if left in df and right in df:
            return left, right
    if required:
        missing = ", ".join(candidates[0])
        raise KeyError(f"Missing required columns: {missing}")
    return None


def disturbance_intervals(
    df: pd.DataFrame,
    times: pd.Series,
    start_s: float | None,
    samples: int | None,
    sample_time_s: float | None,
) -> list[tuple[float, float]]:
    active = None
    if "actuator_disturbance_active" in df:
        active = df["actuator_disturbance_active"].astype(float).to_numpy() > 0.5
    elif "disturbance_left_a" in df and "disturbance_right_a" in df:
        left = df["disturbance_left_a"].astype(float).to_numpy()
        right = df["disturbance_right_a"].astype(float).to_numpy()
        active = (np.abs(left) > 1.0e-12) | (np.abs(right) > 1.0e-12)

    if active is not None and bool(np.any(active)):
        t = times.astype(float).to_numpy()
        intervals: list[tuple[float, float]] = []
        indices = np.flatnonzero(active)
        starts = [indices[0]]
        stops = []
        for prev, cur in zip(indices[:-1], indices[1:]):
            if cur != prev + 1:
                stops.append(prev)
                starts.append(cur)
        stops.append(indices[-1])
        dt = float(np.mean(np.diff(t))) if len(t) > 1 else 0.0
        for start_idx, stop_idx in zip(starts, stops):
            intervals.append((float(t[start_idx]), float(t[stop_idx] + max(dt, 0.0))))
        return intervals

    if start_s is not None and samples is not None and sample_time_s is not None:
        return [(start_s, start_s + samples * sample_time_s)]
    return []


def shade_disturbance(axis: plt.Axes, intervals: list[tuple[float, float]]) -> None:
    for idx, (start_s, stop_s) in enumerate(intervals):
        axis.axvspan(
            start_s,
            stop_s,
            color="tab:red",
            alpha=0.14,
            label="disturbance active" if idx == 0 else None,
        )


def default_output(csv_path: Path) -> Path:
    return csv_path.with_name(f"{csv_path.stem}_control_effort.png")


def main() -> None:
    args = parse_args()
    df = pd.read_csv(args.csv_path)
    if df.empty:
        raise ValueError(f"Empty CSV: {args.csv_path}")

    times = sweep.time_s(df)
    command_pair = find_pair(df, FINAL_COMMAND_COLUMN_CANDIDATES, required=True)
    assert command_pair is not None
    left_col, right_col = command_pair
    left = df[left_col].astype(float)
    right = df[right_col].astype(float)
    combined = np.sqrt(np.square(left) + np.square(right))
    intervals = disturbance_intervals(
        df,
        times,
        args.disturbance_start_s,
        args.disturbance_samples,
        args.sample_time_s,
    )

    output = args.output or default_output(args.csv_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axis = plt.subplots(figsize=(10, 5.5))
    shade_disturbance(axis, intervals)
    axis.plot(times, left, label="left final current command", linewidth=1.5)
    axis.plot(times, right, label="right final current command", linewidth=1.5)
    axis.plot(times, combined, label="combined current effort", linewidth=1.8, color="black")
    axis.set_title(args.title)
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Current command [A]")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)
    print(f"Wrote {output}")

    residual_pair = find_pair(df, RESIDUAL_COLUMN_CANDIDATES, required=False)
    if residual_pair is None:
        return
    residual_output = args.residual_output or output.with_name("residual_action_single_run.png")
    residual_output.parent.mkdir(parents=True, exist_ok=True)
    left_rl_col, right_rl_col = residual_pair
    fig, axis = plt.subplots(figsize=(10, 5.0))
    shade_disturbance(axis, intervals)
    axis.plot(times, df[left_rl_col].astype(float), label="left residual current", linewidth=1.5)
    axis.plot(times, df[right_rl_col].astype(float), label="right residual current", linewidth=1.5)
    axis.set_title("Residual PPO Current Over Time")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Residual current [A]")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    fig.tight_layout()
    fig.savefig(residual_output, dpi=180)
    plt.close(fig)
    print(f"Wrote {residual_output}")


if __name__ == "__main__":
    main()
