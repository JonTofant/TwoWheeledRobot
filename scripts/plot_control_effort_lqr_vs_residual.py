#!/usr/bin/env python3
"""Plot LQR vs residual-PPO final current-command effort for one disturbance run."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_lqr_disturbance_sweep as sweep


FINAL_LEFT_COLUMN = "u_left_cmd_a"
FINAL_RIGHT_COLUMN = "u_right_cmd_a"
DEFAULT_DISTURBANCE_CURRENT_A = 1.5
DEFAULT_DISTURBANCE_START_S = 1.5
DEFAULT_DISTURBANCE_SAMPLES = 10
DEFAULT_SAMPLE_TIME_S = 0.02


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--summary", type=Path, help="per_run_summary.csv from the repeatability benchmark.")
    input_group.add_argument("--lqr-csv", type=Path, help="Raw LQR run CSV.")
    parser.add_argument("--ppo-csv", type=Path, help="Raw residual-PPO run CSV. Required with --lqr-csv.")
    parser.add_argument("--repeat-index", type=int, default=0, help="Repeat index to select from --summary.")
    parser.add_argument("--disturbance-current-a", type=float, default=DEFAULT_DISTURBANCE_CURRENT_A)
    parser.add_argument("--disturbance-start-s", type=float, default=DEFAULT_DISTURBANCE_START_S)
    parser.add_argument("--disturbance-samples", type=int, default=DEFAULT_DISTURBANCE_SAMPLES)
    parser.add_argument("--sample-time-s", type=float, default=DEFAULT_SAMPLE_TIME_S)
    parser.add_argument("--output", type=Path, required=True, help="Output PNG path for total effort.")
    parser.add_argument(
        "--decomposed-output",
        type=Path,
        default=None,
        help="Optional output PNG path for forward/yaw decomposed effort.",
    )
    parser.add_argument(
        "--save-decomposed",
        action="store_true",
        help="Also save a forward/yaw decomposed plot next to --output.",
    )
    return parser.parse_args()


def require_final_command_columns(df: pd.DataFrame, csv_path: Path) -> tuple[pd.Series, pd.Series]:
    missing = [column for column in (FINAL_LEFT_COLUMN, FINAL_RIGHT_COLUMN) if column not in df]
    if missing:
        available = ", ".join(df.columns)
        required = f"{FINAL_LEFT_COLUMN}, {FINAL_RIGHT_COLUMN}"
        raise KeyError(
            f"{csv_path} is missing final current command column(s): {', '.join(missing)}. "
            f"Required columns are {required}; available columns are: {available}"
        )
    return df[FINAL_LEFT_COLUMN].astype(float), df[FINAL_RIGHT_COLUMN].astype(float)


def mean_dt_s(times: pd.Series, fallback_dt_s: float) -> float:
    values = times.astype(float).to_numpy()
    diffs = np.diff(values)
    diffs = diffs[diffs > 0.0]
    if diffs.size:
        return float(np.mean(diffs))
    if fallback_dt_s <= 0.0:
        raise ValueError("Cannot compute dt from time column and --sample-time-s is not positive.")
    return float(fallback_dt_s)


def load_run(csv_path: Path, fallback_dt_s: float) -> dict[str, object]:
    df = pd.read_csv(csv_path)
    if df.empty:
        raise ValueError(f"Empty CSV: {csv_path}")
    times = sweep.time_s(df)
    left, right = require_final_command_columns(df, csv_path)
    left_np = left.to_numpy()
    right_np = right.to_numpy()
    squared_sum = np.square(left_np) + np.square(right_np)
    dt = mean_dt_s(times, fallback_dt_s)
    return {
        "csv_path": csv_path,
        "df": df,
        "times": times.astype(float),
        "left": left_np,
        "right": right_np,
        "effort": np.sqrt(squared_sum),
        "forward_effort": 0.5 * (left_np - right_np),
        "yaw_effort": 0.5 * (left_np + right_np),
        "integrated_effort": float(np.sum(squared_sum * dt)),
    }


def disturbance_intervals_from_active(df: pd.DataFrame, times: pd.Series) -> list[tuple[float, float]]:
    if "actuator_disturbance_active" not in df:
        return []
    active = df["actuator_disturbance_active"].astype(float).to_numpy() > 0.5
    if not bool(np.any(active)):
        return []

    time_values = times.astype(float).to_numpy()
    dt = float(np.mean(np.diff(time_values))) if len(time_values) > 1 else 0.0
    intervals: list[tuple[float, float]] = []
    active_indices = np.flatnonzero(active)
    start_idx = int(active_indices[0])
    prev_idx = start_idx
    for cur_idx in active_indices[1:]:
        cur_idx = int(cur_idx)
        if cur_idx != prev_idx + 1:
            intervals.append((float(time_values[start_idx]), float(time_values[prev_idx] + max(dt, 0.0))))
            start_idx = cur_idx
        prev_idx = cur_idx
    intervals.append((float(time_values[start_idx]), float(time_values[prev_idx] + max(dt, 0.0))))
    return intervals


def disturbance_intervals(
    lqr_run: dict[str, object],
    ppo_run: dict[str, object],
    start_s: float,
    samples: int,
    sample_time_s: float,
) -> list[tuple[float, float]]:
    for run in (lqr_run, ppo_run):
        intervals = disturbance_intervals_from_active(run["df"], run["times"])  # type: ignore[arg-type]
        if intervals:
            return intervals
    return [(start_s, start_s + samples * sample_time_s)]


def shade_disturbance(axis: plt.Axes, intervals: list[tuple[float, float]]) -> None:
    for index, (start_s, end_s) in enumerate(intervals):
        axis.axvspan(start_s, end_s, color="tab:red", alpha=0.14, label="Disturbance" if index == 0 else None)


def format_current_label(current_a: float) -> str:
    return f"{current_a:.1f}".replace(".", "p")


def default_decomposed_output(output: Path, current_a: float) -> Path:
    return output.with_name(f"control_effort_decomposed_{format_current_label(current_a)}A.png")


def resolve_csv_path(summary_path: Path, csv_value: object) -> Path:
    csv_path = Path(str(csv_value))
    if csv_path.is_absolute() or csv_path.exists():
        return csv_path
    summary_relative = summary_path.parent / csv_path
    if summary_relative.exists():
        return summary_relative
    return csv_path


def select_csv_from_summary(summary_path: Path, controller: str, current_a: float, repeat_index: int) -> Path:
    summary = pd.read_csv(summary_path)
    required_columns = {"controller", "disturbance_current_a", "repeat_index", "csv_path"}
    missing = sorted(required_columns.difference(summary.columns))
    if missing:
        raise KeyError(f"{summary_path} is missing required column(s): {', '.join(missing)}")

    currents = pd.to_numeric(summary["disturbance_current_a"], errors="coerce")
    repeats = pd.to_numeric(summary["repeat_index"], errors="coerce")
    mask = (
        (summary["controller"].astype(str) == controller)
        & np.isclose(currents.to_numpy(dtype=float), current_a, rtol=0.0, atol=1.0e-9)
        & (repeats == repeat_index)
    )
    matches = summary.loc[mask]
    if matches.empty:
        raise ValueError(
            f"No {controller} row found in {summary_path} for "
            f"disturbance_current_a={current_a:.9g}, repeat_index={repeat_index}."
        )
    if len(matches) > 1:
        raise ValueError(
            f"Found {len(matches)} {controller} rows in {summary_path} for "
            f"disturbance_current_a={current_a:.9g}, repeat_index={repeat_index}; expected one."
        )
    csv_value = matches.iloc[0]["csv_path"]
    if pd.isna(csv_value) or str(csv_value).strip() == "":
        raise ValueError(f"Selected {controller} row has an empty csv_path.")
    return resolve_csv_path(summary_path, csv_value)


def selected_csv_paths(args: argparse.Namespace) -> tuple[Path, Path]:
    if args.summary is not None:
        lqr_csv = select_csv_from_summary(args.summary, "lqr", args.disturbance_current_a, args.repeat_index)
        ppo_csv = select_csv_from_summary(args.summary, "residual_ppo", args.disturbance_current_a, args.repeat_index)
        return lqr_csv, ppo_csv
    if args.ppo_csv is None:
        raise ValueError("--ppo-csv is required when using --lqr-csv.")
    return args.lqr_csv, args.ppo_csv


def plot_total_effort(
    output: Path,
    lqr_run: dict[str, object],
    ppo_run: dict[str, object],
    intervals: list[tuple[float, float]],
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axis = plt.subplots(figsize=(10, 5.5))
    shade_disturbance(axis, intervals)
    axis.plot(lqr_run["times"], lqr_run["effort"], label="LQR", linewidth=1.8)
    axis.plot(ppo_run["times"], ppo_run["effort"], label="LQR + Residual PPO", linewidth=1.8)
    axis.set_title("Control Effort Over Time")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Instantaneous control effort [A]")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_decomposed_effort(
    output: Path,
    lqr_run: dict[str, object],
    ppo_run: dict[str, object],
    intervals: list[tuple[float, float]],
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 1, figsize=(10, 7.0), sharex=True)
    for axis in axes:
        shade_disturbance(axis, intervals)
        axis.grid(True, alpha=0.3)

    axes[0].plot(lqr_run["times"], lqr_run["forward_effort"], label="LQR", linewidth=1.6)
    axes[0].plot(ppo_run["times"], ppo_run["forward_effort"], label="LQR + Residual PPO", linewidth=1.6)
    axes[0].set_ylabel("Forward effort [A]")
    axes[0].legend(loc="best")

    axes[1].plot(lqr_run["times"], lqr_run["yaw_effort"], label="LQR", linewidth=1.6)
    axes[1].plot(ppo_run["times"], ppo_run["yaw_effort"], label="LQR + Residual PPO", linewidth=1.6)
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Yaw effort [A]")
    axes[1].legend(loc="best")

    fig.suptitle("Decomposed Control Effort Over Time")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def print_integrated_effort(lqr_j: float, ppo_j: float) -> None:
    if math.isclose(lqr_j, 0.0):
        reduction = math.nan
    else:
        reduction = 100.0 * (lqr_j - ppo_j) / lqr_j
    print(f"LQR: J = {lqr_j:.6f} A²s")
    print(f"LQR + Residual PPO: J = {ppo_j:.6f} A²s")
    print(f"Reduction = {reduction:.2f} %")


def main() -> None:
    args = parse_args()
    lqr_csv, ppo_csv = selected_csv_paths(args)
    lqr_run = load_run(lqr_csv, args.sample_time_s)
    ppo_run = load_run(ppo_csv, args.sample_time_s)
    intervals = disturbance_intervals(
        lqr_run,
        ppo_run,
        args.disturbance_start_s,
        args.disturbance_samples,
        args.sample_time_s,
    )

    plot_total_effort(args.output, lqr_run, ppo_run, intervals)
    print(f"Wrote {args.output}")

    decomposed_output = args.decomposed_output
    if args.save_decomposed and decomposed_output is None:
        decomposed_output = default_decomposed_output(args.output, args.disturbance_current_a)
    if decomposed_output is not None:
        plot_decomposed_effort(decomposed_output, lqr_run, ppo_run, intervals)
        print(f"Wrote {decomposed_output}")

    print_integrated_effort(
        float(lqr_run["integrated_effort"]),
        float(ppo_run["integrated_effort"]),
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
