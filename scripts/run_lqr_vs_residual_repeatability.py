#!/usr/bin/env python3
"""Repeat LQR and residual-PPO disturbance tests and aggregate pitch metrics."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_lqr_disturbance_sweep as sweep


DEFAULT_REPEATS = 10
DEFAULT_OUTPUT_DIR = Path("outputs/lqr_vs_residual_repeatability")
CONTROLLERS = ("lqr", "residual_ppo")
CONTROLLER_LABELS = {
    "lqr": "LQR",
    "residual_ppo": "LQR + Residual PPO",
}
PER_RUN_COLUMNS = [
    "controller",
    "disturbance_current_a",
    "repeat_index",
    "run_success",
    "survival_success",
    "recovery_success",
    "post_peak_abs_pitch_deg",
    "post_rms_pitch_deg",
    "last_1s_peak_abs_pitch_deg",
    "last_1s_rms_pitch_deg",
    "last_1s_peak_to_peak_pitch_deg",
    "mean_pitch_last_1s_deg",
    "final_pitch_deg",
    "max_abs_current_command_a",
    "csv_path",
    "error_message",
]
AGGREGATE_METRICS = [
    "post_peak_abs_pitch_deg",
    "post_rms_pitch_deg",
    "last_1s_peak_abs_pitch_deg",
    "last_1s_rms_pitch_deg",
    "last_1s_peak_to_peak_pitch_deg",
    "max_abs_current_command_a",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=DEFAULT_REPEATS, help="Number of repeats per controller/current.")
    parser.add_argument(
        "--currents",
        nargs="+",
        type=float,
        default=sweep.DEFAULT_CURRENTS,
        help="Disturbance currents in A.",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--real-time", action="store_true", help="Pass --real-time to scripts/lqr_control.py.")
    parser.add_argument("--no-plot", action="store_true", help="Skip PNG plot generation.")
    parser.add_argument(
        "--residual-policy",
        "--ppo-checkpoint",
        dest="residual_policy",
        type=Path,
        default=None,
        help="Frozen TorchScript residual policy.pt for residual-PPO evaluation.",
    )
    parser.add_argument("--residual-action-limit", type=float, default=0.5, help="Residual current limit in A.")
    return parser.parse_args()


def bool_text(value: str | bool) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return "true" if str(value).lower() == "true" else "false"


def per_run_row(controller: str, repeat_index: int, metrics: dict[str, str]) -> dict[str, str]:
    row = {column: "" for column in PER_RUN_COLUMNS}
    row.update(
        {
            "controller": controller,
            "disturbance_current_a": metrics.get("disturbance_current_a", ""),
            "repeat_index": str(repeat_index),
            "run_success": bool_text(metrics.get("run_success", "false")),
            "survival_success": bool_text(metrics.get("survival_success", "false")),
            "recovery_success": bool_text(metrics.get("recovery_success", "false")),
            "post_peak_abs_pitch_deg": metrics.get("post_peak_abs_pitch_deg", ""),
            "post_rms_pitch_deg": metrics.get("post_rms_pitch_deg", ""),
            "last_1s_peak_abs_pitch_deg": metrics.get("last_1s_peak_abs_pitch_deg", ""),
            "last_1s_rms_pitch_deg": metrics.get("last_1s_rms_pitch_deg", ""),
            "last_1s_peak_to_peak_pitch_deg": metrics.get("last_1s_peak_to_peak_pitch_deg", ""),
            "mean_pitch_last_1s_deg": metrics.get("mean_pitch_last_1s_deg", ""),
            "final_pitch_deg": metrics.get("final_pitch_deg", ""),
            "max_abs_current_command_a": metrics.get("max_abs_current_command_a", ""),
            "csv_path": metrics.get("csv_path", ""),
            "error_message": metrics.get("error_message", ""),
        }
    )
    return row


def failure_row(controller: str, current: float, repeat_index: int, csv_path: Path, error: str) -> dict[str, str]:
    row = {column: "" for column in PER_RUN_COLUMNS}
    row.update(
        {
            "controller": controller,
            "disturbance_current_a": f"{current:.9g}",
            "repeat_index": str(repeat_index),
            "run_success": "false",
            "survival_success": "false",
            "recovery_success": "false",
            "csv_path": str(csv_path),
            "error_message": error.replace("\n", " ")[:1000],
        }
    )
    return row


def write_csv(path: Path, rows: list[dict[str, str]], columns: list[str]) -> None:
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def successful_frame(rows: list[dict[str, str]]) -> pd.DataFrame:
    df = pd.DataFrame(rows, columns=PER_RUN_COLUMNS)
    if df.empty:
        return df
    df = df[df["run_success"] == "true"].copy()
    for column in ["disturbance_current_a", *AGGREGATE_METRICS]:
        df[column] = pd.to_numeric(df[column], errors="coerce")
    return df


def aggregate_rows(rows: list[dict[str, str]], currents: list[float]) -> list[dict[str, str]]:
    df = successful_frame(rows)
    if df.empty:
        return []

    grouped = df.groupby(["controller", "disturbance_current_a"], sort=False)
    aggregate: list[dict[str, str]] = []
    for (controller, current), group in grouped:
        row: dict[str, str] = {
            "controller": str(controller),
            "disturbance_current_a": f"{float(current):.9g}",
            "successful_runs": str(len(group)),
        }
        for metric in AGGREGATE_METRICS:
            values = group[metric].dropna()
            row[f"{metric}_mean"] = format_float(float(values.mean())) if not values.empty else "nan"
            row[f"{metric}_std"] = format_float(float(values.std(ddof=1))) if len(values) > 1 else "0.000000000"
            row[f"{metric}_min"] = format_float(float(values.min())) if not values.empty else "nan"
            row[f"{metric}_max"] = format_float(float(values.max())) if not values.empty else "nan"
        row["rms_improvement_percent"] = "nan"
        row["peak_improvement_percent"] = "nan"
        aggregate.append(row)

    by_key = {(row["controller"], float(row["disturbance_current_a"])): row for row in aggregate}
    for current in currents:
        lqr = by_key.get(("lqr", float(current)))
        ppo = by_key.get(("residual_ppo", float(current)))
        if lqr is None or ppo is None:
            continue
        lqr_rms = parse_float(lqr["last_1s_rms_pitch_deg_mean"])
        ppo_rms = parse_float(ppo["last_1s_rms_pitch_deg_mean"])
        lqr_peak = parse_float(lqr["post_peak_abs_pitch_deg_mean"])
        ppo_peak = parse_float(ppo["post_peak_abs_pitch_deg_mean"])
        rms_improvement = percent_improvement(lqr_rms, ppo_rms)
        peak_improvement = percent_improvement(lqr_peak, ppo_peak)
        for row in (lqr, ppo):
            row["rms_improvement_percent"] = format_float(rms_improvement)
            row["peak_improvement_percent"] = format_float(peak_improvement)

    order = {controller: index for index, controller in enumerate(CONTROLLERS)}
    aggregate.sort(key=lambda row: (float(row["disturbance_current_a"]), order.get(row["controller"], 99)))
    return aggregate


def aggregate_columns() -> list[str]:
    columns = ["controller", "disturbance_current_a", "successful_runs"]
    for metric in AGGREGATE_METRICS:
        columns.extend([f"{metric}_mean", f"{metric}_std", f"{metric}_min", f"{metric}_max"])
    columns.extend(["rms_improvement_percent", "peak_improvement_percent"])
    return columns


def parse_float(value: str) -> float:
    try:
        return float(value)
    except ValueError:
        return math.nan


def format_float(value: float) -> str:
    if math.isnan(value):
        return "nan"
    return f"{value:.9f}"


def percent_improvement(baseline: float, candidate: float) -> float:
    if math.isnan(baseline) or math.isnan(candidate) or abs(baseline) < 1.0e-12:
        return math.nan
    return 100.0 * (baseline - candidate) / baseline


def metric_table(aggregate: list[dict[str, str]], metric: str, currents: list[float]) -> tuple[np.ndarray, np.ndarray]:
    means = np.full((len(CONTROLLERS), len(currents)), np.nan)
    stds = np.full((len(CONTROLLERS), len(currents)), np.nan)
    index = {
        (row["controller"], float(row["disturbance_current_a"])): row
        for row in aggregate
    }
    for controller_index, controller in enumerate(CONTROLLERS):
        for current_index, current in enumerate(currents):
            row = index.get((controller, float(current)))
            if row is None:
                continue
            means[controller_index, current_index] = parse_float(row[f"{metric}_mean"])
            stds[controller_index, current_index] = parse_float(row[f"{metric}_std"])
    return means, np.nan_to_num(stds, nan=0.0)


def plot_grouped_bars(
    path: Path,
    aggregate: list[dict[str, str]],
    currents: list[float],
    metric: str,
    title: str,
    ylabel: str,
    axis: plt.Axes | None = None,
) -> None:
    owns_figure = axis is None
    if axis is None:
        fig, axis = plt.subplots(figsize=(9, 5.4))
    else:
        fig = axis.figure

    means, stds = metric_table(aggregate, metric, currents)
    x = np.arange(len(currents))
    width = 0.36
    offsets = (-width / 2.0, width / 2.0)
    colors = ("#4c78a8", "#f58518")
    for idx, controller in enumerate(CONTROLLERS):
        axis.bar(
            x + offsets[idx],
            means[idx],
            width,
            yerr=stds[idx],
            capsize=4,
            label=CONTROLLER_LABELS[controller],
            color=colors[idx],
            edgecolor="black",
            linewidth=0.6,
        )

    axis.set_title(title)
    axis.set_xlabel("Disturbance current [A]")
    axis.set_ylabel(ylabel)
    axis.set_xticks(x, [f"{current:.1f} A" for current in currents])
    axis.grid(True, axis="y", alpha=0.3)
    axis.legend(loc="best")
    if owns_figure:
        fig.tight_layout()
        fig.savefig(path, dpi=180)
        plt.close(fig)


def plot_comparisons(output_dir: Path, aggregate: list[dict[str, str]], currents: list[float]) -> None:
    plot_grouped_bars(
        output_dir / "rms_pitch_comparison.png",
        aggregate,
        currents,
        "last_1s_rms_pitch_deg",
        "Last 1 s RMS Pitch Angle",
        "Last 1 s RMS pitch angle [deg]",
    )
    plot_grouped_bars(
        output_dir / "peak_pitch_comparison.png",
        aggregate,
        currents,
        "post_peak_abs_pitch_deg",
        "Peak Pitch Angle After Disturbance",
        "Peak absolute pitch angle after disturbance [deg]",
    )

    fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.2), sharex=True)
    plot_grouped_bars(
        output_dir / "rms_pitch_comparison.png",
        aggregate,
        currents,
        "last_1s_rms_pitch_deg",
        "Last 1 s RMS Pitch Angle",
        "Last 1 s RMS pitch angle [deg]",
        axis=axes[0],
    )
    plot_grouped_bars(
        output_dir / "peak_pitch_comparison.png",
        aggregate,
        currents,
        "post_peak_abs_pitch_deg",
        "Peak Pitch Angle After Disturbance",
        "Peak absolute pitch angle after disturbance [deg]",
        axis=axes[1],
    )
    fig.tight_layout()
    fig.savefig(output_dir / "rms_and_peak_side_by_side.png", dpi=180)
    plt.close(fig)


def print_warnings(rows: list[dict[str, str]], expected_repeats: int, currents: list[float]) -> None:
    df = pd.DataFrame(rows, columns=PER_RUN_COLUMNS)
    for controller in CONTROLLERS:
        for current in currents:
            mask = (
                (df["controller"] == controller)
                & (pd.to_numeric(df["disturbance_current_a"], errors="coerce") == float(current))
                & (df["run_success"] == "true")
            )
            successes = int(mask.sum())
            if successes < expected_repeats:
                print(
                    f"[WARN] {controller} {current:.3g} A has {successes}/{expected_repeats} successful runs.",
                    flush=True,
                )


def main() -> None:
    args = parse_args()
    if args.repeats < 1:
        raise ValueError("--repeats must be at least 1.")

    repo_root = Path(__file__).resolve().parents[1]
    output_dir = args.output_dir if args.output_dir.is_absolute() else repo_root / args.output_dir
    raw_dir = output_dir / "raw"
    for controller in CONTROLLERS:
        (raw_dir / controller).mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, str]] = []
    for controller in CONTROLLERS:
        enable_residual = controller == "residual_ppo"
        for current in args.currents:
            for repeat_index in range(args.repeats):
                csv_path = raw_dir / controller / f"{sweep.current_label(current)}A_run_{repeat_index:02d}.csv"
                result = sweep.run_simulation(
                    repo_root,
                    current,
                    csv_path,
                    args.real_time,
                    enable_residual,
                    args.residual_policy,
                    args.residual_action_limit,
                )
                if result.returncode != 0:
                    message = result.stderr.strip() or result.stdout.strip() or f"subprocess exited with {result.returncode}"
                    print(
                        f"[WARN] {controller} {current:.3g} A repeat {repeat_index} failed: {message}",
                        flush=True,
                    )
                    rows.append(failure_row(controller, current, repeat_index, csv_path, message))
                    continue

                try:
                    metrics, _ = sweep.compute_metrics(csv_path, current)
                except Exception as exc:  # noqa: BLE001
                    print(
                        f"[WARN] {controller} {current:.3g} A repeat {repeat_index} produced unusable output: {exc}",
                        flush=True,
                    )
                    rows.append(failure_row(controller, current, repeat_index, csv_path, str(exc)))
                    continue
                rows.append(per_run_row(controller, repeat_index, metrics))

    per_run_path = output_dir / "per_run_summary.csv"
    write_csv(per_run_path, rows, PER_RUN_COLUMNS)

    aggregate = aggregate_rows(rows, list(args.currents))
    aggregate_path = output_dir / "aggregate_summary.csv"
    write_csv(aggregate_path, aggregate, aggregate_columns())

    if not args.no_plot:
        plot_comparisons(output_dir, aggregate, list(args.currents))

    print_warnings(rows, args.repeats, list(args.currents))
    print(f"Wrote {per_run_path}")
    print(f"Wrote {aggregate_path}")
    if not args.no_plot:
        print(f"Wrote {output_dir / 'rms_pitch_comparison.png'}")
        print(f"Wrote {output_dir / 'peak_pitch_comparison.png'}")
        print(f"Wrote {output_dir / 'rms_and_peak_side_by_side.png'}")


if __name__ == "__main__":
    main()
