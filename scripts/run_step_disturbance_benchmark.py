#!/usr/bin/env python3
"""Run LQR vs residual-PPO step actuator-disturbance benchmark."""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_lqr_disturbance_sweep as sweep


DEFAULT_CURRENTS = [0.1, 0.2, 0.3, 0.5]
DEFAULT_REPEATS = 3
DEFAULT_OUTPUT_DIR = Path("outputs/step_disturbance_benchmark")
DEFAULT_START_S = 1.5
DEFAULT_DURATION_S = 2.0
DEFAULT_SELECTED_CURRENT_A = 0.3
CONTROLLERS = ("lqr", "residual_ppo")
CONTROLLER_LABELS = {
    "lqr": "LQR",
    "residual_ppo": "LQR + Residual PPO",
}
PER_RUN_COLUMNS = [
    "controller",
    "disturbance_current_a",
    "disturbance_start_s",
    "disturbance_duration_s",
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
    "rms_current_command_a",
    "mean_abs_current_command_a",
    "integrated_control_effort_a2s",
    "during_step_rms_pitch_deg",
    "during_step_mean_pitch_deg",
    "during_step_peak_abs_pitch_deg",
    "after_step_rms_pitch_deg",
    "after_step_peak_abs_pitch_deg",
    "csv_path",
    "error_message",
]
AGGREGATE_METRICS = [
    "post_peak_abs_pitch_deg",
    "post_rms_pitch_deg",
    "last_1s_peak_abs_pitch_deg",
    "last_1s_rms_pitch_deg",
    "last_1s_peak_to_peak_pitch_deg",
    "mean_pitch_last_1s_deg",
    "rms_current_command_a",
    "mean_abs_current_command_a",
    "integrated_control_effort_a2s",
    "during_step_rms_pitch_deg",
    "during_step_mean_pitch_deg",
    "during_step_peak_abs_pitch_deg",
    "after_step_rms_pitch_deg",
    "after_step_peak_abs_pitch_deg",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=DEFAULT_REPEATS, help="Number of repeats per controller/current.")
    parser.add_argument("--currents", nargs="+", type=float, default=DEFAULT_CURRENTS, help="Step disturbance currents in A.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--actuator-disturbance-start-s", type=float, default=DEFAULT_START_S)
    parser.add_argument("--actuator-disturbance-duration-s", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument("--selected-current-a", type=float, default=DEFAULT_SELECTED_CURRENT_A)
    parser.add_argument("--real-time", action="store_true", help="Pass --real-time to scripts/lqr_control.py.")
    parser.add_argument("--headless", action="store_true", help="Pass --headless to scripts/lqr_control.py.")
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


def current_label(current: float) -> str:
    return f"{current:.1f}".replace(".", "p")


def fmt(value: float) -> str:
    if math.isnan(value):
        return "nan"
    return f"{value:.9f}"


def parse_float(value: str) -> float:
    try:
        return float(value)
    except ValueError:
        return math.nan


def rms(values: pd.Series | np.ndarray) -> float:
    if len(values) == 0:
        return math.nan
    return float(np.sqrt(np.mean(np.square(values))))


def peak_abs(values: pd.Series | np.ndarray) -> float:
    if len(values) == 0:
        return math.nan
    return float(np.max(np.abs(values)))


def peak_to_peak(values: pd.Series | np.ndarray) -> float:
    if len(values) == 0:
        return math.nan
    return float(np.max(values) - np.min(values))


def mean(values: pd.Series | np.ndarray) -> float:
    if len(values) == 0:
        return math.nan
    return float(np.mean(values))


def mean_sample_time_s(times: pd.Series) -> float:
    diffs = np.diff(times.astype(float).to_numpy())
    diffs = diffs[diffs > 0.0]
    if diffs.size == 0:
        raise ValueError("Cannot compute sample time from CSV time column.")
    return float(np.mean(diffs))


def final_command_columns(df: pd.DataFrame) -> tuple[str, str]:
    candidates = [
        ("u_left_cmd_a", "u_right_cmd_a"),
        ("u_left_final_a", "u_right_final_a"),
        ("u_left_final", "u_right_final"),
        ("left_i_cmd_a", "right_i_cmd_a"),
        ("i_cmd_left", "i_cmd_right"),
        ("left_current_a", "right_current_a"),
    ]
    for left, right in candidates:
        if left in df and right in df:
            return left, right
    raise KeyError("CSV does not contain recognizable left/right final current command columns")


def instantaneous_control_effort(df: pd.DataFrame) -> np.ndarray:
    left_col, right_col = final_command_columns(df)
    u_left = df[left_col].astype(float).to_numpy()
    u_right = df[right_col].astype(float).to_numpy()
    return np.square(u_left) + np.square(u_right)


def compute_metrics(csv_path: Path, current: float, start_s: float, duration_s: float) -> tuple[dict[str, str], pd.DataFrame]:
    df = pd.read_csv(csv_path)
    if df.empty:
        raise ValueError(f"Empty CSV: {csv_path}")

    times = sweep.time_s(df)
    pitch = sweep.pitch_deg(df)
    end_s = start_s + duration_s
    post = times >= start_s
    during_step = (times >= start_s) & (times <= end_s)
    after_step = times > end_s
    if not bool(post.any()):
        raise ValueError(f"CSV has no samples at or after {start_s:.3f} s: {csv_path}")
    if not bool(during_step.any()):
        raise ValueError(f"CSV has no samples during step interval [{start_s:.3f}, {end_s:.3f}] s: {csv_path}")

    final_time_s = float(times.iloc[-1])
    last_window = times >= final_time_s - sweep.LAST_WINDOW_S
    pitch_post = pitch[post]
    pitch_last = pitch[last_window]
    pitch_during = pitch[during_step]
    pitch_after = pitch[after_step]

    effort = instantaneous_control_effort(df)
    dt = mean_sample_time_s(times)
    left_col, right_col = final_command_columns(df)
    u_left = df[left_col].astype(float).to_numpy()
    u_right = df[right_col].astype(float).to_numpy()

    post_peak = peak_abs(pitch_post)
    last_peak = peak_abs(pitch_last)
    last_rms = rms(pitch_last)
    survival_success = post_peak < 10.0
    recovery_success = survival_success and last_peak < 7.0 and last_rms < 4.0

    row = {
        "controller": "",
        "disturbance_current_a": f"{current:.9g}",
        "disturbance_start_s": f"{start_s:.9g}",
        "disturbance_duration_s": f"{duration_s:.9g}",
        "repeat_index": "",
        "run_success": "true",
        "survival_success": "true" if survival_success else "false",
        "recovery_success": "true" if recovery_success else "false",
        "post_peak_abs_pitch_deg": fmt(post_peak),
        "post_rms_pitch_deg": fmt(rms(pitch_post)),
        "last_1s_peak_abs_pitch_deg": fmt(last_peak),
        "last_1s_rms_pitch_deg": fmt(last_rms),
        "last_1s_peak_to_peak_pitch_deg": fmt(peak_to_peak(pitch_last)),
        "mean_pitch_last_1s_deg": fmt(mean(pitch_last)),
        "rms_current_command_a": fmt(float(np.sqrt(np.mean(effort)))),
        "mean_abs_current_command_a": fmt(float(np.mean(np.abs(u_left) + np.abs(u_right)))),
        "integrated_control_effort_a2s": fmt(float(np.sum(effort * dt))),
        "during_step_rms_pitch_deg": fmt(rms(pitch_during)),
        "during_step_mean_pitch_deg": fmt(mean(pitch_during)),
        "during_step_peak_abs_pitch_deg": fmt(peak_abs(pitch_during)),
        "after_step_rms_pitch_deg": fmt(rms(pitch_after)),
        "after_step_peak_abs_pitch_deg": fmt(peak_abs(pitch_after)),
        "csv_path": str(csv_path),
        "error_message": "",
    }
    return row, df


def run_simulation(
    repo_root: Path,
    current: float,
    log_path: Path,
    start_s: float,
    duration_s: float,
    real_time: bool,
    enable_residual_rl: bool,
    residual_policy: Path | None,
    residual_action_limit: float,
    headless: bool,
) -> subprocess.CompletedProcess[str]:
    cmd = [
        sys.executable,
        "scripts/lqr_control.py",
        "--task", sweep.TASK,
        "--test-mode", sweep.TEST_MODE,
        "--actuator-disturbance", "step-forward",
        "--actuator-disturbance-start-s", str(start_s),
        "--actuator-disturbance-duration-s", str(duration_s),
        "--actuator-disturbance-current-a", str(current),
        "--log-csv", str(log_path),
        "--no-plot",
    ]
    if enable_residual_rl:
        cmd.extend(["--enable-residual-rl", "--evaluation-mode", "--residual-action-limit", str(residual_action_limit)])
        if residual_policy is not None:
            cmd.extend(["--residual-policy", str(residual_policy)])
    if real_time:
        cmd.append("--real-time")
    if headless:
        cmd.append("--headless")
    print("Running:", " ".join(cmd), flush=True)
    return subprocess.run(cmd, cwd=repo_root, text=True, capture_output=True, check=False)


def failure_row(controller: str, current: float, repeat_index: int, csv_path: Path, start_s: float, duration_s: float, error: str) -> dict[str, str]:
    row = {column: "" for column in PER_RUN_COLUMNS}
    row.update(
        {
            "controller": controller,
            "disturbance_current_a": f"{current:.9g}",
            "disturbance_start_s": f"{start_s:.9g}",
            "disturbance_duration_s": f"{duration_s:.9g}",
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

    aggregate: list[dict[str, str]] = []
    for (controller, current), group in df.groupby(["controller", "disturbance_current_a"], sort=False):
        row: dict[str, str] = {
            "controller": str(controller),
            "disturbance_current_a": f"{float(current):.9g}",
            "successful_runs": str(len(group)),
        }
        for metric in AGGREGATE_METRICS:
            values = group[metric].dropna()
            row[f"{metric}_mean"] = fmt(float(values.mean())) if not values.empty else "nan"
            row[f"{metric}_std"] = fmt(float(values.std(ddof=1))) if len(values) > 1 else "0.000000000"
            row[f"{metric}_min"] = fmt(float(values.min())) if not values.empty else "nan"
            row[f"{metric}_max"] = fmt(float(values.max())) if not values.empty else "nan"
        aggregate.append(row)

    order = {controller: index for index, controller in enumerate(CONTROLLERS)}
    aggregate.sort(key=lambda row: (float(row["disturbance_current_a"]), order.get(row["controller"], 99)))
    return aggregate


def aggregate_columns() -> list[str]:
    columns = ["controller", "disturbance_current_a", "successful_runs"]
    for metric in AGGREGATE_METRICS:
        columns.extend([f"{metric}_mean", f"{metric}_std", f"{metric}_min", f"{metric}_max"])
    return columns


def metric_table(aggregate: list[dict[str, str]], metric: str, currents: list[float]) -> tuple[np.ndarray, np.ndarray]:
    means = np.full((len(CONTROLLERS), len(currents)), np.nan)
    stds = np.full((len(CONTROLLERS), len(currents)), np.nan)
    index = {(row["controller"], float(row["disturbance_current_a"])): row for row in aggregate}
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
) -> None:
    means, stds = metric_table(aggregate, metric, currents)
    fig, axis = plt.subplots(figsize=(9, 5.4))
    x = np.arange(len(currents))
    width = 0.36
    colors = ("#4c78a8", "#f58518")
    for idx, controller in enumerate(CONTROLLERS):
        axis.bar(
            x + (-width / 2.0 if idx == 0 else width / 2.0),
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
    axis.set_xlabel("Step disturbance current [A]")
    axis.set_ylabel(ylabel)
    axis.set_xticks(x, [f"{current:.1f}" for current in currents])
    axis.grid(True, axis="y", alpha=0.3)
    axis.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def selected_runs(rows: list[dict[str, str]], selected_current: float) -> dict[str, pd.DataFrame]:
    runs: dict[str, pd.DataFrame] = {}
    for controller in CONTROLLERS:
        matches = [
            row for row in rows
            if row.get("controller") == controller
            and row.get("run_success") == "true"
            and math.isclose(parse_float(row.get("disturbance_current_a", "nan")), selected_current, rel_tol=0.0, abs_tol=1.0e-9)
        ]
        if not matches:
            continue
        matches.sort(key=lambda row: int(row.get("repeat_index", "0") or 0))
        runs[controller] = pd.read_csv(matches[0]["csv_path"])
    return runs


def plot_time_response(path: Path, rows: list[dict[str, str]], selected_current: float, start_s: float, duration_s: float) -> None:
    runs = selected_runs(rows, selected_current)
    if not runs:
        print(f"[WARN] No successful runs available for {selected_current:.3g} A time-response plot.", flush=True)
        return

    fig, axes = plt.subplots(2, 1, figsize=(11, 7.5), sharex=True)
    end_s = start_s + duration_s
    for controller, df in runs.items():
        times = sweep.time_s(df)
        axes[0].plot(times, sweep.pitch_deg(df), label=CONTROLLER_LABELS[controller], linewidth=1.5)
        axes[1].plot(times, instantaneous_control_effort(df), label=CONTROLLER_LABELS[controller], linewidth=1.5)
    for axis in axes:
        axis.axvspan(start_s, end_s, color="tab:gray", alpha=0.18, label="step disturbance")
        axis.grid(True, alpha=0.3)
        handles, labels = axis.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        axis.legend(unique.values(), unique.keys(), loc="best")
    axes[0].set_title("Step Disturbance Response")
    axes[0].set_ylabel("Pitch angle [deg]")
    axes[1].set_ylabel("Instantaneous control effort [A^2]")
    axes[1].set_xlabel("Time [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_comparisons(output_dir: Path, aggregate: list[dict[str, str]], rows: list[dict[str, str]], currents: list[float], selected_current: float, start_s: float, duration_s: float) -> None:
    plot_grouped_bars(
        output_dir / "step_pitch_rms_comparison.png",
        aggregate,
        currents,
        "last_1s_rms_pitch_deg",
        "Last 1 s RMS Pitch Angle",
        "Last 1 s RMS pitch angle [deg]",
    )
    plot_grouped_bars(
        output_dir / "step_peak_pitch_comparison.png",
        aggregate,
        currents,
        "post_peak_abs_pitch_deg",
        "Peak Pitch Angle",
        "Peak absolute pitch angle [deg]",
    )
    plot_grouped_bars(
        output_dir / "step_control_effort_comparison.png",
        aggregate,
        currents,
        "integrated_control_effort_a2s",
        "Integrated Control Effort",
        "Integrated control effort [A^2s]",
    )
    plot_time_response(
        output_dir / "step_single_run_time_response.png",
        rows,
        selected_current,
        start_s,
        duration_s,
    )


def main() -> None:
    args = parse_args()
    if args.repeats < 1:
        raise ValueError("--repeats must be at least 1.")
    if args.actuator_disturbance_start_s < 0.0:
        raise ValueError("--actuator-disturbance-start-s must be non-negative.")
    if args.actuator_disturbance_duration_s <= 0.0:
        raise ValueError("--actuator-disturbance-duration-s must be positive.")

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
                csv_path = raw_dir / controller / f"step_{current_label(current)}A_run_{repeat_index:02d}.csv"
                result = run_simulation(
                    repo_root,
                    current,
                    csv_path,
                    args.actuator_disturbance_start_s,
                    args.actuator_disturbance_duration_s,
                    args.real_time,
                    enable_residual,
                    args.residual_policy,
                    args.residual_action_limit,
                    args.headless,
                )
                if result.returncode != 0:
                    message = result.stderr.strip() or result.stdout.strip() or f"subprocess exited with {result.returncode}"
                    print(f"[WARN] {controller} {current:.3g} A repeat {repeat_index} failed: {message}", flush=True)
                    rows.append(
                        failure_row(
                            controller,
                            current,
                            repeat_index,
                            csv_path,
                            args.actuator_disturbance_start_s,
                            args.actuator_disturbance_duration_s,
                            message,
                        )
                    )
                    continue
                try:
                    row, _df = compute_metrics(
                        csv_path,
                        current,
                        args.actuator_disturbance_start_s,
                        args.actuator_disturbance_duration_s,
                    )
                except Exception as exc:  # noqa: BLE001
                    print(f"[WARN] {controller} {current:.3g} A repeat {repeat_index} produced unusable output: {exc}", flush=True)
                    rows.append(
                        failure_row(
                            controller,
                            current,
                            repeat_index,
                            csv_path,
                            args.actuator_disturbance_start_s,
                            args.actuator_disturbance_duration_s,
                            str(exc),
                        )
                    )
                    continue
                row["controller"] = controller
                row["repeat_index"] = str(repeat_index)
                rows.append(row)

    per_run_path = output_dir / "per_run_summary.csv"
    write_csv(per_run_path, rows, PER_RUN_COLUMNS)
    aggregate = aggregate_rows(rows, list(args.currents))
    aggregate_path = output_dir / "aggregate_summary.csv"
    write_csv(aggregate_path, aggregate, aggregate_columns())

    if not args.no_plot:
        plot_comparisons(
            output_dir,
            aggregate,
            rows,
            list(args.currents),
            args.selected_current_a,
            args.actuator_disturbance_start_s,
            args.actuator_disturbance_duration_s,
        )

    print(f"Wrote {per_run_path}")
    print(f"Wrote {aggregate_path}")
    if not args.no_plot:
        print(f"Wrote {output_dir / 'step_pitch_rms_comparison.png'}")
        print(f"Wrote {output_dir / 'step_peak_pitch_comparison.png'}")
        print(f"Wrote {output_dir / 'step_control_effort_comparison.png'}")
        print(f"Wrote {output_dir / 'step_single_run_time_response.png'}")


if __name__ == "__main__":
    main()
