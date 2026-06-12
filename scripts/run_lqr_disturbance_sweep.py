#!/usr/bin/env python3
"""Run an LQR actuator-disturbance sweep and summarize recovery metrics."""

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


DEFAULT_CURRENTS = [0.5, 1.0, 1.5, 2.0]
DISTURBANCE_TYPE = "physical-forward"
DISTURBANCE_START_S = 1.5
DISTURBANCE_SAMPLES = 10
TEST_MODE = "lqr-floor"
TASK = "Template-Twowheeledrobot-Standup-v0"
LAST_WINDOW_S = 1.0
SUMMARY_COLUMNS = [
    "disturbance_current_a",
    "disturbance_start_s",
    "disturbance_samples",
    "pre_rms_pitch_deg",
    "pre_peak_abs_pitch_deg",
    "post_rms_pitch_deg",
    "post_peak_abs_pitch_deg",
    "last_1s_rms_pitch_deg",
    "last_1s_peak_abs_pitch_deg",
    "last_1s_peak_to_peak_pitch_deg",
    "post_peak_to_peak_pitch_deg",
    "mean_pitch_last_1s_deg",
    "max_abs_pitch_deg_after_disturbance",
    "rms_pitch_deg_after_disturbance",
    "final_pitch_deg",
    "max_abs_current_command_a",
    "final_base_x_m",
    "final_base_y_m",
    "survival_success",
    "recovery_success",
    "csv_path",
    "run_success",
    "error_message",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--real-time", action="store_true", help="Pass --real-time to scripts/lqr_control.py.")
    parser.add_argument("--currents", nargs="+", type=float, default=DEFAULT_CURRENTS, help="Disturbance currents in A.")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/lqr_disturbance_sweep"))
    parser.add_argument("--enable-residual-rl", action="store_true", help="Evaluate LQR plus frozen residual PPO.")
    parser.add_argument("--residual-policy", type=Path, default=None, help="TorchScript residual policy.pt for evaluation.")
    parser.add_argument("--residual-action-limit", type=float, default=0.5, help="Residual current limit in A.")
    return parser.parse_args()


def current_label(current: float) -> str:
    return f"{current:.1f}".replace(".", "p")


def display_label(current: float) -> str:
    return f"{current:.1f} A"


def pitch_deg(df: pd.DataFrame) -> pd.Series:
    if "pitch_deg" in df:
        return df["pitch_deg"].astype(float)
    if "theta" in df:
        return np.degrees(df["theta"].astype(float))
    if "pitch_rad" in df:
        return np.degrees(df["pitch_rad"].astype(float))
    raise KeyError("CSV does not contain pitch_deg, theta, or pitch_rad")


def time_s(df: pd.DataFrame) -> pd.Series:
    if "time" in df:
        return df["time"].astype(float)
    if "time_s" in df:
        return df["time_s"].astype(float)
    if "t" in df:
        return df["t"].astype(float)
    raise KeyError("CSV does not contain time, time_s, or t")


def command_columns(df: pd.DataFrame) -> tuple[str, str]:
    candidates = [
        ("u_left_cmd_a", "u_right_cmd_a"),
        ("left_i_des_a", "right_i_des_a"),
        ("i_des_left", "i_des_right"),
        ("left_i_cmd_a", "right_i_cmd_a"),
        ("i_cmd_left", "i_cmd_right"),
        ("left_current_a", "right_current_a"),
    ]
    for left, right in candidates:
        if left in df and right in df:
            return left, right
    raise KeyError("CSV does not contain recognizable left/right current command columns")


def final_value(df: pd.DataFrame, column: str) -> float:
    if column not in df:
        return math.nan
    return float(df[column].iloc[-1])


def rms(values: pd.Series) -> float:
    if values.empty:
        return math.nan
    return float(np.sqrt(np.mean(np.square(values))))


def peak_abs(values: pd.Series) -> float:
    if values.empty:
        return math.nan
    return float(np.max(np.abs(values)))


def peak_to_peak(values: pd.Series) -> float:
    if values.empty:
        return math.nan
    return float(np.max(values) - np.min(values))


def mean(values: pd.Series) -> float:
    if values.empty:
        return math.nan
    return float(np.mean(values))


def fmt(value: float) -> str:
    if math.isnan(value):
        return "nan"
    return f"{value:.9f}"


def compute_metrics(csv_path: Path, current: float) -> tuple[dict[str, str], pd.DataFrame]:
    df = pd.read_csv(csv_path)
    if df.empty:
        raise ValueError(f"Empty CSV: {csv_path}")

    times = time_s(df)
    pitch = pitch_deg(df)
    pre = times < DISTURBANCE_START_S
    post = times >= DISTURBANCE_START_S
    if not bool(post.any()):
        raise ValueError(f"CSV has no samples at or after {DISTURBANCE_START_S:.3f} s: {csv_path}")

    final_time_s = float(times.iloc[-1])
    last_window_start_s = final_time_s - LAST_WINDOW_S
    last_window = times >= last_window_start_s
    pitch_pre = pitch[pre]
    pitch_post = pitch[post]
    pitch_last = pitch[last_window]

    left_cmd, right_cmd = command_columns(df)
    command_after = df.loc[post, [left_cmd, right_cmd]].astype(float)
    final_pitch = float(pitch.iloc[-1])
    max_abs_command = float(np.max(np.abs(command_after.to_numpy())))

    pre_rms = rms(pitch_pre)
    pre_peak = peak_abs(pitch_pre)
    post_rms = rms(pitch_post)
    post_peak = peak_abs(pitch_post)
    last_rms = rms(pitch_last)
    last_peak = peak_abs(pitch_last)
    last_ptp = peak_to_peak(pitch_last)
    post_ptp = peak_to_peak(pitch_post)
    last_mean = mean(pitch_last)

    survival_success = post_peak < 10.0
    recovery_success = survival_success and last_peak < 7.0 and last_rms < 4.0

    row = {
        "disturbance_current_a": f"{current:.9g}",
        "disturbance_start_s": f"{DISTURBANCE_START_S:.9g}",
        "disturbance_samples": str(DISTURBANCE_SAMPLES),
        "pre_rms_pitch_deg": fmt(pre_rms),
        "pre_peak_abs_pitch_deg": fmt(pre_peak),
        "post_rms_pitch_deg": fmt(post_rms),
        "post_peak_abs_pitch_deg": fmt(post_peak),
        "last_1s_rms_pitch_deg": fmt(last_rms),
        "last_1s_peak_abs_pitch_deg": fmt(last_peak),
        "last_1s_peak_to_peak_pitch_deg": fmt(last_ptp),
        "post_peak_to_peak_pitch_deg": fmt(post_ptp),
        "mean_pitch_last_1s_deg": fmt(last_mean),
        "max_abs_pitch_deg_after_disturbance": fmt(post_peak),
        "rms_pitch_deg_after_disturbance": fmt(post_rms),
        "final_pitch_deg": fmt(final_pitch),
        "max_abs_current_command_a": fmt(max_abs_command),
        "final_base_x_m": fmt(final_value(df, "base_position_x")),
        "final_base_y_m": fmt(final_value(df, "base_position_y")),
        "survival_success": "true" if survival_success else "false",
        "recovery_success": "true" if recovery_success else "false",
        "csv_path": str(csv_path),
        "run_success": "true",
        "error_message": "",
    }
    return row, df


def failure_row(current: float, csv_path: Path, error: str) -> dict[str, str]:
    row = {column: "" for column in SUMMARY_COLUMNS}
    row.update({
        "disturbance_current_a": f"{current:.9g}",
        "disturbance_start_s": f"{DISTURBANCE_START_S:.9g}",
        "disturbance_samples": str(DISTURBANCE_SAMPLES),
        "survival_success": "false",
        "recovery_success": "false",
        "csv_path": str(csv_path),
        "run_success": "false",
        "error_message": error.replace("\n", " ")[:1000],
    })
    return row


def run_simulation(
    repo_root: Path,
    current: float,
    log_path: Path,
    real_time: bool,
    enable_residual_rl: bool,
    residual_policy: Path | None,
    residual_action_limit: float,
) -> subprocess.CompletedProcess[str]:
    cmd = [
        sys.executable,
        "scripts/lqr_control.py",
        "--task", TASK,
        "--test-mode", TEST_MODE,
        "--actuator-disturbance", DISTURBANCE_TYPE,
        "--actuator-disturbance-start-s", str(DISTURBANCE_START_S),
        "--actuator-disturbance-samples", str(DISTURBANCE_SAMPLES),
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
    print("Running:", " ".join(cmd), flush=True)
    return subprocess.run(cmd, cwd=repo_root, text=True, capture_output=True, check=False)


def write_summary(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=SUMMARY_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def plot_pitch_overlay(path: Path, runs: list[tuple[float, pd.DataFrame]]) -> None:
    fig, axis = plt.subplots(figsize=(11, 6))
    for current, df in runs:
        axis.plot(time_s(df), pitch_deg(df), label=display_label(current), linewidth=1.4)
    axis.axvline(DISTURBANCE_START_S, color="black", linestyle="--", linewidth=1.0, label="disturbance start")
    if runs:
        max_time = max(float(time_s(df).iloc[-1]) for _, df in runs)
        axis.axvspan(max_time - LAST_WINDOW_S, max_time, color="tab:gray", alpha=0.12, label="last 1 s window")
    axis.set_title("LQR actuator disturbance sweep: pitch response")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Pitch [deg]")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_command_overlay(path: Path, runs: list[tuple[float, pd.DataFrame]]) -> None:
    fig, axis = plt.subplots(figsize=(11, 6))
    for current, df in runs:
        left_cmd, _ = command_columns(df)
        axis.plot(time_s(df), df[left_cmd].astype(float), label=display_label(current), linewidth=1.4)
    axis.axvline(DISTURBANCE_START_S, color="black", linestyle="--", linewidth=1.0, label="disturbance start")
    axis.set_title("LQR actuator disturbance sweep: left command response")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Current command [A]")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_pitch_rms_bar(path: Path, rows: list[dict[str, str]]) -> None:
    successful_rows = [row for row in rows if row.get("run_success") == "true"]
    labels = [display_label(float(row["disturbance_current_a"])) for row in successful_rows]
    pre_rms = [float(row["pre_rms_pitch_deg"]) for row in successful_rows]
    post_rms = [float(row["post_rms_pitch_deg"]) for row in successful_rows]
    last_rms = [float(row["last_1s_rms_pitch_deg"]) for row in successful_rows]

    fig, axis = plt.subplots(figsize=(11, 6))
    x = np.arange(len(labels))
    width = 0.25
    axis.bar(x - width, pre_rms, width, label="pre RMS")
    axis.bar(x, post_rms, width, label="post RMS")
    axis.bar(x + width, last_rms, width, label="last 1 s RMS")
    axis.set_title("LQR actuator disturbance sweep: pitch RMS")
    axis.set_xlabel("Disturbance current")
    axis.set_ylabel("Pitch RMS [deg]")
    axis.set_xticks(x, labels)
    axis.grid(True, axis="y", alpha=0.3)
    axis.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def print_summary(rows: list[dict[str, str]]) -> None:
    df = pd.DataFrame(rows, columns=SUMMARY_COLUMNS)
    display_columns = [
        "disturbance_current_a",
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
        "error_message",
    ]
    print("\nSummary:")
    print(df[display_columns].to_string(index=False))


def main() -> None:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    output_dir = args.output_dir if args.output_dir.is_absolute() else repo_root / args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, str]] = []
    successful_runs: list[tuple[float, pd.DataFrame]] = []
    for current in args.currents:
        log_path = output_dir / f"lqr_disturbance_{current_label(current)}A.csv"
        result = run_simulation(
            repo_root,
            current,
            log_path,
            args.real_time,
            args.enable_residual_rl,
            args.residual_policy,
            args.residual_action_limit,
        )
        if result.returncode != 0:
            message = result.stderr.strip() or result.stdout.strip() or f"subprocess exited with {result.returncode}"
            print(f"[WARN] Disturbance {current:.3g} A failed: {message}", flush=True)
            rows.append(failure_row(current, log_path, message))
            continue
        try:
            row, df = compute_metrics(log_path, current)
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] Disturbance {current:.3g} A produced unusable output: {exc}", flush=True)
            rows.append(failure_row(current, log_path, str(exc)))
            continue
        rows.append(row)
        successful_runs.append((current, df))

    summary_path = output_dir / "summary.csv"
    write_summary(summary_path, rows)
    pitch_plot_path = output_dir / "pitch_overlay.png"
    command_plot_path = output_dir / "command_overlay.png"
    pitch_rms_plot_path = output_dir / "pitch_rms_bar.png"
    plot_pitch_overlay(pitch_plot_path, successful_runs)
    plot_command_overlay(command_plot_path, successful_runs)
    plot_pitch_rms_bar(pitch_rms_plot_path, rows)

    print_summary(rows)
    print(f"Wrote {summary_path}")
    print(f"Wrote {pitch_plot_path}")
    print(f"Wrote {command_plot_path}")
    print(f"Wrote {pitch_rms_plot_path}")


if __name__ == "__main__":
    main()
