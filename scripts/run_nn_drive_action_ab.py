#!/usr/bin/env python3
"""Run a matched stage-1 A/B of policy-controlled legs versus fixed legs.

Both arms train from scratch with the same seed, observations, rewards,
commands, randomization, terrain and PPO hyperparameters. The only intentional
difference is the action interface:

* four_leg: 4 CyberGear targets + 2 wheel currents
* fixed_stance: fixed zero-radian CyberGear targets + 2 wheel currents

The final checkpoint from each arm is evaluated on the same deterministic
stage-1 scenarios. This is intentionally shorter than another full curriculum.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from nn_drive_benchmark_contract import checkpoint_score

VARIANTS = {
    "four_leg": "Template-Twowheeledrobot-NNDrive-v0",
    "fixed_stance": "Template-Twowheeledrobot-NNDriveFixedStance-v0",
}
SCENARIOS = ("station_keeping", "drive_forward_slow", "drive_backward_slow")
EXPERIMENT_NAME = "nn_drive_action_ab"


def _checkpoint_iteration(path: Path) -> int:
    match = re.fullmatch(r"model_(\d+)\.pt", path.name)
    if match is None:
        raise ValueError(f"Unexpected checkpoint name: {path.name}")
    return int(match.group(1))


def _latest_checkpoint(run_dir: Path) -> Path:
    checkpoints = list(run_dir.glob("model_*.pt"))
    if not checkpoints:
        raise RuntimeError(f"No checkpoints found in {run_dir}")
    return max(checkpoints, key=_checkpoint_iteration)


def _new_run(log_root: Path, previous_runs: set[Path], run_name: str) -> Path:
    current_runs = {path for path in log_root.iterdir() if path.is_dir()} if log_root.is_dir() else set()
    matches = sorted(path for path in current_runs - previous_runs if path.name.endswith(f"_{run_name}"))
    if len(matches) != 1:
        names = ", ".join(path.name for path in matches) or "none"
        raise RuntimeError(f"Expected one new run ending in _{run_name}; found: {names}")
    return matches[0]


def _mean(values: list[float]) -> float:
    return sum(values) / len(values)


def _summarize(records: list[dict]) -> dict:
    summary: dict[str, dict] = {}
    for variant in VARIANTS:
        variant_records = [record for record in records if record["variant"] == variant]
        scenario_summary = {}
        for scenario in SCENARIOS:
            metrics = [record["benchmark"]["scenarios"][scenario] for record in variant_records]
            scenario_summary[scenario] = {key: _mean([float(item[key]) for item in metrics]) for key in metrics[0]}
        scores = [checkpoint_score(record["benchmark"]["scenarios"]) for record in variant_records]
        summary[variant] = {
            "mean_checkpoint_score": _mean(scores),
            "scenarios": scenario_summary,
        }

    four = summary["four_leg"]["scenarios"]["station_keeping"]
    fixed = summary["fixed_stance"]["scenarios"]["station_keeping"]
    termination_improvement = four["termination_rate"] - fixed["termination_rate"]
    score_ratio = summary["fixed_stance"]["mean_checkpoint_score"] / max(
        summary["four_leg"]["mean_checkpoint_score"], 1.0e-9
    )
    if fixed["termination_rate"] <= 0.10 and four["termination_rate"] > 0.10:
        verdict = (
            "fixed stance clears the balance gate while four-leg control does not; "
            "the leg-action interface is implicated"
        )
    elif termination_improvement >= 0.15 and score_ratio <= 0.75:
        verdict = "fixed stance is materially better; the leg-action interface is the leading cause"
    elif four["termination_rate"] > 0.20 and fixed["termination_rate"] > 0.20 and abs(termination_improvement) < 0.10:
        verdict = "both variants regress similarly; investigate the shared balance-learning path"
    elif termination_improvement <= -0.15:
        verdict = "four-leg control is materially better; fixed stance removes balance authority the policy needs"
    else:
        verdict = "the result is inconclusive at this horizon; add seeds before changing the curriculum"
    summary["comparison"] = {
        "station_keeping_termination_improvement_fixed_minus_four_leg": termination_improvement,
        "fixed_to_four_leg_score_ratio": score_ratio,
        "verdict": verdict,
        "seed_count": len({record["seed"] for record in records}),
    }
    return summary


def _print_summary(summary: dict) -> None:
    print("\nA/B mean results")
    print("variant       score    station_term  station_pitch  station_roll  station_drift")
    for variant in VARIANTS:
        station = summary[variant]["scenarios"]["station_keeping"]
        print(
            f"{variant:13} {summary[variant]['mean_checkpoint_score']:7.3f} "
            f"{station['termination_rate']:13.3f} {station['rms_pitch_deg']:14.3f} "
            f"{station['rms_roll_deg']:13.3f} {station['world_drift_m']:14.3f}"
        )
    print(f"\nVerdict: {summary['comparison']['verdict']}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--num-envs", type=int, default=4096)
    parser.add_argument("--iterations", type=int, default=200)
    parser.add_argument("--seeds", type=int, nargs="+", default=[42])
    parser.add_argument("--benchmark-num-envs", type=int, default=64)
    parser.add_argument("--benchmark-num-steps", type=int, default=1000)
    parser.add_argument("--device", type=str, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--dry-run", action="store_true")
    # Applied to BOTH training and benchmarking. Any cfg change that alters the
    # task must reach both, or the arms are evaluated under a distribution they
    # were not trained on.
    parser.add_argument(
        "--env-override",
        nargs="*",
        default=[],
        metavar="KEY=VALUE",
        help="Extra hydra overrides, e.g. env.com_offset_y_range_m=[-0.008,0.008]",
    )
    args = parser.parse_args()
    for override in args.env_override:
        if "=" not in override:
            parser.error(f"--env-override entries must be KEY=VALUE, got: {override}")
    if args.iterations < 1:
        parser.error("--iterations must be at least 1")
    if args.num_envs < 1 or args.benchmark_num_envs < 1 or args.benchmark_num_steps < 1:
        parser.error("environment and benchmark sizes must be at least 1")

    repo = Path(__file__).resolve().parents[1]
    train_py = repo / "scripts" / "rsl_rl" / "train.py"
    benchmark_py = repo / "scripts" / "benchmark_nn_drive.py"
    log_root = repo / "logs" / "rsl_rl" / EXPERIMENT_NAME
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_dir = args.output_dir or repo / "outputs" / "nn_drive_action_ab" / stamp
    records: list[dict] = []

    for seed in args.seeds:
        for variant, task in VARIANTS.items():
            run_name = f"{variant}_seed{seed}"
            train_cmd = [
                sys.executable,
                str(train_py),
                "--task",
                task,
                "--experiment_name",
                EXPERIMENT_NAME,
                "--run_name",
                run_name,
                "--num_envs",
                str(args.num_envs),
                "--max_iterations",
                str(args.iterations),
                "--seed",
                str(seed),
                "env.curriculum_stage=1",
                "env.terrain_mode=flat",
                *args.env_override,
            ]
            if args.device is not None:
                train_cmd.extend(["--device", args.device])
            if args.headless:
                train_cmd.append("--headless")
            print(" ".join(train_cmd))
            if args.dry_run:
                continue

            previous_runs = {path for path in log_root.iterdir() if path.is_dir()} if log_root.is_dir() else set()
            subprocess.run(train_cmd, cwd=repo, check=True)
            run_dir = _new_run(log_root, previous_runs, run_name)
            checkpoint = _latest_checkpoint(run_dir)
            benchmark_output = output_dir / f"{variant}_seed{seed}.json"
            benchmark_cmd = [
                sys.executable,
                str(benchmark_py),
                "--task",
                task,
                "--checkpoint",
                str(checkpoint),
                "--num_envs",
                str(args.benchmark_num_envs),
                "--num_steps",
                str(args.benchmark_num_steps),
                "--terrain",
                "flat",
                "--seed",
                str(seed),
                "--scenarios",
                *SCENARIOS,
                "--json-output",
                str(benchmark_output),
                *args.env_override,
            ]
            if args.device is not None:
                benchmark_cmd.extend(["--device", args.device])
            if args.headless:
                benchmark_cmd.append("--headless")
            print(" ".join(benchmark_cmd))
            subprocess.run(benchmark_cmd, cwd=repo, check=True)
            records.append(
                {
                    "variant": variant,
                    "task": task,
                    "seed": seed,
                    "run": str(run_dir),
                    "checkpoint": str(checkpoint),
                    "benchmark": json.loads(benchmark_output.read_text(encoding="utf-8")),
                }
            )

    if args.dry_run:
        return
    summary = _summarize(records)
    payload = {
        "experiment": EXPERIMENT_NAME,
        "env_override": args.env_override,
        "iterations": args.iterations,
        "num_envs": args.num_envs,
        "records": records,
        "summary": summary,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _print_summary(summary)
    print(f"Wrote paired A/B summary: {summary_path}")


if __name__ == "__main__":
    main()
