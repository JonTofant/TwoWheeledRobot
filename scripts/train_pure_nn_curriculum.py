#!/usr/bin/env python3
"""Run the five-stage pure NN balance curriculum with the existing RSL-RL trainer."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def latest_run(log_root: Path) -> str | None:
    runs = [p for p in log_root.iterdir() if p.is_dir()] if log_root.is_dir() else []
    if not runs:
        return None
    return max(runs, key=lambda p: p.stat().st_mtime).name


def latest_checkpoint(run_dir: Path) -> Path | None:
    checkpoints = list(run_dir.glob("model_*.pt"))
    if not checkpoints:
        return None
    return max(checkpoints, key=lambda p: p.stat().st_mtime)


def main() -> None:
    parser = argparse.ArgumentParser(description="Train pure NN balance policy through stages 1..5.")
    parser.add_argument("--num_envs", type=int, default=4096)
    parser.add_argument("--iterations", type=int, nargs=5, default=[50, 50, 50, 50, 50])
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--i-max-a", type=float, default=2.0)
    parser.add_argument("--skip-export", action="store_true")
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    train_py = repo / "scripts" / "rsl_rl" / "train.py"
    log_root = repo / "logs" / "rsl_rl" / "pure_nn_balance_two_wheel"
    load_run = latest_run(log_root)

    for stage, iterations in enumerate(args.iterations, start=1):
        cmd = [
            sys.executable,
            str(train_py),
            "--task",
            "Template-Twowheeledrobot-PureNNBalance-v0",
            "--num_envs",
            str(args.num_envs),
            "--max_iterations",
            str(iterations),
            "--run_name",
            f"stage{stage}",
            f"env.curriculum_stage={stage}",
        ]
        if args.headless:
            cmd.append("--headless")
        if args.seed is not None:
            cmd.extend(["--seed", str(args.seed)])
        if stage > 1 and load_run is not None:
            cmd.extend(["--resume", "--load_run", load_run])
        print(" ".join(cmd))
        if not args.dry_run:
            subprocess.run(cmd, cwd=repo, check=True)
            load_run = latest_run(log_root)

    if load_run is not None:
        run_dir = log_root / load_run
        checkpoint = latest_checkpoint(run_dir)
        print(f"Best/latest checkpoint run: {run_dir}")
        if checkpoint is not None and not args.skip_export:
            play_cmd = [
                sys.executable,
                str(repo / "scripts" / "rsl_rl" / "play.py"),
                "--task",
                "Template-Twowheeledrobot-PureNNBalance-v0",
                "--checkpoint",
                str(checkpoint),
                "--num_envs",
                "1",
                "--num_steps",
                "1",
            ]
            if args.headless:
                play_cmd.append("--headless")
            print(" ".join(play_cmd))
            if not args.dry_run:
                subprocess.run(play_cmd, cwd=repo, check=True)

            exported_policy = run_dir / "exported" / "policy.pt"
            current_onnx = run_dir / "exported" / "policy_current.onnx"
            export_cmd = [
                sys.executable,
                str(repo / "scripts" / "export_pure_nn_current_onnx.py"),
                "--policy",
                str(exported_policy),
                "--output",
                str(current_onnx),
                "--i-max-a",
                str(args.i_max_a),
            ]
            print(" ".join(export_cmd))
            if not args.dry_run:
                subprocess.run(export_cmd, cwd=repo, check=True)


if __name__ == "__main__":
    main()
