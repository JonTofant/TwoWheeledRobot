#!/usr/bin/env python3
"""Run the five-stage NN drive curriculum with the existing RSL-RL trainer.

Stages:
  1  flat terrain, no commands, no disturbances     — learn to balance with legs
  2  flat terrain, small commands                   — learn to drive and turn
  3  flat terrain, full commands + pushes           — robustness while driving
  4  generated terrain (bumps + slopes), commands   — terrain driving
  5  generated terrain, full commands + all pushes/payloads + force noise

Each stage resumes from the latest checkpoint of the previous one. At the end
the policy is exported to TorchScript/ONNX with the 6-output deployment scaling
(4 CyberGear target radians + 2 wheel currents in ampere).
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

STAGE_TERRAIN = {1: "flat", 2: "flat", 3: "flat", 4: "generator", 5: "generator"}


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
    parser = argparse.ArgumentParser(description="Train NN drive policy through stages 1..5.")
    parser.add_argument("--num_envs", type=int, default=4096)
    # Sized from where each stage actually stops improving (measured on the
    # 2026-07-27 run): stage 1 plateaus at ~60% of its budget, stage 2 ~79%,
    # stage 4 ~32%, and stage 5 is converged by ~100 iterations — its last 700
    # iterations moved fall rate/pitch/roll/pos_err by under 3%. The old
    # [300,400,500,600,800] cost 280 min; this costs ~137 min.
    parser.add_argument("--iterations", type=int, nargs=5, default=[200, 350, 200, 250, 300])
    parser.add_argument("--start-stage", type=int, default=1, choices=[1, 2, 3, 4, 5])
    parser.add_argument(
        "--load-run",
        type=str,
        default=None,
        help=(
            "Run directory to resume the FIRST executed stage from. Without it the "
            "resume point is whichever run has the newest mtime, so repeated "
            "single-stage experiments silently chain off each other instead of off a "
            "fixed baseline. Pin this for A/B comparisons, e.g. "
            "--start-stage 5 --load-run 2026-07-27_15-14-04_stage4"
        ),
    )
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--i-max-a", type=float, default=2.0)
    parser.add_argument("--cg-authority-rad", type=float, default=0.45)
    parser.add_argument("--skip-export", action="store_true")
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    train_py = repo / "scripts" / "rsl_rl" / "train.py"
    log_root = repo / "logs" / "rsl_rl" / "nn_drive_two_wheel"
    # An explicit --load-run pins the baseline; otherwise fall back to newest-mtime.
    # Subsequent stages still chain off the run this invocation just produced.
    load_run = args.load_run or latest_run(log_root)
    if args.load_run is not None and not (log_root / args.load_run).is_dir():
        parser.error(f"--load-run directory does not exist: {log_root / args.load_run}")

    for stage, iterations in enumerate(args.iterations, start=1):
        if stage < args.start_stage:
            continue
        cmd = [
            sys.executable,
            str(train_py),
            "--task",
            "Template-Twowheeledrobot-NNDrive-v0",
            "--num_envs",
            str(args.num_envs),
            "--max_iterations",
            str(iterations),
            "--run_name",
            f"stage{stage}",
            f"env.curriculum_stage={stage}",
            f"env.terrain_mode={STAGE_TERRAIN[stage]}",
        ]
        if args.headless:
            cmd.append("--headless")
        if args.seed is not None:
            cmd.extend(["--seed", str(args.seed)])
        if (stage > 1 or args.start_stage > 1) and load_run is not None:
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
                "Template-Twowheeledrobot-NNDrive-v0",
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
            drive_onnx = run_dir / "exported" / "policy_drive.onnx"
            export_cmd = [
                sys.executable,
                str(repo / "scripts" / "export_pure_nn_current_onnx.py"),
                "--policy",
                str(exported_policy),
                "--output",
                str(drive_onnx),
                "--obs-dim",
                "18",
                "--cg-outputs",
                "4",
                "--cg-authority-rad",
                str(args.cg_authority_rad),
                "--i-max-a",
                str(args.i_max_a),
            ]
            print(" ".join(export_cmd))
            if not args.dry_run:
                subprocess.run(export_cmd, cwd=repo, check=True)


if __name__ == "__main__":
    main()
