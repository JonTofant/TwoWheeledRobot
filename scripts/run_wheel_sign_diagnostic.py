#!/usr/bin/env python3
"""Run the left/right wheel sign diagnostic through scripts/lqr_control.py."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--current-a", type=float, default=0.20, help="Positive current for each one-wheel phase.")
    parser.add_argument("--phase-time-s", type=float, default=0.75, help="Seconds for left, coast, and right phases.")
    parser.add_argument("--headless", action="store_true", help="Forward --headless to Isaac Sim.")
    parser.add_argument("--no-plot", action="store_true", default=True, help="Disable live plots by default.")
    parser.add_argument("--log-csv", type=Path, default=Path("logs/wheel_sign_diagnostic.csv"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    command = [
        sys.executable,
        str(repo_root / "scripts" / "lqr_control.py"),
        "--test-mode",
        "free-spin",
        "--sign-diagnostic",
        "--sign-diagnostic-current-a",
        str(args.current_a),
        "--sign-diagnostic-phase-time-s",
        str(args.phase_time_s),
        "--log-csv",
        str(args.log_csv),
    ]
    if args.no_plot:
        command.append("--no-plot")
    if args.headless:
        command.append("--headless")
    print("[RUN]", " ".join(command))
    return subprocess.call(command, cwd=repo_root)


if __name__ == "__main__":
    raise SystemExit(main())
