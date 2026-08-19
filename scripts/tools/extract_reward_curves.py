#!/usr/bin/env python3
"""Extract Train/mean_reward curves from curriculum stage runs into one CSV.

docs/paper/figures/data/reward_curves.csv was originally produced by hand for
the seed-42 arms; make_paper_figures.py consumes it. This script reproduces
that extraction mechanically so additional seeds can be added without a manual
step, and adds a `seed` column (seed-42 rows written by the old hand process
have no such column -- see --legacy-no-seed-column).

Verify against the original before trusting it on new data:

  python scripts/tools/extract_reward_curves.py --check docs/paper/figures/data/reward_curves.csv \\
      --arm A=ExportedPolicy/fixedstance_point_2026-08-10 \\
      --arm B=ExportedPolicy/fixedstance_range_2026-08-10 \\
      --arm C=ExportedPolicy/fixedstance_range_gru_2026-08-10 --legacy-no-seed-column

Normal use, one --arm per arm/seed, value is the directory holding that arm's
five *_stage<N> run directories:

  python scripts/tools/extract_reward_curves.py --output <csv> \\
      --arm A:42=ExportedPolicy/fixedstance_point_2026-08-10 \\
      --arm A:43=logs/rsl_rl/nn_drive_fixed_stance --run-glob '*_point_s43_stage*'
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def stage_of(run_dir: Path) -> int:
    match = re.search(r"_stage(\d+)$", run_dir.name)
    if match is None:
        raise ValueError(f"Not a stage run directory: {run_dir}")
    return int(match.group(1))


def scalars(run_dir: Path) -> list[tuple[int, float]]:
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

    points: dict[int, float] = {}
    event_files = sorted(run_dir.glob("events.out.tfevents.*"))
    if not event_files:
        raise RuntimeError(f"No TensorBoard event file in {run_dir}")
    for event_file in event_files:
        acc = EventAccumulator(str(event_file), size_guidance={"scalars": 0})
        acc.Reload()
        if "Train/mean_reward" not in acc.Tags().get("scalars", []):
            continue
        points.update((e.step, e.value) for e in acc.Scalars("Train/mean_reward"))
    if not points:
        raise RuntimeError(f"No Train/mean_reward scalars in {run_dir}")
    return sorted(points.items())


def collect(arm: str, seed: str | None, run_dirs: list[Path]) -> list[dict]:
    rows: list[dict] = []
    offset = 0
    for run_dir in sorted(run_dirs, key=stage_of):
        stage = stage_of(run_dir)
        points = scalars(run_dir)
        # iteration_in_stage is RSL-RL's raw counter, deliberately NOT
        # re-anchored to the resume point: it continues from whichever
        # checkpoint the stage resumed at, and make_paper_figures.py does its
        # own per-stage re-anchoring at plot time. Re-anchoring here too would
        # shift every stage twice.
        #
        # cumulative_iteration just lays the stages end-to-end in order. It is
        # vestigial -- the plotting script never reads it, and because the raw
        # counters differ per arm it is not comparable across arms -- but it is
        # reproduced exactly so --check can diff against the original CSV.
        last = offset
        for step, value in points:
            last = offset + step
            row = {
                "arm": arm,
                "stage": stage,
                "iteration_in_stage": step,
                "cumulative_iteration": last,
                "reward": value,
            }
            if seed is not None:
                row["seed"] = seed
            rows.append(row)
        offset = last + 1
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--arm",
        action="append",
        required=True,
        metavar="ARM[:SEED]=DIR",
        help="Arm letter, optional seed, and the directory containing its *_stage<N> run dirs",
    )
    parser.add_argument("--run-glob", default="*_stage*", help="Glob for stage run dirs inside each --arm DIR")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--check", type=Path, default=None, help="Compare against an existing CSV instead of writing")
    parser.add_argument(
        "--legacy-no-seed-column",
        action="store_true",
        help="Omit the seed column, matching the original hand-made reward_curves.csv",
    )
    args = parser.parse_args()

    rows: list[dict] = []
    for spec in args.arm:
        key, _, dirname = spec.partition("=")
        arm, _, seed = key.partition(":")
        base = Path(dirname) if Path(dirname).is_absolute() else REPO / dirname
        run_dirs = [p for p in base.glob(args.run_glob) if p.is_dir() and re.search(r"_stage\d+$", p.name)]
        if not run_dirs:
            raise SystemExit(f"No stage run directories matched {base}/{args.run_glob}")
        rows.extend(collect(arm, None if args.legacy_no_seed_column else (seed or None), run_dirs))

    fields = ["arm", "stage", "iteration_in_stage", "cumulative_iteration", "reward"]
    if not args.legacy_no_seed_column and any("seed" in r for r in rows):
        fields.append("seed")

    if args.check is not None:
        existing = list(csv.DictReader(open(args.check)))
        if len(existing) != len(rows):
            raise SystemExit(f"Row count differs: existing {len(existing)}, extracted {len(rows)}")
        worst = 0.0
        for a, b in zip(existing, rows):
            for k in ("arm", "stage", "iteration_in_stage", "cumulative_iteration"):
                if str(a[k]) != str(b[k]):
                    raise SystemExit(f"Mismatch on {k}: {a} vs {b}")
            worst = max(worst, abs(float(a["reward"]) - float(b["reward"])))
        print(f"MATCH: {len(rows)} rows identical, max reward delta {worst:.3e}")
        return

    if args.output is None:
        raise SystemExit("--output is required unless --check is given")
    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {len(rows)} rows to {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
