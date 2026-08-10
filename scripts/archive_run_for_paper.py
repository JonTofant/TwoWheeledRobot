#!/usr/bin/env python3
"""Copy the small, paper-relevant artifacts of one or more training runs into
ExportedPolicy/<name>/, which (unlike logs/) is git-tracked.

Deliberately excludes the large per-iteration model_*.pt checkpoints (dozens
of near-duplicate snapshots per run) -- only the final numbered checkpoint and
the play.py/export_pure_nn_current_onnx.py outputs are kept, alongside
everything needed to reproduce or audit the run:

  params/env.yaml, params/agent.yaml   resolved config (every DR range,
                                        reward weight, curriculum stage, seed)
  selected_checkpoint.json             candidate lineage + embedded raw
                                        benchmark JSON (curriculum-gated runs)
  benchmark_selection/*.json           same benchmark JSON, standalone
  git/*.diff                           exact code state (commit + uncommitted
                                        diff) -- see CLAUDE.md on the git
                                        ownership fix this needed
  events.out.tfevents.*                TensorBoard scalars (reward curve etc.)
  exported/policy.pt, policy.onnx,
  exported/policy_drive.onnx           deployable artifacts

Usage:
    python scripts/archive_run_for_paper.py \\
        --dest fixedstance_range_2026-08-10 \\
        logs/rsl_rl/nn_drive_fixed_stance/2026-08-10_*_stage1 \\
        logs/rsl_rl/nn_drive_fixed_stance/2026-08-10_*_stage2 \\
        ...

Each run directory is copied under ExportedPolicy/<dest>/<run-dir-name>/, so a
full 5-stage curriculum archives as 5 stage subfolders under one destination.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path

KEEP_FILES = ["selected_checkpoint.json"]
KEEP_GLOBS = ["events.out.tfevents.*"]
KEEP_DIRS = ["params", "git", "benchmark_selection", "exported"]


def latest_numbered_checkpoint(run_dir: Path) -> Path | None:
    import re

    best_iter, best_path = -1, None
    for path in run_dir.glob("model_*.pt"):
        match = re.fullmatch(r"model_(\d+)\.pt", path.name)
        if match and int(match.group(1)) > best_iter:
            best_iter, best_path = int(match.group(1)), path
    return best_path


def archive_run(run_dir: Path, dest_dir: Path) -> list[str]:
    if not run_dir.is_dir():
        raise FileNotFoundError(f"Not a directory: {run_dir}")
    dest_dir.mkdir(parents=True, exist_ok=True)
    copied = []

    for name in KEEP_FILES:
        src = run_dir / name
        if src.is_file():
            shutil.copy2(src, dest_dir / name)
            copied.append(name)

    for pattern in KEEP_GLOBS:
        for src in run_dir.glob(pattern):
            shutil.copy2(src, dest_dir / src.name)
            copied.append(src.name)

    for name in KEEP_DIRS:
        src = run_dir / name
        if src.is_dir():
            dst = dest_dir / name
            if dst.exists():
                shutil.rmtree(dst)
            shutil.copytree(src, dst)
            copied.append(name + "/")

    final_ckpt = latest_numbered_checkpoint(run_dir)
    if final_ckpt is not None:
        shutil.copy2(final_ckpt, dest_dir / final_ckpt.name)
        copied.append(final_ckpt.name)

    return copied


def write_manifest(dest_root: Path, run_dirs: list[Path], repo: Path) -> None:
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=repo, capture_output=True, text=True, check=False
    ).stdout.strip()
    dirty = subprocess.run(
        ["git", "status", "--short"], cwd=repo, capture_output=True, text=True, check=False
    ).stdout.strip()
    lines = [
        f"# {dest_root.name}",
        "",
        f"Archived from: {', '.join(str(r) for r in run_dirs)}",
        f"Repo commit at archive time: {commit or 'unknown (not a git repo?)'}",
        f"Repo dirty at archive time: {'yes -- see below' if dirty else 'no'}",
    ]
    if dirty:
        lines += ["", "```", dirty, "```"]
    lines += [
        "",
        "Each subfolder below is one training run (one curriculum stage, or a",
        "standalone run). See its params/env.yaml and params/agent.yaml for the",
        "exact resolved config and seed, git/*.diff for the exact code state,",
        "and selected_checkpoint.json / benchmark_selection/*.json for raw",
        "benchmark results. See scripts/archive_run_for_paper.py for what was",
        "deliberately excluded (intermediate model_*.pt checkpoints).",
    ]
    (dest_root / "MANIFEST.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dirs", nargs="+", type=Path, help="One or more logs/rsl_rl/.../<run> directories")
    parser.add_argument("--dest", type=str, required=True, help="Destination name under ExportedPolicy/")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    dest_root = repo / "ExportedPolicy" / args.dest
    for run_dir in args.run_dirs:
        run_dir = run_dir.resolve()
        dest_dir = dest_root / run_dir.name
        copied = archive_run(run_dir, dest_dir)
        print(f"{run_dir} -> {dest_dir}")
        for item in copied:
            print(f"  {item}")
        if not copied:
            print("  (nothing found to copy -- is this the right run directory?)")

    write_manifest(dest_root, [r.resolve() for r in args.run_dirs], repo)
    print(f"\nWrote {dest_root / 'MANIFEST.md'}")
    print(f"Review with: git status {dest_root}")
    print(f"Then: git add {dest_root} && git commit ...")


if __name__ == "__main__":
    main()
