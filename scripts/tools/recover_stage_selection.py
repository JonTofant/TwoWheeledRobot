#!/usr/bin/env python3
"""Finish a curriculum stage whose checkpoint selection was interrupted.

train_nn_drive_curriculum.py benchmarks each shortlisted checkpoint in its own
Kit subprocess and writes benchmark_selection/<checkpoint>.json as it goes. If
one of those launches dies (Kit has been seen to SIGSEGV during startup, before
the sim loads), the whole curriculum aborts -- discarding a finished multi-hour
arm even though every stage had already trained and most candidates had already
been benchmarked.

This script resumes from that point: it runs only the benchmarks whose JSON is
missing, then reproduces the selection and writes SELECTION_MANIFEST exactly as
select_checkpoint() would have. Scoring and gating are imported from the same
modules the curriculum uses, so the two cannot drift apart.

Usage:
  python scripts/tools/recover_stage_selection.py \
      --run-dir logs/rsl_rl/nn_drive_fixed_stance/<run> --stage 5 \
      --task <task-id> [--allow-gate-failure] [env.override=...]
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts"))

from nn_drive_benchmark_contract import STAGE_GATE_SCENARIOS, checkpoint_score, stage_gate_failures  # noqa: E402
from train_nn_drive_curriculum import (  # noqa: E402
    SELECTION_MANIFEST,
    STAGE_TERRAIN,
    checkpoint_iteration,
    shortlist_checkpoints,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--stage", type=int, required=True, choices=[1, 2, 3, 4, 5])
    parser.add_argument("--task", type=str, required=True)
    parser.add_argument("--selection-candidates", type=int, default=3)
    parser.add_argument("--benchmark-num-envs", type=int, default=64)
    parser.add_argument("--benchmark-num-steps", type=int, default=1000)
    parser.add_argument("--benchmark-seed", type=int, default=42)
    parser.add_argument("--allow-gate-failure", action="store_true")
    args, env_overrides = parser.parse_known_args()

    run_dir = args.run_dir if args.run_dir.is_absolute() else REPO / args.run_dir
    bench_dir = run_dir / "benchmark_selection"
    bench_dir.mkdir(parents=True, exist_ok=True)

    # Reuse the curriculum's own shortlist so the candidate set is identical to
    # the one the interrupted run was working through.
    candidates = shortlist_checkpoints(run_dir, args.selection_candidates)
    print(f"Shortlist: {', '.join(c.name for c in candidates)}")

    for checkpoint in candidates:
        output = bench_dir / f"{checkpoint.stem}.json"
        if output.is_file():
            print(f"Reusing existing benchmark for {checkpoint.name}")
            continue
        cmd = [
            sys.executable,
            str(REPO / "scripts" / "benchmark_nn_drive.py"),
            "--task", args.task,
            "--checkpoint", str(checkpoint),
            "--num_envs", str(args.benchmark_num_envs),
            "--num_steps", str(args.benchmark_num_steps),
            "--terrain", STAGE_TERRAIN[args.stage],
            "--seed", str(args.benchmark_seed),
            "--scenarios", *STAGE_GATE_SCENARIOS[args.stage],
            "--json-output", str(output),
            "--headless",
            *env_overrides,
        ]
        print(f"Re-running missing benchmark for {checkpoint.name}")
        subprocess.run(cmd, cwd=REPO, check=True)

    evaluated = []
    passing = []
    for checkpoint in candidates:
        payload = json.loads((bench_dir / f"{checkpoint.stem}.json").read_text(encoding="utf-8"))
        results = payload["scenarios"]
        failures = stage_gate_failures(args.stage, results)
        score = checkpoint_score(results)
        evaluated.append(
            {
                "checkpoint": checkpoint.name,
                "iteration": checkpoint_iteration(checkpoint),
                "score": score,
                "gate_passed": not failures,
                "gate_failures": failures,
                "benchmark": payload,
            }
        )
        if not failures:
            passing.append((score, checkpoint))

    selected = min(passing, default=None, key=lambda item: item[0])
    used_fallback = False
    if selected is None and args.allow_gate_failure and evaluated:
        best = min(evaluated, key=lambda c: c["score"])
        selected = (best["score"], run_dir / best["checkpoint"])
        used_fallback = True
        print(
            f"WARNING: stage {args.stage} gate failed for every candidate; falling back to "
            f"best-scoring {best['checkpoint']} (score={best['score']:.6f}) per --allow-gate-failure. "
            "This checkpoint did NOT pass its gate -- see gate_failures in the manifest."
        )

    manifest = {
        "stage": args.stage,
        "run": run_dir.name,
        "selected_checkpoint": selected[1].name if selected is not None else None,
        "selection_score": selected[0] if selected is not None else None,
        "selected_via_gate_failure_fallback": used_fallback,
        "candidates": evaluated,
        # Present only on recovered stages, so a manifest written by this path is
        # never mistaken for one the curriculum produced end-to-end.
        "recovered_by": "scripts/tools/recover_stage_selection.py",
    }
    (run_dir / SELECTION_MANIFEST).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if selected is None:
        details = "\n".join(f"  {c['checkpoint']}: " + "; ".join(c["gate_failures"]) for c in evaluated)
        raise SystemExit(f"Stage {args.stage} gate failed for every shortlisted checkpoint:\n{details}")
    print(f"Selected {selected[1]} for stage {args.stage}: benchmark score={selected[0]:.6f}")


if __name__ == "__main__":
    main()
