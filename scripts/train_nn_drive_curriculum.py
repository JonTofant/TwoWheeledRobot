#!/usr/bin/env python3
"""Train NNDrive through five benchmark-gated curriculum stages.

Each stage saves frequent checkpoints, shortlists them by training reward, and
runs a deterministic frozen-policy benchmark. Only a checkpoint that passes the
stage gate is promoted. The selected checkpoint—not the newest file—is resumed
by the next stage and exported after the final stage.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

from nn_drive_benchmark_contract import STAGE_GATE_SCENARIOS, checkpoint_score, stage_gate_failures

STAGE_TERRAIN = {1: "flat", 2: "flat", 3: "flat", 4: "generator", 5: "generator"}
SELECTION_MANIFEST = "selected_checkpoint.json"


def checkpoint_iteration(checkpoint: Path) -> int:
    match = re.fullmatch(r"model_(\d+)\.pt", checkpoint.name)
    if match is None:
        raise ValueError(f"Unexpected checkpoint name: {checkpoint.name}")
    return int(match.group(1))


def newly_created_run(log_root: Path, previous_runs: set[Path], stage: int) -> Path:
    new_runs = {path for path in log_root.iterdir() if path.is_dir()} - previous_runs
    matching = sorted(path for path in new_runs if path.name.endswith(f"_stage{stage}"))
    if len(matching) != 1:
        names = ", ".join(path.name for path in matching) or "none"
        raise RuntimeError(f"Expected exactly one new stage-{stage} run directory; found: {names}")
    return matching[0]


def shortlist_checkpoints(run_dir: Path, count: int) -> list[Path]:
    """Rank saved checkpoints by the matching TensorBoard mean-reward scalar.

    Raw reward is biased toward standstill policies when
    cmd_still_episode_prob is high (see nn_drive_env_cfg.py), so a converged
    driving checkpoint can rank below a standstill-attractor checkpoint on
    reward alone. The benchmark gate is the real filter; this function's job
    is only to not withhold a legitimate candidate from it, so the newest
    checkpoint is always included regardless of its reward rank.
    """
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    except ModuleNotFoundError as exc:
        raise RuntimeError("TensorBoard is required for checkpoint shortlisting") from exc

    rewards_by_step: dict[int, float] = {}
    event_files = sorted(run_dir.glob("events.out.tfevents.*"))
    if not event_files:
        raise RuntimeError(f"No TensorBoard event file found in {run_dir}")
    for event_file in event_files:
        accumulator = EventAccumulator(str(event_file), size_guidance={"scalars": 0})
        accumulator.Reload()
        if "Train/mean_reward" not in accumulator.Tags().get("scalars", []):
            continue
        rewards_by_step.update((event.step, event.value) for event in accumulator.Scalars("Train/mean_reward"))

    ranked: list[tuple[float, int, Path]] = []
    missing: list[str] = []
    for checkpoint in run_dir.glob("model_*.pt"):
        iteration = checkpoint_iteration(checkpoint)
        if iteration not in rewards_by_step:
            missing.append(checkpoint.name)
            continue
        ranked.append((rewards_by_step[iteration], iteration, checkpoint))
    if not ranked:
        detail = f"; missing scalar steps for {', '.join(sorted(missing))}" if missing else ""
        raise RuntimeError(f"No checkpoint could be matched to Train/mean_reward in {run_dir}{detail}")
    ranked.sort(reverse=True)
    top_by_reward = [checkpoint for _, _, checkpoint in ranked[:count]]
    newest = max(run_dir.glob("model_*.pt"), key=checkpoint_iteration, default=None)
    if newest is not None and newest not in top_by_reward:
        top_by_reward.append(newest)
    return top_by_reward


def benchmark_checkpoint(
    *,
    repo: Path,
    checkpoint: Path,
    stage: int,
    terrain: str,
    num_envs: int,
    num_steps: int,
    seed: int,
    headless: bool,
) -> dict:
    benchmark_dir = checkpoint.parent / "benchmark_selection"
    output = benchmark_dir / f"{checkpoint.stem}.json"
    cmd = [
        sys.executable,
        str(repo / "scripts" / "benchmark_nn_drive.py"),
        "--checkpoint",
        str(checkpoint),
        "--num_envs",
        str(num_envs),
        "--num_steps",
        str(num_steps),
        "--terrain",
        terrain,
        "--seed",
        str(seed),
        "--scenarios",
        *STAGE_GATE_SCENARIOS[stage],
        "--json-output",
        str(output),
    ]
    if headless:
        cmd.append("--headless")
    print(" ".join(cmd))
    subprocess.run(cmd, cwd=repo, check=True)
    return json.loads(output.read_text(encoding="utf-8"))


def select_checkpoint(
    *,
    repo: Path,
    run_dir: Path,
    stage: int,
    shortlist_count: int,
    benchmark_num_envs: int,
    benchmark_num_steps: int,
    benchmark_seed: int,
    headless: bool,
) -> Path:
    candidates = shortlist_checkpoints(run_dir, shortlist_count)
    evaluated = []
    passing = []
    for checkpoint in candidates:
        payload = benchmark_checkpoint(
            repo=repo,
            checkpoint=checkpoint,
            stage=stage,
            terrain=STAGE_TERRAIN[stage],
            num_envs=benchmark_num_envs,
            num_steps=benchmark_num_steps,
            seed=benchmark_seed,
            headless=headless,
        )
        results = payload["scenarios"]
        failures = stage_gate_failures(stage, results)
        score = checkpoint_score(results)
        candidate = {
            "checkpoint": checkpoint.name,
            "iteration": checkpoint_iteration(checkpoint),
            "score": score,
            "gate_passed": not failures,
            "gate_failures": failures,
            "benchmark": payload,
        }
        evaluated.append(candidate)
        if not failures:
            passing.append((score, checkpoint))

    selected = min(passing, default=None, key=lambda item: item[0])
    manifest = {
        "stage": stage,
        "run": run_dir.name,
        "selected_checkpoint": selected[1].name if selected is not None else None,
        "selection_score": selected[0] if selected is not None else None,
        "candidates": evaluated,
    }
    manifest_path = run_dir / SELECTION_MANIFEST
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if selected is None:
        details = "\n".join(
            f"  {candidate['checkpoint']}: " + "; ".join(candidate["gate_failures"]) for candidate in evaluated
        )
        raise RuntimeError(f"Stage {stage} gate failed for every shortlisted checkpoint:\n{details}")
    print(f"Selected {selected[1]} for stage {stage}: benchmark score={selected[0]:.6f}")
    return selected[1]


def selected_checkpoint_from_manifest(run_dir: Path) -> Path:
    manifest_path = run_dir / SELECTION_MANIFEST
    if not manifest_path.is_file():
        raise RuntimeError(
            f"No {SELECTION_MANIFEST} in {run_dir}; pass --load-checkpoint explicitly after benchmarking the run"
        )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    name = manifest.get("selected_checkpoint")
    if not name:
        raise RuntimeError(f"Selection manifest contains no passing checkpoint: {manifest_path}")
    return run_dir / name


def main() -> None:
    parser = argparse.ArgumentParser(description="Train NNDrive through benchmark-gated stages.")
    parser.add_argument("--num_envs", type=int, default=4096)
    parser.add_argument("--iterations", type=int, nargs=5, default=[200, 350, 200, 250, 300])
    parser.add_argument("--start-stage", type=int, default=1, choices=[1, 2, 3, 4, 5])
    parser.add_argument("--load-run", type=str, default=None, help="Explicit run to resume when --start-stage > 1")
    parser.add_argument(
        "--load-checkpoint",
        type=str,
        default=None,
        help=f"Checkpoint within --load-run; defaults to its {SELECTION_MANIFEST}",
    )
    parser.add_argument("--selection-candidates", type=int, default=3)
    parser.add_argument("--benchmark-num-envs", type=int, default=64)
    parser.add_argument("--benchmark-num-steps", type=int, default=1000)
    parser.add_argument("--benchmark-seed", type=int, default=42)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--i-max-a", type=float, default=2.0)
    parser.add_argument("--skip-export", action="store_true")
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.selection_candidates < 1:
        parser.error("--selection-candidates must be at least 1")
    if args.start_stage > 1 and args.load_run is None:
        parser.error("--load-run is required when --start-stage > 1; newest-run fallback is intentionally disabled")

    repo = Path(__file__).resolve().parents[1]
    train_py = repo / "scripts" / "rsl_rl" / "train.py"
    log_root = repo / "logs" / "rsl_rl" / "nn_drive_two_wheel"
    selected_run: Path | None = None
    selected_checkpoint: Path | None = None
    if args.load_run is not None:
        selected_run = log_root / args.load_run
        if not selected_run.is_dir():
            parser.error(f"--load-run directory does not exist: {selected_run}")
        selected_checkpoint = (
            selected_run / args.load_checkpoint
            if args.load_checkpoint is not None
            else selected_checkpoint_from_manifest(selected_run)
        )
        if not selected_checkpoint.is_file():
            parser.error(f"Resume checkpoint does not exist: {selected_checkpoint}")

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
        if selected_run is not None and selected_checkpoint is not None:
            cmd.extend(["--resume", "--load_run", selected_run.name, "--checkpoint", selected_checkpoint.name])
        print(" ".join(cmd))
        if args.dry_run:
            print("DRY RUN: checkpoint selection and later stages require the completed run; stopping here.")
            return

        previous_runs = {path for path in log_root.iterdir() if path.is_dir()} if log_root.is_dir() else set()
        subprocess.run(cmd, cwd=repo, check=True)
        selected_run = newly_created_run(log_root, previous_runs, stage)
        selected_checkpoint = select_checkpoint(
            repo=repo,
            run_dir=selected_run,
            stage=stage,
            shortlist_count=args.selection_candidates,
            benchmark_num_envs=args.benchmark_num_envs,
            benchmark_num_steps=args.benchmark_num_steps,
            benchmark_seed=args.benchmark_seed,
            headless=args.headless,
        )

    if selected_run is None or selected_checkpoint is None:
        raise RuntimeError("No checkpoint was selected")
    if args.skip_export:
        return

    play_cmd = [
        sys.executable,
        str(repo / "scripts" / "rsl_rl" / "play.py"),
        "--task",
        "Template-Twowheeledrobot-NNDrive-v0",
        "--checkpoint",
        str(selected_checkpoint),
        "--num_envs",
        "1",
        "--num_steps",
        "1",
    ]
    if args.headless:
        play_cmd.append("--headless")
    print(" ".join(play_cmd))
    subprocess.run(play_cmd, cwd=repo, check=True)

    exported_policy = selected_run / "exported" / "policy.pt"
    actor_onnx = selected_run / "exported" / "policy.onnx"
    if not exported_policy.is_file() or not actor_onnx.is_file():
        raise RuntimeError(f"play.py did not produce both required actor exports in {exported_policy.parent}")
    drive_onnx = selected_run / "exported" / "policy_drive.onnx"
    export_cmd = [
        sys.executable,
        str(repo / "scripts" / "export_pure_nn_current_onnx.py"),
        "--policy",
        str(exported_policy),
        "--actor-onnx",
        str(actor_onnx),
        "--output",
        str(drive_onnx),
        "--obs-dim",
        "20",
        "--cg-outputs",
        "4",
        "--i-max-a",
        str(args.i_max_a),
        "--require-validation",
    ]
    print(" ".join(export_cmd))
    subprocess.run(export_cmd, cwd=repo, check=True)


if __name__ == "__main__":
    main()
