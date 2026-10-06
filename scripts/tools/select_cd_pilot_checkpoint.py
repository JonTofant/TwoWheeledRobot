#!/usr/bin/env python3
"""Validate predetermined C/D pilot candidates and export a passing policy.

Run inside Isaac Lab's container Python. Simulation jobs are child processes;
this coordinator does not import Isaac or initialize Kit itself.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts"))
from nn_drive_benchmark_contract import SCENARIOS, checkpoint_score, stage_gate_failures  # noqa: E402
from train_nn_drive_curriculum import checkpoint_iteration  # noqa: E402

TASKS = {
    "C": "Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0",
    "D": "Template-Twowheeledrobot-NNDriveFixedStanceWide-v0",
}
NOMINAL = ["env.motor_gain_range=[1.0,1.0]", "env.motor_deadzone_a_range=[0.0534,0.0534]"]


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_protocol(protocol: dict) -> None:
    expected = {
        "training_seed": 43,
        "num_envs": 4096,
        "stage_update_budgets": [300, 350, 200, 250, 150],
        "total_update_budget": 1250,
        "num_steps_per_env": 64,
        "checkpoint_save_interval": 25,
        "exploration_floor": [0.15, 0.15],
    }
    for key, value in expected.items():
        if protocol.get(key) != value:
            raise ValueError(f"Pilot protocol mismatch: {key}: {protocol.get(key)!r} != {value!r}")
    if list(protocol["arms"]) != ["C", "D"]:
        raise ValueError("This pilot must cover C and D, in that order")
    validation = protocol["deployment_selection"]
    if (validation["validation_base_seed"], validation["num_envs"], validation["num_steps"],
            validation["terrain"]) != (1001, 64, 1000, "generator"):
        raise ValueError("Unexpected validation settings")
    if protocol["reporting"]["test_base_seed"] != 2001:
        raise ValueError("Test and validation seeds must remain separate")


def candidates(stage4: Path, stage5: Path, budget: int) -> list[Path]:
    end4 = max(stage4.glob("model_*.pt"), key=checkpoint_iteration)
    saved5 = sorted(stage5.glob("model_*.pt"), key=checkpoint_iteration)
    start = checkpoint_iteration(end4)
    # RSL-RL resumes from the saved label, so a budget of N ends at start+N-1.
    if checkpoint_iteration(saved5[-1]) != start + budget - 1:
        raise ValueError("Stage-5 final iteration does not match the fixed update budget")
    picks = [end4]
    for fraction in (0.25, 0.50):
        threshold = start + math.ceil(budget * fraction) - 1
        picks.append(next(path for path in saved5 if checkpoint_iteration(path) >= threshold))
    picks.append(saved5[-1])
    if len(set(picks)) != 4:
        raise ValueError("Expected four distinct predetermined candidates")
    return picks


def run_job(command: list[str], log: Path, *, retry: bool = False) -> None:
    attempts = 3 if retry else 1
    for attempt in range(1, attempts + 1):
        print(f"RUN ({attempt}/{attempts}): {' '.join(command)} -> {log}", flush=True)
        with log.open("a") as stream:
            result = subprocess.run(command, cwd=REPO, stdout=stream, stderr=subprocess.STDOUT)
        if result.returncode == 0:
            return
    raise subprocess.CalledProcessError(result.returncode, command)


def benchmark(checkpoint: Path, arm: str, condition: str, seed: int, out: Path) -> dict:
    command = [
        sys.executable, str(REPO / "scripts/benchmark_nn_drive.py"),
        "--task", TASKS[arm], "--checkpoint", str(checkpoint),
        "--num_envs", "64", "--num_steps", "1000", "--terrain", "generator",
        "--seed", str(seed), "--independent-scenario-seeds", "--json-output", str(out), "--headless",
    ]
    if condition == "nominal":
        command.extend(NOMINAL)
    run_job(command, out.with_suffix(".log"), retry=True)
    payload = json.loads(out.read_text())
    if set(payload["scenarios"]) != set(SCENARIOS):
        raise ValueError(f"Incomplete benchmark: {out}")
    if any(not math.isfinite(value) for row in payload["scenarios"].values() for value in row.values()):
        raise ValueError(f"Non-finite benchmark: {out}")
    return payload


def choose(evaluated: list[dict]) -> dict | None:
    passing = [row for row in evaluated if row["gate_passed"]]
    return min(passing, key=lambda row: (row["score"], row["iteration"])) if passing else None


def export(checkpoint: Path, arm: str, out: Path) -> dict:
    run_job([
        sys.executable, str(REPO / "scripts/rsl_rl/play.py"), "--task", TASKS[arm],
        "--checkpoint", str(checkpoint), "--num_envs", "1", "--num_steps", "1", "--headless",
    ], out / "export.log", retry=True)
    source = checkpoint.parent / "exported"
    for name in ("policy.pt", "policy.onnx"):
        shutil.copy2(source / name, out / name)
    run_job([
        sys.executable, str(REPO / "scripts/export_pure_nn_current_onnx.py"),
        "--policy", str(out / "policy.pt"), "--actor-onnx", str(out / "policy.onnx"),
        "--output", str(out / "policy_drive.onnx"), "--obs-dim", "13", "--cg-outputs", "0",
        "--i-max-a", "2.0", "--require-validation",
    ], out / "current_export.log")
    if arm == "C":
        run_job([
            sys.executable, str(REPO / "scripts/tools/expand_gru_for_stm32ai.py"),
            str(out / "policy_drive.onnx"), str(out / "policy_drive_stm32ai.onnx"), "--steps", "1024",
        ], out / "stm32ai_equivalence.log")
    else:
        shutil.copy2(out / "policy_drive.onnx", out / "policy_drive_stm32ai.onnx")
    return {name: sha256(out / name) for name in (
        "policy.pt", "policy.onnx", "policy_drive.onnx", "policy_drive_stm32ai.onnx"
    )}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--arm", choices=tuple(TASKS))
    parser.add_argument("--stage4", type=Path)
    parser.add_argument("--stage5", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    protocol = json.loads(args.protocol.read_text())
    check_protocol(protocol)
    if args.check_only:
        print("Pilot protocol verified: C/D, seed 43, 1250 updates, independent validation/test.")
        return
    if any(value is None for value in (args.arm, args.stage4, args.stage5, args.output)):
        parser.error("--arm, --stage4, --stage5 and --output are required without --check-only")
    args.output.mkdir(parents=True, exist_ok=False)
    picks = candidates(args.stage4, args.stage5, protocol["stage_update_budgets"][-1])
    evaluated = []
    for index, checkpoint in enumerate(picks):
        output = args.output / f"validation_{index}_{checkpoint.stem}.json"
        payload = benchmark(checkpoint, args.arm, "range", 1001, output)
        results = payload["scenarios"]
        failures = stage_gate_failures(5, results)
        evaluated.append({
            "checkpoint": str(checkpoint), "sha256": sha256(checkpoint),
            "iteration": checkpoint_iteration(checkpoint), "score": checkpoint_score(results),
            "gate_passed": not failures, "gate_failures": failures, "benchmark": str(output),
        })
        write_json(args.output / "validation_progress.json", {"candidates": evaluated})
    selected = choose(evaluated)
    manifest = {
        "arm": args.arm, "training_seed": 43, "protocol_sha256": sha256(args.protocol),
        "selection": "held-out validation among four predetermined candidates",
        "validation_base_seed": 1001, "test_base_seed": 2001,
        "candidates": evaluated, "selected": selected, "fallback_used": False,
        "fixed_budget_final": str(picks[-1]), "fixed_budget_final_sha256": sha256(picks[-1]),
        "hardware_validation_complete": False,
    }
    write_json(args.output / "deployment_selection.json", manifest)
    for condition in ("range", "nominal"):
        final_out = args.output / f"test_final_under_{condition}.json"
        benchmark(picks[-1], args.arm, condition, 2001, final_out)
        if selected is not None:
            selected_out = args.output / f"test_selected_under_{condition}.json"
            if Path(selected["checkpoint"]) == picks[-1]:
                shutil.copy2(final_out, selected_out)
            else:
                benchmark(Path(selected["checkpoint"]), args.arm, condition, 2001, selected_out)
    if selected is None:
        print(f"FAIL: arm {args.arm}: no validation candidate passed; final test retained, no export.", flush=True)
        raise SystemExit(3)
    manifest["exports"] = export(Path(selected["checkpoint"]), args.arm, args.output)
    manifest["status"] = "selected export validated; hardware test pending"
    write_json(args.output / "deployment_selection.json", manifest)
    print(f"OK: arm {args.arm}: selected {selected['checkpoint']} score={selected['score']:.6f}", flush=True)


if __name__ == "__main__":
    main()
