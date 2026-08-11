#!/usr/bin/env python3
"""Generate the point/range/range+GRU comparison figures and results table
for the paper from docs/paper/figures/data/*.

Source data (docs/paper/figures/data/):
  arm_A_under_range.json / arm_A_under_nominal.json / arm_B_under_range.json /
  arm_C_under_range.json  -- scripts/benchmark_nn_drive.py output, 11 scenarios
  each, seed 42, terrain=generator, num_envs=64, num_steps=1000, against the
  stage-5 checkpoint selected_checkpoint.json actually selected for each arm
  (see ExportedPolicy/fixedstance_{point,range,range_gru}_2026-08-10/).
  reward_curves.csv -- Train/mean_reward extracted from each arm's 5 stage
  run directories' TensorBoard event files (see scripts/archive_run_for_paper.py
  for how those runs are archived).

Regenerating after more seeds: re-run the benchmarks (same seed/terrain/
scenarios for comparability) into new JSON files following this naming
pattern, re-extract reward_curves.csv the same way, then either re-point this
script's data or extend it to aggregate across seeds (mean +/- std per arm) --
it currently assumes exactly one run per arm/condition.
"""

import csv
import json
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.rcParams.update(
    {
        "font.size": 9,
        "font.family": "sans-serif",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.edgecolor": "#8a8a86",
        "axes.labelcolor": "#0b0b0b",
        "xtick.color": "#52514e",
        "ytick.color": "#52514e",
        "text.color": "#0b0b0b",
        "axes.grid": True,
        "grid.color": "#e5e4e0",
        "grid.linewidth": 0.6,
        "axes.axisbelow": True,
        "savefig.dpi": 300,
        "figure.dpi": 150,
    }
)

REPO = Path(__file__).resolve().parents[1]
DATA = REPO / "docs" / "paper" / "figures" / "data"
OUT = REPO / "docs" / "paper" / "figures"
OUT.mkdir(parents=True, exist_ok=True)

# Validated categorical palette (light mode), fixed order, one hue per entity.
COLOR_A = "#2a78d6"  # blue
COLOR_B = "#eb6834"  # orange
COLOR_C = "#1baf7a"  # aqua
COLOR_A_NOMINAL = "#8fb8e8"  # lighter tint of A's blue, same hue family

ARM_LABEL = {"A": "Baseline (point)", "B": "Proposed 1 (range, MLP)", "C": "Proposed 2 (range, GRU)"}
ARM_COLOR = {"A": COLOR_A, "B": COLOR_B, "C": COLOR_C}

# ---------------------------------------------------------------------------
# Figure 1: training reward curves
#
# NOTE: raw cumulative_iteration (RSL-RL's own counter) does NOT reset to 0 on
# a stage resume -- it continues from whichever checkpoint iteration that
# stage's shortlist actually selected (e.g. stage 1 selecting model_200 of a
# 300-iteration budget means stage 2 resumes counting from 200, not 300), and
# that selected iteration differs per arm. Plotting raw values would misalign
# every stage transition both within and across arms. Re-anchor each stage to
# "iterations completed since it resumed," then lay stages out end-to-end
# using their fixed iteration BUDGETS (which are identical across arms) so
# all three curves' stage boundaries line up on one shared axis.
# ---------------------------------------------------------------------------
STAGE_ITERS = [300, 350, 200, 250, 300]
stage_boundaries = np.cumsum(STAGE_ITERS)[:-1]
stage_offset = [0, *np.cumsum(STAGE_ITERS)[:-1]]

rows = list(csv.DictReader(open(DATA / "reward_curves.csv")))
by_arm = {"A": [], "B": [], "C": []}
by_arm_stage_min = {}
for r in rows:
    key = (r["arm"], int(r["stage"]))
    val = int(r["iteration_in_stage"])
    by_arm_stage_min[key] = min(by_arm_stage_min.get(key, val), val)
for r in rows:
    arm, stage = r["arm"], int(r["stage"])
    since_resume = int(r["iteration_in_stage"]) - by_arm_stage_min[(arm, stage)]
    x = stage_offset[stage - 1] + since_resume
    by_arm[arm].append((x, float(r["reward"])))
for arm in by_arm:
    by_arm[arm].sort()

fig, ax = plt.subplots(figsize=(6.8, 3.2))
for arm in ["A", "B", "C"]:
    xs, ys = zip(*by_arm[arm])
    ax.plot(xs, ys, color=ARM_COLOR[arm], linewidth=1.6, label=ARM_LABEL[arm], alpha=0.9)
for b in stage_boundaries:
    ax.axvline(b, color="#c3c2b7", linewidth=0.8, linestyle=(0, (3, 2)), zorder=0)
ax.set_xlabel("Cumulative training iteration (stage boundaries dashed)")
ax.set_ylabel("Mean episode reward")
ax.legend(frameon=False, loc="lower right")
ax.set_title("Training reward across the 5-stage curriculum (seed 42)")
fig.tight_layout()
fig.savefig(OUT / "fig1_training_curves.png")
fig.savefig(OUT / "fig1_training_curves.pdf")
plt.close(fig)

# ---------------------------------------------------------------------------
# Load benchmark JSON
# ---------------------------------------------------------------------------
bench = {
    "A_range": json.load(open(DATA / "arm_A_under_range.json")),
    "A_nominal": json.load(open(DATA / "arm_A_under_nominal.json")),
    "B_range": json.load(open(DATA / "arm_B_under_range.json")),
    "C_range": json.load(open(DATA / "arm_C_under_range.json")),
}
scenarios = list(bench["A_range"]["scenarios"].keys())
metrics = [
    ("fall_rate", "Fall rate", ""),
    ("rms_vel_err_mps", "RMS velocity error", "m/s"),
    ("world_drift_m", "World drift", "m"),
    ("rms_pitch_deg", "RMS pitch", "deg"),
]


def scenario_mean(run_key, metric):
    return np.mean([bench[run_key]["scenarios"][s][metric] for s in scenarios])


# ---------------------------------------------------------------------------
# Figure 2: aggregate comparison across A/B/C under range DR (main result)
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(1, 4, figsize=(8.0, 2.7))
arms = ["A_range", "B_range", "C_range"]
colors = [COLOR_A, COLOR_B, COLOR_C]
short_labels = ["A", "B", "C"]
for ax, (metric, title, unit) in zip(axes, metrics):
    vals = [scenario_mean(a, metric) for a in arms]
    bars = ax.bar(short_labels, vals, color=colors, width=0.62)
    ax.set_title(title, fontsize=8.5)
    ax.set_ylabel(unit if unit else "fraction", fontsize=8)
    ax.tick_params(axis="x", labelsize=9)
    for b, v in zip(bars, vals):
        ax.text(
            b.get_x() + b.get_width() / 2,
            v + 0.02 * max(vals),
            f"{v:.3f}",
            ha="center",
            va="bottom",
            fontsize=7,
            color="#52514e",
        )
    ax.set_ylim(0, max(vals) * 1.28)
fig.suptitle(
    "Mean over 11 benchmark scenarios, all three arms under measured-range actuator DR (seed 42)",
    fontsize=8.5,
    y=1.06,
)
handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in colors]
fig.legend(
    handles,
    ["A: baseline (nominal DR, MLP)", "B: proposed 1 (range DR, MLP)", "C: proposed 2 (range DR, GRU)"],
    loc="lower center",
    bbox_to_anchor=(0.5, -0.14),
    ncol=3,
    frameon=False,
    fontsize=8,
)
fig.tight_layout()
fig.savefig(OUT / "fig2_aggregate_comparison.png", bbox_inches="tight")
fig.savefig(OUT / "fig2_aggregate_comparison.pdf", bbox_inches="tight")
plt.close(fig)

# ---------------------------------------------------------------------------
# Figure 3: per-scenario velocity tracking error (standout metric)
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(8.2, 3.4))
x = np.arange(len(scenarios))
width = 0.26
for i, (arm, color, label) in enumerate(zip(arms, colors, ["A: baseline", "B: range+MLP", "C: range+GRU"])):
    vals = [bench[arm]["scenarios"][s]["rms_vel_err_mps"] for s in scenarios]
    ax.bar(x + (i - 1) * width, vals, width=width * 0.92, color=color, label=label)
ax.set_xticks(x)
ax.set_xticklabels([s.replace("_", "\n") for s in scenarios], fontsize=6.8)
ax.set_ylabel("RMS velocity error (m/s)")
ax.set_title("Velocity-tracking error by scenario, all three arms under measured-range DR")
ax.legend(frameon=False, ncol=3, loc="upper center", bbox_to_anchor=(0.5, -0.18))
fig.tight_layout()
fig.savefig(OUT / "fig3_scenario_velocity_tracking.png", bbox_inches="tight")
fig.savefig(OUT / "fig3_scenario_velocity_tracking.pdf", bbox_inches="tight")
plt.close(fig)

# ---------------------------------------------------------------------------
# Figure 4: baseline degradation check (A under nominal vs range DR)
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(1, 4, figsize=(8.0, 2.6))
conds = ["A_nominal", "A_range"]
cond_colors = [COLOR_A_NOMINAL, COLOR_A]
cond_labels = ["Home turf\n(nominal DR)", "Stress test\n(range DR)"]
for ax, (metric, title, unit) in zip(axes, metrics):
    vals = [scenario_mean(c, metric) for c in conds]
    bars = ax.bar(cond_labels, vals, color=cond_colors, width=0.55)
    ax.set_title(title, fontsize=8.5)
    ax.set_ylabel(unit if unit else "fraction", fontsize=8)
    ax.tick_params(axis="x", labelsize=7.5)
    for b, v in zip(bars, vals):
        ax.text(
            b.get_x() + b.get_width() / 2,
            v + 0.02 * max(vals),
            f"{v:.3f}",
            ha="center",
            va="bottom",
            fontsize=7,
            color="#52514e",
        )
    ax.set_ylim(0, max(vals) * 1.28)
fig.suptitle(
    "Arm A (baseline) evaluated on its own training distribution vs. the measured-range DR it never saw",
    fontsize=8.2,
    y=1.03,
)
fig.tight_layout()
fig.savefig(OUT / "fig4_baseline_degradation.png", bbox_inches="tight")
fig.savefig(OUT / "fig4_baseline_degradation.pdf", bbox_inches="tight")
plt.close(fig)

# ---------------------------------------------------------------------------
# Results table (CSV + Markdown)
# ---------------------------------------------------------------------------
table_metrics = [
    ("fall_rate", "Fall rate"),
    ("rms_vel_err_mps", "RMS vel. error (m/s)"),
    ("rms_yaw_rate_err_radps", "RMS yaw-rate error (rad/s)"),
    ("world_drift_m", "World drift (m)"),
    ("rms_pitch_deg", "RMS pitch (deg)"),
]
run_order = ["A_range", "A_nominal", "B_range", "C_range"]
run_labels = {
    "A_range": "A: baseline, stress-tested (range DR)",
    "A_nominal": "A: baseline, home turf (nominal DR)",
    "B_range": "B: proposed 1, range DR + MLP",
    "C_range": "C: proposed 2, range DR + GRU",
}

with open(OUT / "results_table.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["run"] + [label for _, label in table_metrics])
    for run in run_order:
        writer.writerow([run_labels[run]] + [f"{scenario_mean(run, m):.4f}" for m, _ in table_metrics])

with open(OUT / "results_table.md", "w") as f:
    f.write("| Run | " + " | ".join(label for _, label in table_metrics) + " |\n")
    f.write("|---" * (len(table_metrics) + 1) + "|\n")
    for run in run_order:
        vals = [f"{scenario_mean(run, m):.4f}" for m, _ in table_metrics]
        f.write(f"| {run_labels[run]} | " + " | ".join(vals) + " |\n")

print("Wrote figures to", OUT)
for p in sorted(OUT.iterdir()):
    print(" ", p.name)
