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

Multiple seeds: an unsuffixed file is the seed-42 run; additional seeds are
`<stem>_s<seed>.json` (e.g. arm_B_under_range_s43.json), produced by
scripts/tools/run_multiseed_arms.sh with the benchmark seed/terrain/scenarios
held fixed so they stay comparable. Every arm/condition is aggregated over
whatever seeds are present -- bars become the across-seed mean and error bars
the across-seed spread. With a single seed the spread is zero and the output is
identical to the original single-seed figures.

reward_curves.csv gains an optional `seed` column; rows without one are treated
as seed 42. Regenerate it with scripts/tools/extract_reward_curves.py.
"""

import argparse
import csv
import json
import re
from collections import defaultdict
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

_parser = argparse.ArgumentParser(description="Build the paper's comparison figures and results table.")
_parser.add_argument("--data-dir", type=Path, default=REPO / "docs" / "paper" / "figures" / "data")
_parser.add_argument(
    "--out-dir",
    type=Path,
    default=REPO / "docs" / "paper" / "figures",
    help="Where figures and tables are written. Point this at a scratch directory to preview "
    "a partial multi-seed set without overwriting the published figures.",
)
_args = _parser.parse_args()
DATA = _args.data_dir
OUT = _args.out_dir
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
# The seed column is optional: the original hand-made CSV predates it and holds
# seed 42 only.
by_curve = defaultdict(list)
by_curve_stage_min = {}
for r in rows:
    seed = r.get("seed") or "42"
    key = (r["arm"], seed, int(r["stage"]))
    val = int(r["iteration_in_stage"])
    by_curve_stage_min[key] = min(by_curve_stage_min.get(key, val), val)
for r in rows:
    seed = r.get("seed") or "42"
    arm, stage = r["arm"], int(r["stage"])
    since_resume = int(r["iteration_in_stage"]) - by_curve_stage_min[(arm, seed, stage)]
    x = stage_offset[stage - 1] + since_resume
    by_curve[(arm, seed)].append((x, float(r["reward"])))
for key in by_curve:
    by_curve[key].sort()

curve_seeds = sorted({seed for _, seed in by_curve})

fig, ax = plt.subplots(figsize=(6.8, 3.2))
for arm in ["A", "B", "C"]:
    seeds_here = [s for s in curve_seeds if (arm, s) in by_curve]
    by_seed = {seed: dict(by_curve[(arm, seed)]) for seed in seeds_here}
    # Only x positions every seed reached, so the ribbon is always the spread of
    # the same number of runs and cannot narrow just because a seed ran short.
    shared_x = sorted(set.intersection(*(set(v) for v in by_seed.values())))
    stacked = np.array([[by_seed[seed][x] for x in shared_x] for seed in seeds_here])
    mean = stacked.mean(axis=0)
    if len(seeds_here) > 1:
        sd = stacked.std(axis=0, ddof=1)
        ax.fill_between(
            shared_x, mean - sd, mean + sd, color=ARM_COLOR[arm], alpha=0.18, linewidth=0, zorder=1
        )
    ax.plot(shared_x, mean, color=ARM_COLOR[arm], linewidth=1.5, label=ARM_LABEL[arm], zorder=2)
for b in stage_boundaries:
    ax.axvline(b, color="#c3c2b7", linewidth=0.8, linestyle=(0, (3, 2)), zorder=0)
ax.set_xlabel("Cumulative training iteration (stage boundaries dashed)")
ax.set_ylabel("Mean episode reward")
ax.legend(frameon=False, loc="lower right")
seed_label = (
    f"seed {curve_seeds[0]}"
    if len(curve_seeds) == 1
    else f"mean +/- s.d. over {len(curve_seeds)} seeds"
)
ax.set_title(f"Training reward across the 5-stage curriculum ({seed_label})")
fig.tight_layout()
fig.savefig(OUT / "fig1_training_curves.png")
fig.savefig(OUT / "fig1_training_curves.pdf")
plt.close(fig)

# ---------------------------------------------------------------------------
# Load benchmark JSON
# ---------------------------------------------------------------------------
RUN_FILES = {
    "A_range": "arm_A_under_range",
    "A_nominal": "arm_A_under_nominal",
    "B_range": "arm_B_under_range",
    "C_range": "arm_C_under_range",
}


def load_seeds(stem):
    """Return {seed: payload} for a run: the unsuffixed file plus every _s<seed>."""
    found = {}
    base = DATA / f"{stem}.json"
    if base.is_file():
        found["42"] = json.load(open(base))
    for path in sorted(DATA.glob(f"{stem}_s*.json")):
        match = re.fullmatch(rf"{re.escape(stem)}_s(\d+)", path.stem)
        if match:
            found[match.group(1)] = json.load(open(path))
    if not found:
        raise FileNotFoundError(f"No benchmark JSON for {stem} in {DATA}")
    return found


bench = {key: load_seeds(stem) for key, stem in RUN_FILES.items()}

# Every arm must be averaged over the same seeds, or a "mean across seeds" bar
# silently compares an arm's good seed against another arm's full set.
seed_sets = {key: frozenset(runs) for key, runs in bench.items()}
if len(set(seed_sets.values())) != 1:
    detail = "\n".join(f"  {k}: {sorted(v)}" for k, v in seed_sets.items())
    raise SystemExit(f"Arms do not share the same seed set; aggregating them would compare unlike sets:\n{detail}")
SEEDS = sorted(next(iter(seed_sets.values())))
print(f"Aggregating over {len(SEEDS)} seed(s): {', '.join(SEEDS)}")

scenarios = list(bench["A_range"][SEEDS[0]]["scenarios"].keys())
metrics = [
    ("fall_rate", "Fall rate", ""),
    ("rms_vel_err_mps", "RMS velocity error", "m/s"),
    ("world_drift_m", "World drift", "m"),
    ("rms_pitch_deg", "RMS pitch", "deg"),
]


def per_seed_scenario_mean(run_key, metric):
    """One value per seed: that seed's mean over the 11 scenarios."""
    return [np.mean([bench[run_key][seed]["scenarios"][s][metric] for s in scenarios]) for seed in SEEDS]


def scenario_mean(run_key, metric):
    return float(np.mean(per_seed_scenario_mean(run_key, metric)))


def scenario_spread(run_key, metric):
    """Across-seed sample std; 0.0 for a single seed (nothing to spread over)."""
    values = per_seed_scenario_mean(run_key, metric)
    return float(np.std(values, ddof=1)) if len(values) > 1 else 0.0


def per_scenario_mean(run_key, metric, scenario):
    return float(np.mean([bench[run_key][seed]["scenarios"][scenario][metric] for seed in SEEDS]))


def per_scenario_spread(run_key, metric, scenario):
    values = [bench[run_key][seed]["scenarios"][scenario][metric] for seed in SEEDS]
    return float(np.std(values, ddof=1)) if len(values) > 1 else 0.0


SEED_NOTE = f"seed {SEEDS[0]}" if len(SEEDS) == 1 else f"mean +/- s.d. over {len(SEEDS)} seeds"
SURFACE = "#ffffff"


def spread_marks(ax, positions, means, spreads, per_seed, colors, width=0.34):
    """Draw each category as a translucent mean +/- s.d. band, a mean rule, and
    one dot per seed.

    Replaces bar + error bar. A bar encodes magnitude by length from zero, which
    for these metrics is not the quantity being compared -- the comparison is
    between the arms' levels and, just as importantly, their spread. The band
    gives spread its own area, and the per-seed dots keep n visible so three
    runs are never mistaken for a smooth distribution.
    """
    for x, mean, sd, seeds, color in zip(positions, means, spreads, per_seed, colors):
        if sd > 0:
            ax.add_patch(
                plt.Rectangle(
                    (x - width / 2, mean - sd),
                    width,
                    2 * sd,
                    facecolor=color,
                    edgecolor="none",
                    alpha=0.22,
                    zorder=1,
                )
            )
        ax.hlines(mean, x - width / 2, x + width / 2, color=color, linewidth=2.0, zorder=3)
        # 2px surface ring so overlapping seed dots stay countable.
        ax.scatter(
            [x] * len(seeds),
            seeds,
            s=26,
            facecolor=color,
            edgecolor=SURFACE,
            linewidth=1.0,
            zorder=4,
            clip_on=False,
        )


def spread_limits(means, spreads, per_seed, pad=0.18):
    lo = min(min(seeds) for seeds in per_seed)
    lo = min(lo, min(m - s for m, s in zip(means, spreads)))
    hi = max(max(seeds) for seeds in per_seed)
    hi = max(hi, max(m + s for m, s in zip(means, spreads)))
    span = hi - lo or max(hi, 1.0)
    return max(0.0, lo - pad * span), hi + pad * span * 2.0


# ---------------------------------------------------------------------------
# Figure 2: aggregate comparison across A/B/C under range DR (main result)
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(1, 4, figsize=(8.0, 2.7))
arms = ["A_range", "B_range", "C_range"]
colors = [COLOR_A, COLOR_B, COLOR_C]
short_labels = ["A", "B", "C"]
positions = np.arange(len(arms))
for ax, (metric, title, unit) in zip(axes, metrics):
    vals = [scenario_mean(a, metric) for a in arms]
    errs = [scenario_spread(a, metric) for a in arms]
    seeds_by_arm = [per_seed_scenario_mean(a, metric) for a in arms]
    spread_marks(ax, positions, vals, errs, seeds_by_arm, colors)
    ax.set_title(title, fontsize=8.5)
    ax.set_ylabel(unit if unit else "fraction", fontsize=8)
    ax.set_xticks(positions)
    ax.set_xticklabels(short_labels)
    ax.tick_params(axis="x", labelsize=9)
    ax.set_xlim(-0.55, len(arms) - 0.45)
    low, high = spread_limits(vals, errs, seeds_by_arm)
    ax.set_ylim(low, high)
    for x, v, e in zip(positions, vals, errs):
        ax.text(
            x,
            max(v + e, max(seeds_by_arm[int(x)])) + 0.05 * (high - low),
            f"{v:.3f}",
            ha="center",
            va="bottom",
            fontsize=7,
            color="#52514e",
        )
fig.suptitle(
    "Mean over 11 benchmark scenarios, all three arms under measured-range actuator DR "
    f"({SEED_NOTE})",
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
    vals = [per_scenario_mean(arm, "rms_vel_err_mps", s) for s in scenarios]
    errs = [per_scenario_spread(arm, "rms_vel_err_mps", s) for s in scenarios]
    per_seed_vals = [
        [bench[arm][seed]["scenarios"][s]["rms_vel_err_mps"] for seed in SEEDS] for s in scenarios
    ]
    offsets = x + (i - 1) * width
    spread_marks(ax, offsets, vals, errs, per_seed_vals, [color] * len(scenarios), width=width * 0.72)
    # spread_marks draws no legend handle; add a proxy so identity is never color-alone.
    ax.plot([], [], color=color, linewidth=2.0, label=label)
ax.set_ylim(bottom=0)
ax.set_xticks(x)
ax.set_xticklabels([s.replace("_", "\n") for s in scenarios], fontsize=6.8)
ax.set_ylabel("RMS velocity error (m/s)")
ax.set_title(f"Velocity-tracking error by scenario, all three arms under measured-range DR ({SEED_NOTE})")
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
# Short ticks on purpose: at this panel width the parenthetical "(nominal DR)"
# / "(range DR)" forms collide into each other. The suptitle names both
# distributions, so the ticks only have to distinguish them.
cond_labels = ["Home turf", "Stress test"]
cond_positions = np.arange(len(conds))
for ax, (metric, title, unit) in zip(axes, metrics):
    vals = [scenario_mean(c, metric) for c in conds]
    errs = [scenario_spread(c, metric) for c in conds]
    seeds_by_cond = [per_seed_scenario_mean(c, metric) for c in conds]
    spread_marks(ax, cond_positions, vals, errs, seeds_by_cond, cond_colors, width=0.30)
    ax.set_title(title, fontsize=8.5)
    ax.set_ylabel(unit if unit else "fraction", fontsize=8)
    ax.set_xticks(cond_positions)
    ax.set_xticklabels(cond_labels)
    ax.tick_params(axis="x", labelsize=7.5)
    ax.set_xlim(-0.55, len(conds) - 0.45)
    low, high = spread_limits(vals, errs, seeds_by_cond)
    ax.set_ylim(low, high)
    for x, v, e in zip(cond_positions, vals, errs):
        ax.text(
            x,
            max(v + e, max(seeds_by_cond[int(x)])) + 0.05 * (high - low),
            f"{v:.3f}",
            ha="center",
            va="bottom",
            fontsize=7,
            color="#52514e",
        )
fig.suptitle(
    "Arm A (baseline) evaluated on its own training distribution vs. the measured-range DR it never saw "
    f"({SEED_NOTE})",
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

multi_seed = len(SEEDS) > 1

# CSV keeps mean and s.d. in separate columns so it stays machine-readable;
# the Markdown table renders them as "mean +/- s.d." for dropping into the paper.
with open(OUT / "results_table.csv", "w", newline="") as f:
    writer = csv.writer(f)
    header = ["run", "seeds"]
    for _, label in table_metrics:
        header.append(label)
        if multi_seed:
            header.append(f"{label} (s.d.)")
    writer.writerow(header)
    for run in run_order:
        row = [run_labels[run], len(SEEDS)]
        for m, _ in table_metrics:
            row.append(f"{scenario_mean(run, m):.4f}")
            if multi_seed:
                row.append(f"{scenario_spread(run, m):.4f}")
        writer.writerow(row)

with open(OUT / "results_table.md", "w") as f:
    f.write(f"Aggregated over {len(SEEDS)} seed(s): {', '.join(SEEDS)}\n\n")
    f.write("| Run | " + " | ".join(label for _, label in table_metrics) + " |\n")
    f.write("|---" * (len(table_metrics) + 1) + "|\n")
    for run in run_order:
        vals = []
        for m, _ in table_metrics:
            mean = scenario_mean(run, m)
            vals.append(f"{mean:.4f} ± {scenario_spread(run, m):.4f}" if multi_seed else f"{mean:.4f}")
        f.write(f"| {run_labels[run]} | " + " | ".join(vals) + " |\n")

print("Wrote figures to", OUT)
for p in sorted(OUT.iterdir()):
    print(" ", p.name)
