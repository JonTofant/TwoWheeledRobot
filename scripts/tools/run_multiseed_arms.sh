#!/usr/bin/env bash
# Train the three paper arms (point / range / range+GRU) at additional seeds and
# benchmark each resulting stage-5 policy under the same conditions as seed 42.
#
# Runs INSIDE the isaac-lab-dev container. Every training and benchmark knob is
# pinned to what the 2026-08-10 seed-42 arms used, verified against their
# archived params/*.yaml -- the seed is the only intended difference:
#
#   task/experiment  NNDriveFixedStance(GRU)-v0 under nn_drive_fixed_stance
#   num_envs         4096
#   iterations       300 350 200 250 300
#   selection        3 reward-ranked candidates + newest, benchmark seed 42,
#                    64 envs x 1000 steps, --allow-gate-failure
#   arm A (point)    the six actuator ranges collapsed to their point estimates
#   arm B (range)    repo-default ranges (no overrides)
#   arm C (range)    same as B, GRU task
#
# Arms run sequentially: 4096 envs is ~8 GB on a 16 GB card, so two curricula
# cannot share the GPU. Order is seed-major, so the earlier seed is a complete
# three-arm comparison before the later seed starts.
#
# Usage (from the host):
#   docker exec -d isaac-lab-dev bash -c \
#     'cd /workspace/TwoWheeledRobot && bash scripts/tools/run_multiseed_arms.sh 43 44'

set -u

SEEDS=("$@")
if [ ${#SEEDS[@]} -eq 0 ]; then
    echo "usage: run_multiseed_arms.sh SEED [SEED ...]" >&2
    exit 2
fi

REPO=/workspace/TwoWheeledRobot
PY=/isaac-sim/python.sh
RUN_ROOT=$REPO/logs/rsl_rl/nn_drive_fixed_stance
LOG_DIR=$REPO/logs/multiseed_$(date +%Y-%m-%d)
DATA_DIR=$REPO/docs/paper/figures/data
STATUS=$LOG_DIR/STATUS.txt
mkdir -p "$LOG_DIR" "$DATA_DIR"

# Arm A's point estimates. These are the seed-42 point arm's resolved values
# (ExportedPolicy/fixedstance_point_2026-08-10/*/params/env.yaml), i.e. the
# midpoint/nominal of each corresponding range in the range arm.
POINT_OVERRIDES=(
    "env.motor_gain_range=[1.0,1.0]"
    "env.motor_deadzone_a_range=[0.0534,0.0534]"
    "env.motor_tau_s_range=[0.0075,0.0075]"
    "env.motor_current_limit_a_range=[2.0,2.0]"
    "env.cg_kp_range=[30.0,30.0]"
    "env.cg_kd_range=[3.0,3.0]"
)

MLP_TASK=Template-Twowheeledrobot-NNDriveFixedStance-v0
GRU_TASK=Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0

say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$STATUS"; }

# Fail loudly if an arm trained against a distribution it was not meant to.
# PAPER-02: a configured-but-unapplied range silently turns the range arm into a
# second point arm, and it is the resolved env.yaml -- not the command line --
# that proves which one actually ran.
verify_ranges() {
    local run_dir=$1 expect=$2
    $PY - "$run_dir/params/env.yaml" "$expect" <<'PYEOF'
import sys, yaml
cfg = yaml.unsafe_load(open(sys.argv[1]))
expect = sys.argv[2]
want = {
    "point": {
        "motor_gain_range": (1.0, 1.0),
        "motor_deadzone_a_range": (0.0534, 0.0534),
        "motor_tau_s_range": (0.0075, 0.0075),
        "motor_current_limit_a_range": (2.0, 2.0),
        "cg_kp_range": (30.0, 30.0),
        "cg_kd_range": (3.0, 3.0),
    },
    "range": {
        "motor_gain_range": (0.97, 1.03),
        "motor_deadzone_a_range": (0.031, 0.078),
        "motor_tau_s_range": (0.005, 0.010),
        "motor_current_limit_a_range": (1.9, 2.1),
        "cg_kp_range": (28.5, 31.5),
        "cg_kd_range": (2.85, 3.15),
    },
}[expect]
bad = []
for key, value in want.items():
    got = tuple(cfg[key])
    if got != value:
        bad.append(f"{key}: got {got}, expected {value}")
if bad:
    print("RANGE MISMATCH (" + expect + "):\n  " + "\n  ".join(bad))
    sys.exit(1)
print(f"ranges verified as '{expect}'; seed={cfg['seed']}")
PYEOF
}

latest_run() { ls -d "$RUN_ROOT"/*_"$1"_stage"$2" 2>/dev/null | sort | tail -1; }

# train_arm publishes the selected stage-5 run directory here rather than on
# stdout: it also calls say(), and a $(...) capture would swallow the progress
# lines and mask the real exit code behind a pipeline.
STAGE5_DIR=""

# $1 arm letter, $2 arm slug, $3 seed, $4 task, $5 point|range, rest: overrides
train_arm() {
    local letter=$1 slug=$2 seed=$3 task=$4 kind=$5
    shift 5
    local prefix="${slug}_s${seed}"
    local log="$LOG_DIR/${prefix}_train.log"
    STAGE5_DIR=""

    say "START train arm $letter ($slug) seed $seed -> $log"
    $PY "$REPO/scripts/train_nn_drive_curriculum.py" \
        --task "$task" \
        --experiment-name nn_drive_fixed_stance \
        --run-name-prefix "$prefix" \
        --num_envs 4096 \
        --iterations 300 350 200 250 300 \
        --selection-candidates 3 \
        --benchmark-num-envs 64 \
        --benchmark-num-steps 1000 \
        --benchmark-seed 42 \
        --seed "$seed" \
        --allow-gate-failure \
        --headless \
        "$@" >"$log" 2>&1
    local rc=$?
    if [ $rc -ne 0 ]; then
        say "FAIL train arm $letter seed $seed (exit $rc) -- see $log"
        return $rc
    fi

    local stage1 stage5
    stage1=$(latest_run "$prefix" 1)
    stage5=$(latest_run "$prefix" 5)
    if [ -z "$stage5" ]; then
        say "FAIL arm $letter seed $seed: no stage-5 run directory found"
        return 1
    fi
    say "OK train arm $letter seed $seed -> $(basename "$stage5")"
    local verdict
    if verdict=$(verify_ranges "$stage1" "$kind" 2>&1); then
        say "  $(echo "$verdict" | tail -1)"
    else
        say "FAIL arm $letter seed $seed trained the wrong distribution:"
        say "  $(echo "$verdict" | tail -8)"
        return 1
    fi
    STAGE5_DIR="$stage5"
}

# $1 arm letter, $2 seed, $3 stage-5 run dir, $4 task, $5 condition, rest: overrides
bench_arm() {
    local letter=$1 seed=$2 stage5=$3 task=$4 condition=$5
    shift 5
    local ckpt out log
    ckpt=$($PY -c "import json,sys;print(json.load(open(sys.argv[1]))['selected_checkpoint'])" \
        "$stage5/selected_checkpoint.json" 2>/dev/null)
    if [ -z "$ckpt" ]; then
        say "FAIL bench arm $letter seed $seed: no selected_checkpoint.json in $stage5"
        return 1
    fi
    out="$DATA_DIR/arm_${letter}_under_${condition}_s${seed}.json"
    log="$LOG_DIR/arm_${letter}_s${seed}_under_${condition}_bench.log"

    say "START bench arm $letter seed $seed under $condition ($ckpt)"
    $PY "$REPO/scripts/benchmark_nn_drive.py" \
        --task "$task" \
        --checkpoint "$stage5/$ckpt" \
        --num_envs 64 \
        --num_steps 1000 \
        --terrain generator \
        --seed 42 \
        --json-output "$out" \
        --headless \
        "$@" >"$log" 2>&1
    local rc=$?
    if [ $rc -ne 0 ]; then
        say "FAIL bench arm $letter seed $seed under $condition (exit $rc) -- see $log"
        return $rc
    fi
    say "OK bench arm $letter seed $seed under $condition -> $(basename "$out")"
}

say "=== multi-seed run started for seeds: ${SEEDS[*]} ==="
say "repo commit: $(cd "$REPO" && git rev-parse --short HEAD)"

for seed in "${SEEDS[@]}"; do
    say "--- seed $seed ---"

    # Arm A: point estimates, evaluated under BOTH its own nominal world and the
    # range world (the baseline-degradation figure needs the mismatch case).
    if train_arm A point "$seed" "$MLP_TASK" point "${POINT_OVERRIDES[@]}"; then
        bench_arm A "$seed" "$STAGE5_DIR" "$MLP_TASK" nominal "${POINT_OVERRIDES[@]}"
        bench_arm A "$seed" "$STAGE5_DIR" "$MLP_TASK" range
    fi

    # Arm B: repo-default ranges, MLP.
    if train_arm B range "$seed" "$MLP_TASK" range; then
        bench_arm B "$seed" "$STAGE5_DIR" "$MLP_TASK" range
    fi

    # Arm C: repo-default ranges, GRU.
    if train_arm C range_gru "$seed" "$GRU_TASK" range; then
        bench_arm C "$seed" "$STAGE5_DIR" "$GRU_TASK" range
    fi

    say "--- seed $seed complete ---"
done

say "=== multi-seed run finished ==="
