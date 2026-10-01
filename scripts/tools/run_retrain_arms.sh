#!/usr/bin/env bash
# Retrain the paper arms after the 2026-09-30 review audit, and benchmark them.
#
# Runs INSIDE the isaac-lab-dev container. Differences from run_multiseed_arms.sh
# (the 2026-08 arms) are the fixes the audit found, plus one extra arm:
#
#   obs[9:10]        policy's own clamped command (firmware semantics), not the
#                    motor model's current          prev_current_obs_source=command
#   current lag      integrated every 1 ms substep; tau 5-10 ms now has an effect
#   left wheel       torque-speed envelope uses the logical wheel velocity
#   tire friction    effective static 0.5-1.1 / dynamic 0.4-0.9 per env per
#                    episode (was one draw per run, halved by the 0.5 default
#                    robot material)              ground_friction_randomization_mode=per_episode
#   noise floor      action std floor 0.05 (was 0.15)
#   selection        fixed budget, last checkpoint of each stage (was a gated
#                    shortlist benchmarked on the same seed the paper reports)
#   arm D            [145,145] MLP, capacity-matched to the GRU actor
#   benchmark        256 envs, GRU state reset per scenario, separate output dir
#
# 4096 envs is ~8 GB on a 16 GB card, so arms run sequentially, seed-major.
#
# Usage (from the host):
#   docker exec -d isaac-lab-dev bash -c \
#     'cd /workspace/TwoWheeledRobot && bash scripts/tools/run_retrain_arms.sh 42 43 44'

set -u

SEEDS=("$@")
if [ ${#SEEDS[@]} -eq 0 ]; then
    echo "usage: run_retrain_arms.sh SEED [SEED ...]" >&2
    exit 2
fi

REPO=/workspace/TwoWheeledRobot
PY=/isaac-sim/python.sh
TAG=${RETRAIN_TAG:-r2}
RUN_ROOT=$REPO/logs/rsl_rl/nn_drive_fixed_stance
LOG_DIR=$REPO/logs/retrain_${TAG}_$(date +%Y-%m-%d)
DATA_DIR=${RETRAIN_DATA_DIR:-$REPO/docs/paper/figures/data_${TAG}}
STATUS=$LOG_DIR/STATUS.txt
ITERATIONS=${RETRAIN_ITERATIONS:-"300 350 200 250 300"}
NUM_ENVS=${RETRAIN_NUM_ENVS:-4096}
BENCH_ENVS=${RETRAIN_BENCH_ENVS:-256}
BENCH_STEPS=${RETRAIN_BENCH_STEPS:-1000}
ARMS=${RETRAIN_ARMS:-"A B C D"}
mkdir -p "$LOG_DIR" "$DATA_DIR"

# Arm A collapses the actuator ranges (wheel motors and CyberGear gains) to
# their point estimates. Everything else -- friction, delays, sensors, body --
# is randomized identically in every arm.
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
WIDE_TASK=Template-Twowheeledrobot-NNDriveFixedStanceWide-v0

say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$STATUS"; }

# PAPER-02: the resolved params/*.yaml -- not the command line -- proves what
# ran. Check the actuator ranges for the arm kind AND every audit fix.
verify_run() {
    local run_dir=$1 expect=$2
    $PY - "$run_dir/params/env.yaml" "$run_dir/params/agent.yaml" "$expect" <<'PYEOF'
import sys, yaml
cfg = yaml.unsafe_load(open(sys.argv[1]))
agent = yaml.unsafe_load(open(sys.argv[2]))
expect = sys.argv[3]
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
want.update({
    "prev_current_obs_source": "command",
    "ground_friction_randomization_mode": "per_episode",
    "ground_static_friction_range": (0.5, 1.1),
    "ground_dynamic_friction_range": (0.4, 0.9),
    "obs_delay_steps_range": (0, 1),
    "action_delay_steps_range": (0, 1),
})
bad = []
for key, value in want.items():
    got = cfg[key]
    got = tuple(got) if isinstance(got, (list, tuple)) else got
    if got != value:
        bad.append(f"{key}: got {got}, expected {value}")
floor = tuple(agent.get("action_std_floor") or ())
if floor != (0.05, 0.05):
    bad.append(f"action_std_floor: got {floor}, expected (0.05, 0.05)")
if bad:
    print("CONFIG MISMATCH (" + expect + "):\n  " + "\n  ".join(bad))
    sys.exit(1)
print(f"config verified as '{expect}' + audit fixes; seed={cfg['seed']}")
PYEOF
}

latest_run() { ls -d "$RUN_ROOT"/*_"$1"_stage"$2" 2>/dev/null | sort | tail -1; }

STAGE5_DIR=""

# $1 arm letter, $2 arm slug, $3 seed, $4 task, $5 point|range, rest: overrides
train_arm() {
    local letter=$1 slug=$2 seed=$3 task=$4 kind=$5
    shift 5
    local prefix="${TAG}_${slug}_s${seed}"
    local log="$LOG_DIR/${prefix}_train.log"
    STAGE5_DIR=""

    say "START train arm $letter ($slug) seed $seed -> $log"
    # shellcheck disable=SC2086
    $PY "$REPO/scripts/train_nn_drive_curriculum.py" \
        --task "$task" \
        --experiment-name nn_drive_fixed_stance \
        --run-name-prefix "$prefix" \
        --num_envs "$NUM_ENVS" \
        --iterations $ITERATIONS \
        --selection last \
        --seed "$seed" \
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
    if verdict=$(verify_run "$stage1" "$kind" 2>&1); then
        say "  $(echo "$verdict" | tail -1)"
    else
        say "FAIL arm $letter seed $seed trained the wrong distribution:"
        say "  $(echo "$verdict" | tail -12)"
        return 1
    fi
    STAGE5_DIR="$stage5"
}

# $1 arm letter, $2 seed, $3 stage-5 run dir, $4 task, $5 condition, rest: overrides
bench_arm() {
    local letter=$1 seed=$2 stage5=$3 task=$4 condition=$5
    shift 5
    local ckpt out log rc attempt
    ckpt=$($PY -c "import json,sys;print(json.load(open(sys.argv[1]))['selected_checkpoint'])" \
        "$stage5/selected_checkpoint.json" 2>/dev/null)
    if [ -z "$ckpt" ]; then
        say "FAIL bench arm $letter seed $seed: no selected_checkpoint.json in $stage5"
        return 1
    fi
    out="$DATA_DIR/arm_${letter}_under_${condition}_s${seed}.json"
    log="$LOG_DIR/arm_${letter}_s${seed}_under_${condition}_bench.log"

    say "START bench arm $letter seed $seed under $condition ($ckpt)"
    # Kit occasionally SIGSEGVs at launch; the benchmark is seeded, so a retry
    # re-runs the identical measurement.
    for attempt in 1 2 3; do
        $PY "$REPO/scripts/benchmark_nn_drive.py" \
            --task "$task" \
            --checkpoint "$stage5/$ckpt" \
            --num_envs "$BENCH_ENVS" \
            --num_steps "$BENCH_STEPS" \
            --terrain generator \
            --seed 42 \
            --json-output "$out" \
            --headless \
            "$@" >"$log" 2>&1
        rc=$?
        [ $rc -eq 0 ] && break
        say "  bench attempt $attempt failed (exit $rc)"
    done
    if [ $rc -ne 0 ]; then
        say "FAIL bench arm $letter seed $seed under $condition (exit $rc) -- see $log"
        return $rc
    fi
    say "OK bench arm $letter seed $seed under $condition -> $(basename "$out")"
}

wants() { [[ " $ARMS " == *" $1 "* ]]; }

say "=== retrain ($TAG) started for seeds: ${SEEDS[*]}, arms: $ARMS, iterations: $ITERATIONS ==="
say "repo commit: $(cd "$REPO" && git rev-parse --short HEAD) $(cd "$REPO" && git status --porcelain --untracked-files=no | wc -l) dirty files"

for seed in "${SEEDS[@]}"; do
    say "--- seed $seed ---"
    if wants A && train_arm A point "$seed" "$MLP_TASK" point "${POINT_OVERRIDES[@]}"; then
        bench_arm A "$seed" "$STAGE5_DIR" "$MLP_TASK" nominal "${POINT_OVERRIDES[@]}"
        bench_arm A "$seed" "$STAGE5_DIR" "$MLP_TASK" range
    fi
    if wants B && train_arm B range "$seed" "$MLP_TASK" range; then
        bench_arm B "$seed" "$STAGE5_DIR" "$MLP_TASK" range
    fi
    if wants C && train_arm C range_gru "$seed" "$GRU_TASK" range; then
        bench_arm C "$seed" "$STAGE5_DIR" "$GRU_TASK" range
    fi
    if wants D && train_arm D range_wide "$seed" "$WIDE_TASK" range; then
        bench_arm D "$seed" "$STAGE5_DIR" "$WIDE_TASK" range
    fi
    say "--- seed $seed complete ---"
done

say "=== retrain ($TAG) finished ==="
