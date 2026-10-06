#!/usr/bin/env bash
# Retrain the paper arms after the 2026-09-30 review audit, and benchmark them.
#
# Runs INSIDE the isaac-lab-dev container. Differences from run_multiseed_arms.sh
# (the 2026-08 arms) are the fixes the audit found, plus one extra arm:
#
#   obs[9:10]        policy's own clamped command (firmware semantics), not the
#                    motor model's current          prev_current_obs_source=command
#   current lag      removed (it was inert); latency is the action/obs delays
#   left wheel       torque-speed envelope uses the logical wheel velocity,
#                    evaluated every 1 ms physics substep
#   tire friction    effective static 0.5-1.1 / dynamic 0.4-0.9 per env per
#                    episode (was one draw per run, halved by the 0.5 default
#                    robot material)              ground_friction_randomization_mode=per_episode
#   noise floor      restored to 0.15 after C43's hardware regression;
#                    the archived r2 run used 0.05
#   selection        fixed budget, last checkpoint of each stage (was a gated
#                    shortlist benchmarked on the same seed the paper reports)
#   removed          inert: joint friction coefficient, current bias, randomized
#                    current limit, "position_far" reward, reward clip;
#                    negligible: +-1 mm COM fore-aft, CyberGear gain randomization
#   wheel damping    0.0007 N m s/rad measured (was 0.006-0.014 assumed, ~15x)
#   no-load speed    270 rpm measured (was 200 rpm datasheet)
#   hold drift       measured from where the hold began, not the episode spawn
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
# Changed defaults must not overwrite the completed r2 benchmark records.
TAG=${RETRAIN_TAG:-r3}
RUN_ROOT=$REPO/logs/rsl_rl/nn_drive_fixed_stance
LOG_DIR=$REPO/logs/retrain_${TAG}_$(date +%Y-%m-%d)
DATA_DIR=${RETRAIN_DATA_DIR:-$REPO/docs/paper/figures/data_${TAG}}
STATUS=$LOG_DIR/STATUS.txt
ITERATIONS=${RETRAIN_ITERATIONS:-"300 350 200 250 300"}
NUM_ENVS=${RETRAIN_NUM_ENVS:-4096}
BENCH_ENVS=${RETRAIN_BENCH_ENVS:-256}
BENCH_STEPS=${RETRAIN_BENCH_STEPS:-1000}
ARMS=${RETRAIN_ARMS:-"A B C D"}
PILOT_PROTOCOL=${RETRAIN_PILOT_PROTOCOL:-}
FAILURES=0
if [ -n "$PILOT_PROTOCOL" ]; then
    if [ "$ARMS" != "C D" ] || [ "${SEEDS[*]}" != "43" ] || \
       [ "$ITERATIONS" != "300 350 200 250 150" ] || [ "$NUM_ENVS" != "4096" ]; then
        echo "Pilot settings must match C/D, seed 43, 4096 envs, 1250 updates" >&2
        exit 2
    fi
    $PY "$REPO/scripts/tools/select_cd_pilot_checkpoint.py" \
        --protocol "$PILOT_PROTOCOL" --check-only || exit 2
fi
mkdir -p "$LOG_DIR" "$DATA_DIR"

# Arm A collapses the wheel-actuator ranges (gain, deadzone) to their point
# estimates. That is now the ONLY difference from arm B: CyberGear gains are
# fixed at 30/3 in every arm, and friction, delays, sensors and body are
# randomized identically.
POINT_OVERRIDES=(
    "env.motor_gain_range=[1.0,1.0]"
    "env.motor_deadzone_a_range=[0.0534,0.0534]"
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
    },
    "range": {
        "motor_gain_range": (0.97, 1.03),
        "motor_deadzone_a_range": (0.031, 0.078),
    },
}[expect]
removed = ["motor_tau_s_range", "wheel_frictionloss_range", "motor_bias_a_range",
           "motor_current_limit_a_range", "rew_position_far", "pos_err_far_max_m"]
present = [k for k in removed if k in cfg]
if present:
    print(f"CONFIG MISMATCH: removed parameters still present: {present}")
    sys.exit(1)
want.update({
    "prev_current_obs_source": "command",
    "ground_friction_randomization_mode": "per_episode",
    "ground_static_friction_range": (0.5, 1.1),
    "ground_dynamic_friction_range": (0.4, 0.9),
    "obs_delay_steps_range": (0, 1),
    "action_delay_steps_range": (0, 1),
    "cg_kp_range": (30.0, 30.0),
    "cg_kd_range": (3.0, 3.0),
    "com_offset_y_range_m": (0.0, 0.0),
    "wheel_viscous_damping_range": (0.0007, 0.0007),
    "rew_cg_pos": 0.0,
    "rew_cg_rate": 0.0,
})
bad = []
for key, value in want.items():
    got = cfg[key]
    got = tuple(got) if isinstance(got, (list, tuple)) else got
    if got != value:
        bad.append(f"{key}: got {got}, expected {value}")
floor = tuple(agent.get("action_std_floor") or ())
if floor != (0.15, 0.15):
    bad.append(f"action_std_floor: got {floor}, expected (0.15, 0.15)")
if agent.get("num_steps_per_env") != 64 or agent.get("save_interval") != 25:
    bad.append("rollout/save interval must be 64/25")
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
    local extra_train=()
    # The pilot exports only a gate-passing held-out validation choice later.
    [ -n "$PILOT_PROTOCOL" ] && extra_train+=(--skip-export)

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
        "${extra_train[@]}" \
        "$@" >"$log" 2>&1
    local rc=$?
    if [ $rc -ne 0 ]; then
        say "FAIL train arm $letter seed $seed (exit $rc) -- see $log"
        return $rc
    fi

    local stage5 stage run_dir
    stage5=$(latest_run "$prefix" 5)
    if [ -z "$stage5" ]; then
        say "FAIL arm $letter seed $seed: no stage-5 run directory found"
        return 1
    fi
    say "OK train arm $letter seed $seed -> $(basename "$stage5")"
    local verdict
    for stage in 1 2 3 4 5; do
        run_dir=$(latest_run "$prefix" "$stage")
        if verdict=$(verify_run "$run_dir" "$kind" 2>&1); then
            say "  stage $stage: $(echo "$verdict" | tail -1)"
        else
            say "FAIL arm $letter seed $seed stage $stage trained the wrong configuration:"
            say "  $(echo "$verdict" | tail -12)"
            return 1
        fi
    done
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

run_arm() {
    local letter=$1 slug=$2 seed=$3 task=$4 kind=$5
    shift 5
    if ! train_arm "$letter" "$slug" "$seed" "$task" "$kind" "$@"; then
        FAILURES=$((FAILURES + 1))
        return
    fi
    if [ -n "$PILOT_PROTOCOL" ]; then
        local prefix="${TAG}_${slug}_s${seed}" stage4 log
        stage4=$(latest_run "$prefix" 4)
        log="$LOG_DIR/arm_${letter}_s${seed}_selection.log"
        say "START held-out selection/test/export arm $letter -> $log"
        if $PY "$REPO/scripts/tools/select_cd_pilot_checkpoint.py" \
            --protocol "$PILOT_PROTOCOL" --arm "$letter" --stage4 "$stage4" \
            --stage5 "$STAGE5_DIR" --output "$DATA_DIR/arm_${letter}_s${seed}" >"$log" 2>&1; then
            say "OK held-out selection/test/export arm $letter"
        else
            say "FAIL held-out selection/test/export arm $letter -- see $log"
            FAILURES=$((FAILURES + 1))
        fi
    else
        if [ "$letter" = A ]; then
            bench_arm "$letter" "$seed" "$STAGE5_DIR" "$task" nominal "$@" || FAILURES=$((FAILURES + 1))
        fi
        bench_arm "$letter" "$seed" "$STAGE5_DIR" "$task" range || FAILURES=$((FAILURES + 1))
    fi
}

say "=== retrain ($TAG) started for seeds: ${SEEDS[*]}, arms: $ARMS, iterations: $ITERATIONS ==="
say "repo commit: $(cd "$REPO" && git rev-parse --short HEAD) $(cd "$REPO" && git status --porcelain --untracked-files=no | wc -l) dirty files"

for seed in "${SEEDS[@]}"; do
    say "--- seed $seed ---"
    wants A && run_arm A point "$seed" "$MLP_TASK" point "${POINT_OVERRIDES[@]}"
    wants B && run_arm B range "$seed" "$MLP_TASK" range
    wants C && run_arm C range_gru "$seed" "$GRU_TASK" range
    wants D && run_arm D range_wide "$seed" "$WIDE_TASK" range
    say "--- seed $seed complete ---"
done

if [ "$FAILURES" -ne 0 ]; then
    say "=== retrain ($TAG) finished with $FAILURES failures ==="
    exit 1
fi
say "=== retrain ($TAG) finished ==="
