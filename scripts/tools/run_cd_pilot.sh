#!/usr/bin/env bash
# Run inside isaac-lab-dev; the live repo must stay unchanged during training.
set -euo pipefail
cd /workspace/TwoWheeledRobot
export RETRAIN_TAG=${RETRAIN_TAG:-r3_cd_pilot_$(date -u +%Y%m%dT%H%M%SZ)}
export RETRAIN_ARMS="C D"
export RETRAIN_ITERATIONS="300 350 200 250 150"
export RETRAIN_NUM_ENVS=4096
export RETRAIN_PILOT_PROTOCOL=/workspace/TwoWheeledRobot/docs/experiments/2026-10-06-cd-pilot/protocol.json
export RETRAIN_DATA_DIR=/workspace/TwoWheeledRobot/logs/pilot_outputs/$RETRAIN_TAG
pilot_log="logs/retrain_${RETRAIN_TAG}_$(date +%Y-%m-%d)"
if [ -e "$pilot_log" ] || [ -e "$RETRAIN_DATA_DIR" ]; then
    echo "Refusing to overwrite an existing pilot: $RETRAIN_TAG" >&2
    exit 2
fi
if ! git diff --quiet HEAD -- scripts source docs/experiments/2026-10-06-cd-pilot; then
    echo "Commit the pilot source/configuration before launch" >&2
    exit 2
fi
mkdir -p "$pilot_log"
git rev-parse HEAD > "$pilot_log/SOURCE_COMMIT.txt"
cp "$RETRAIN_PILOT_PROTOCOL" "$pilot_log/protocol.json"
echo "PRECHECK started" > "$pilot_log/STATUS.txt"
if /isaac-sim/python.sh scripts/tools/verify_retrain_changes.py --headless > "$pilot_log/preflight.log" 2>&1 \
   && /isaac-sim/python.sh -c 'import pathlib,sys; assert "[verify] DONE failures=[]" in pathlib.Path(sys.argv[1]).read_text()' "$pilot_log/preflight.log"; then
    echo "PRECHECK passed" >> "$pilot_log/STATUS.txt"
else
    echo "FAIL preflight; no training launched" >> "$pilot_log/STATUS.txt"
    exit 1
fi
exec bash scripts/tools/run_retrain_arms.sh 43
