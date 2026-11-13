#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BATCH="$ROOT/autonomous_medium_iterations_20261018/candidate_64_c60_wamv1_post_conflict_recovery/evidence/batch_calm_medium_3seed_gui_v1"
PID_FILE="$BATCH/batch.pid"
mkdir -p "$BATCH" "$ROOT/logs"

if [[ -f "$PID_FILE" ]]; then
  old_pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$old_pid" ]] && kill -0 "$old_pid" 2>/dev/null; then
    echo "Batch is already running with PID $old_pid"
    exit 0
  fi
fi

rm -f "$BATCH/STOP_REQUESTED"
log="$ROOT/logs/c64_calm_medium_3seed_gui_$(date +%Y%m%d_%H%M%S).log"
nohup setsid bash "$ROOT/run_c64_calm_medium_3seed_gui_batch.sh" >"$log" 2>&1 &
pid=$!
echo "$pid" > "$PID_FILE"
echo "PID=$pid"
echo "LOG=$log"
echo "RESULTS=$BATCH"
