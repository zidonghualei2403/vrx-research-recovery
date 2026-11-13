#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BATCH_NAME="${BASELINE_BATCH_NAME:-baseline_5seed_50runs_gui_verified_20261101}"
BATCH_DIR="$ROOT/results/$BATCH_NAME"
PID_FILE="$ROOT/.baseline_5seed_50runs.pid"
HEARTBEAT="$BATCH_DIR/heartbeat.txt"

mkdir -p "$BATCH_DIR" "$ROOT/logs"
if [[ -f "$PID_FILE" ]]; then
  old_pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$old_pid" ]] && kill -0 "$old_pid" 2>/dev/null; then
    age=999999
    if [[ -f "$HEARTBEAT" ]]; then
      heartbeat_epoch="$(cut -d, -f1 "$HEARTBEAT" 2>/dev/null || echo 0)"
      age=$(( $(date +%s) - heartbeat_epoch ))
    fi
    if (( age < 120 )); then
      echo "Baseline batch is already running with PID $old_pid (heartbeat ${age}s ago)."
      exit 0
    fi
    echo "A stale batch PID exists. Run stop_baseline_5seed_50runs.sh, then start again." >&2
    exit 1
  fi
  rm -f "$PID_FILE"
fi

log="$ROOT/logs/baseline_5seed_50runs_$(date +%Y%m%d_%H%M%S).log"
nohup setsid bash "$ROOT/run_baseline_5seed_50runs.sh" >"$log" 2>&1 &
pid=$!
echo "$pid" > "$PID_FILE"
echo "PID=$pid"
echo "LOG=$log"
echo "RESULTS=$BATCH_DIR"
echo "Restart this script after an interruption; completed runs will be skipped."
