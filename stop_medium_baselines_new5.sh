#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BATCH_DIR="$ROOT/results/medium_baselines_new5_gui_20261104"
mkdir -p "$BATCH_DIR"
touch "$BATCH_DIR/STOP_REQUESTED"
PID_FILE="$BATCH_DIR/runner.pid"
if [[ -f "$PID_FILE" ]]; then
  pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
  fi
fi
bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
echo "Stop requested. Completed runs are preserved; rerun the start command to resume."
