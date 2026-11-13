#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BATCH_NAME="${BASELINE_BATCH_NAME:-baseline_5seed_50runs_gui_verified_20261101}"
BATCH_DIR="$ROOT/results/$BATCH_NAME"
PID_FILE="$ROOT/.baseline_5seed_50runs.pid"

mkdir -p "$BATCH_DIR"
touch "$BATCH_DIR/STOP_REQUESTED"

if [[ -f "$PID_FILE" ]]; then
  pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  fi
fi

bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
echo "Stop requested. Completed CSV rows and result directories are preserved."
