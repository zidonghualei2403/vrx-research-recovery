#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ITER_NAME="${1:-iter_00}"
ITER_DIR="$ROOT/autonomous_medium_iterations_20261018/$ITER_NAME"
PID_FILE="$ITER_DIR/runner.pid"

mkdir -p "$ITER_DIR"
if [[ -f "$PID_FILE" ]]; then
  pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    echo "Locked V6 reproduction is already running with PID $pid."
    exit 0
  fi
fi

nohup setsid bash "$ROOT/run_autonomous_locked_v6_iter00.sh" "$ITER_NAME" \
  > "$ITER_DIR/runner.log" 2>&1 &
pid=$!
echo "$pid" > "$PID_FILE"
echo "PID=$pid"
echo "ITER_DIR=$ITER_DIR"
