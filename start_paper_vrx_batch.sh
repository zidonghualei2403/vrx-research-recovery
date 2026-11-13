#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEED="${1:-20261018}"
mkdir -p "$ROOT/logs"
PID_FILE="$ROOT/.paper_vrx_batch_${SEED}.pid"
BATCH_DIR="$ROOT/results/paper_matched_body_aware_seed${SEED}"
HEARTBEAT="$BATCH_DIR/heartbeat.txt"

stop_stale_batch() {
  local pid="$1"
  echo "Stopping stale batch process group $pid before resuming..."
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  sleep 3
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
  rm -f "$PID_FILE" "$HEARTBEAT" "$BATCH_DIR/current.txt"
}

if [[ -f "$PID_FILE" ]]; then
  old_pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$old_pid" ]] && kill -0 "$old_pid" 2>/dev/null; then
    now="$(date +%s)"
    heartbeat_age=999999
    if [[ -f "$HEARTBEAT" ]]; then
      heartbeat_time="$(cut -d, -f1 "$HEARTBEAT" 2>/dev/null || echo 0)"
      heartbeat_age=$((now - heartbeat_time))
    elif [[ -f "$BATCH_DIR/current.txt" ]]; then
      current_time="$(stat -c %Y "$BATCH_DIR/current.txt" 2>/dev/null || echo 0)"
      heartbeat_age=$((now - current_time))
    fi
    if (( heartbeat_age < 120 )); then
      echo "Batch is already running with PID $old_pid (heartbeat ${heartbeat_age}s ago)"
      exit 0
    fi
    stop_stale_batch "$old_pid"
  else
    rm -f "$PID_FILE"
  fi
fi

log="$ROOT/logs/paper_vrx_batch_${SEED}_$(date +%Y%m%d_%H%M%S).log"
nohup setsid bash "$ROOT/run_paper_vrx_batch.sh" "$SEED" >"$log" 2>&1 &
pid=$!
echo "$pid" > "$PID_FILE"
echo "PID=$pid"
echo "LOG=$log"
echo "Restart this command after an interruption; completed cells will be skipped."
