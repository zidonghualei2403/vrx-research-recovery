#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEED="${1:-20261014}"
PID_FILE="$ROOT/.matched_baselines.pid"

if [[ -f "$PID_FILE" ]]; then
  old_pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$old_pid" ]] && kill -0 "$old_pid" 2>/dev/null; then
    echo "Matched baseline batch is already running with PID $old_pid."
    exit 0
  fi
fi

mkdir -p "$ROOT/logs"
rm -f "$ROOT/.matched_baselines_complete"
stamp="$(date +%Y%m%d_%H%M%S)"
log="$ROOT/logs/matched_baselines_${SEED}_${stamp}.log"

nohup setsid bash "$ROOT/run_matched_baselines_20261014.sh" "$SEED" >"$log" 2>&1 &
pid=$!
printf '%s\n' "$pid" > "$PID_FILE"
printf '%s\n' "$log" > "$ROOT/.matched_baselines_log"

echo "PID: $pid"
echo "LOG: $log"
echo "The batch is resumable: run this start script again after interruption."
