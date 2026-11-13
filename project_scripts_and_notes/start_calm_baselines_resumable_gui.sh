#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAMPAIGN="${CALM_BASELINE_CAMPAIGN:-$ROOT/results/calm_baselines_5seed_10repeat_gui_20261103}"
PID_FILE="$CAMPAIGN/runner.pid"
mkdir -p "$CAMPAIGN" "$ROOT/logs"

if [[ -f "$PID_FILE" ]]; then
  pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    echo "Calm baseline campaign is already running (PID=$pid)."
    exit 0
  fi
  rm -f "$PID_FILE"
fi

log="$ROOT/logs/calm_baselines_resumable_$(date +%Y%m%d_%H%M%S).log"
nohup setsid bash "$ROOT/run_calm_baselines_resumable_gui.sh" >"$log" 2>&1 &
pid=$!
echo "$pid" > "$PID_FILE"
echo "$log" > "$CAMPAIGN/launcher.log.path"
echo "PID=$pid"
echo "LOG=$log"
echo "RESULTS=$CAMPAIGN"
echo "Run the progress command in another terminal."
