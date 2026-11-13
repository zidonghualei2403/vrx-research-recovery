#!/usr/bin/env bash
# Clean only stale six-vessel VRX processes created by this experiment family.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STOP_REQUEST="$ROOT/.stop_paper_vrx_batch"

# A detached batch can live in a different PID namespace from a later Codex
# wake-up. Let the batch stop itself at the next cleanup boundary instead of
# relying on an out-of-namespace kill.
if [[ -f "$STOP_REQUEST" ]]; then
  parent_pid="$PPID"
  parent_cmd="$(tr '\0' ' ' < "/proc/$parent_pid/cmdline" 2>/dev/null || true)"
  if [[ "$parent_cmd" == *"run_paper_vrx_batch.sh"* ]]; then
    echo "Paper batch stop request acknowledged by PID $parent_pid."
    kill -TERM "$parent_pid" 2>/dev/null || true
    exit 0
  fi
fi

declare -A PROTECTED_PIDS=()
ancestor="$$"
while [[ -n "$ancestor" && "$ancestor" -gt 1 ]]; do
  PROTECTED_PIDS["$ancestor"]=1
  ancestor="$(ps -o ppid= -p "$ancestor" 2>/dev/null | tr -d ' ' || true)"
done

matching_pids() {
  local pattern="$1"
  local pid comm
  while read -r pid; do
    [[ -z "$pid" || -n "${PROTECTED_PIDS[$pid]:-}" ]] && continue
    comm="$(ps -o comm= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
    case "$comm" in
      python|python3|ruby|gz|gz-sim|ros2) ;;
      *) continue ;;
    esac
    printf '%s\n' "$pid"
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
}

signal_matching() {
  local signal="$1"
  local pattern="$2"
  local pid
  while read -r pid; do
    [[ -z "$pid" ]] && continue
    kill "-$signal" "$pid" 2>/dev/null || true
  done < <(matching_pids "$pattern")
}

has_matching_process() {
  [[ -n "$(matching_pids "$1")" ]]
}

pattern='six_vessel_crossing_driver.py|csd_shadow_graph.py|gz sim.*csd_harbor'
signal_matching TERM 'six_vessel_crossing_driver.py'
signal_matching TERM 'csd_shadow_graph.py'
signal_matching TERM 'gz sim.*csd_harbor'

deadline=$((SECONDS + 20))
while (( SECONDS < deadline )) && has_matching_process "$pattern"; do
  sleep 1
done

if has_matching_process "$pattern"; then
  signal_matching KILL 'six_vessel_crossing_driver.py'
  signal_matching KILL 'csd_shadow_graph.py'
  signal_matching KILL 'gz sim.*csd_harbor'
  sleep 2
fi

if has_matching_process "$pattern"; then
  echo "ERROR: stale six-vessel VRX processes remain." >&2
  exit 1
fi

echo "Six-vessel VRX process cleanup complete."
