#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEED="${1:-20261014}"
mkdir -p "$ROOT/logs"

is_our_process() {
  local pid="$1"
  [[ -r "/proc/$pid/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/$pid/cmdline" | grep -Fq "run_v6_sea_suite.sh"
}

if [[ -f "$ROOT/.v6_suite_pid" ]]; then
  old_pid="$(cat "$ROOT/.v6_suite_pid" 2>/dev/null || true)"
  if [[ -n "$old_pid" ]] && kill -0 "$old_pid" 2>/dev/null \
      && is_our_process "$old_pid"; then
    echo "V6 suite is already running with PID $old_pid."
    exit 0
  fi
fi

stamp="$(date +%Y%m%d_%H%M%S)"
log="$ROOT/logs/v6_sea_suite_${SEED}_${stamp}.log"
rm -f "$ROOT/.v6_suite_complete"
nohup bash "$ROOT/run_v6_sea_suite.sh" "$SEED" >"$log" 2>&1 &
pid=$!
printf '%s\n' "$pid" > "$ROOT/.v6_suite_pid"
printf '%s\n' "$log" > "$ROOT/.v6_suite_log"

echo "PID: $pid"
echo "LOG: $log"
echo "The same command resumes unfinished sea states after a shutdown."
