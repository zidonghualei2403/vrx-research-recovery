#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEED="${1:-20261017}"
mkdir -p "$ROOT/logs"

is_our_process() {
  local pid="$1"
  [[ -r "/proc/$pid/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/$pid/cmdline" \
    | grep -Fq "run_v6_disturbed_sea_suite.sh"
}

if [[ -f "$ROOT/.v6_disturbed_pid" ]]; then
  old_pid="$(cat "$ROOT/.v6_disturbed_pid" 2>/dev/null || true)"
  if [[ -n "$old_pid" ]] && kill -0 "$old_pid" 2>/dev/null \
      && is_our_process "$old_pid"; then
    echo "Locked-V6 disturbed-sea suite is already running with PID $old_pid."
    exit 0
  fi
fi

stamp="$(date +%Y%m%d_%H%M%S)"
log="$ROOT/logs/v6_paper_disturbed_suite_${SEED}_${stamp}.log"
rm -f "$ROOT/.v6_disturbed_complete"
nohup bash "$ROOT/run_v6_disturbed_sea_suite.sh" "$SEED" >"$log" 2>&1 &
pid=$!
printf '%s\n' "$pid" > "$ROOT/.v6_disturbed_pid"
printf '%s\n' "$log" > "$ROOT/.v6_disturbed_log"
printf '%s\n' "$SEED" > "$ROOT/.v6_disturbed_seed"

echo "PID: $pid"
echo "LOG: $log"
echo "SEED: $SEED"
echo "Restart this command after a shutdown to resume unfinished sea states."
