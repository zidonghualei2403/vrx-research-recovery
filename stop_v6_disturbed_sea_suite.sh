#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pid="$(cat "$ROOT/.v6_disturbed_pid" 2>/dev/null || true)"

is_our_process() {
  local candidate="$1"
  [[ -r "/proc/$candidate/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/$candidate/cmdline" \
    | grep -Fq "run_v6_disturbed_sea_suite.sh"
}

if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null \
    || ! is_our_process "$pid"; then
  echo "No active locked-V6 disturbed-sea suite was found."
  echo "Completed CSV files are preserved."
  exit 0
fi

kill -INT "$pid" 2>/dev/null || true
sleep 3
kill -TERM "$pid" 2>/dev/null || true
bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
echo "Stop requested for PID $pid. Completed sea states remain resumable."
