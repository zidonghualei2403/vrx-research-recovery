#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pid="$(cat "$ROOT/.v6_suite_pid" 2>/dev/null || true)"
is_our_process() {
  local candidate="$1"
  [[ -r "/proc/$candidate/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/$candidate/cmdline" | grep -Fq "run_v6_sea_suite.sh"
}

if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null \
    || ! is_our_process "$pid"; then
  echo "No active V6 suite process was found. Completed CSV files are preserved."
  exit 0
fi

kill -INT "$pid" 2>/dev/null || true
sleep 3
kill -TERM "$pid" 2>/dev/null || true
echo "Stop requested for PID $pid. Completed sea states remain resumable."
