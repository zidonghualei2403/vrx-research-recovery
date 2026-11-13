#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="$ROOT/.matched_baselines.pid"

if [[ ! -f "$PID_FILE" ]]; then
  echo "No matched baseline batch PID was recorded."
  exit 0
fi

pid="$(cat "$PID_FILE" 2>/dev/null || true)"
if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  echo "Stop requested for PID $pid. Completed CSV files are preserved."
else
  echo "The recorded process is not running. Completed CSV files are preserved."
fi
