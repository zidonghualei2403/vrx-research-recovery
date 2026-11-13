#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAMPAIGN="${CALM_BASELINE_CAMPAIGN:-$ROOT/results/calm_baselines_5seed_10repeat_gui_20261103}"
mkdir -p "$CAMPAIGN"
touch "$CAMPAIGN/STOP_REQUESTED"

child="$(cat "$CAMPAIGN/child.pid" 2>/dev/null || true)"
if [[ -n "$child" ]] && kill -0 "$child" 2>/dev/null; then
  kill -INT -- "-$child" 2>/dev/null || kill -INT "$child" 2>/dev/null || true
fi
echo "Stop requested. The current simulation will close; all completed runs are preserved."
