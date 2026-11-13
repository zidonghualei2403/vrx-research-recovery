#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "$ROOT/stop_medium_baselines_new5.sh" >/dev/null 2>&1 || true
mkdir -p "$ROOT/c119_medium_matched5seed_20repeat_20261110"
touch "$ROOT/c119_medium_matched5seed_20repeat_20261110/STOP_REQUESTED"
echo "Stop requested. Completed GNN-GRU and C119 runs are preserved."
