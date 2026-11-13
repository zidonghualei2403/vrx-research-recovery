#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Stage 1/2: resume the medium baseline campaign and finish GNN-GRU."
REPEATS_PER_SEED=20 bash "$ROOT/run_medium_baselines_new5_gui.sh"
status=$?
if [[ "$status" -ne 0 ]]; then
  echo "GNN-GRU stage stopped or failed (exit=$status); C119 was not started."
  exit "$status"
fi
if [[ -f "$ROOT/results/medium_baselines_new5_gui_20261104/STOP_REQUESTED" ]]; then
  echo "GNN-GRU stage was stopped by request; C119 was not started."
  exit 0
fi

echo "Stage 2/2: complete matched C119 SAC runs."
bash "$ROOT/run_c119_medium_matched5seed_20repeat_gui.sh"
