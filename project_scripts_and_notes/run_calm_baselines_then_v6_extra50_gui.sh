#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASELINE_CAMPAIGN="$ROOT/results/calm_baselines_5seed_10repeat_gui_20261103"
V6_CAMPAIGN="$ROOT/locked_v6_calm_extra50_20261108"

echo "Stage 1/2: resume calm baselines to 100 runs per method."
echo "Existing completed runs will be skipped."
CALM_BASELINE_CAMPAIGN="$BASELINE_CAMPAIGN" \
REPEATS_PER_SEED=20 \
bash "$ROOT/run_calm_baselines_resumable_gui.sh"

if [[ ! -f "$BASELINE_CAMPAIGN/FINISHED" ]]; then
  echo "Baseline stage stopped before completion; V6 extra runs were not started."
  exit 0
fi

echo
echo "Stage 2/2: run 50 additional locked-V6 calm trials (5 seeds x 10 repeats)."
CAMPAIGN_OVERRIDE="$V6_CAMPAIGN" \
TARGET_REPEATS_OVERRIDE=10 \
bash "$ROOT/run_locked_v6_calm_5seed_10repeat_gui.sh"

echo
echo "BOTH_STAGES_COMPLETE"
echo "BASELINES=$BASELINE_CAMPAIGN"
echo "V6_EXTRA50=$V6_CAMPAIGN"
