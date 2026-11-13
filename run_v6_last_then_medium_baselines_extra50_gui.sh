#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
V6_CAMPAIGN="$ROOT/locked_v6_calm_extra50_20261108"
MEDIUM_CAMPAIGN="$ROOT/results/medium_baselines_new5_gui_20261104"

echo "Stage 1/2: finish the locked-V6 calm extra-50 campaign."
CAMPAIGN_OVERRIDE="$V6_CAMPAIGN" \
TARGET_REPEATS_OVERRIDE=10 \
bash "$ROOT/run_locked_v6_calm_5seed_10repeat_gui.sh"

v6_rows="$(awk 'END {print NR > 0 ? NR - 1 : 0}' "$V6_CAMPAIGN/results.tsv" 2>/dev/null || echo 0)"
if (( v6_rows < 50 )); then
  echo "V6 stage stopped at $v6_rows/50; medium baselines were not started."
  exit 0
fi

echo
echo "Stage 2/2: extend medium-water baselines from 50 to 100 runs per method."
echo "Existing repeats 1-10 are preserved; only repeats 11-20 will run."
REPEATS_PER_SEED=20 \
MAX_EXPERIMENT_DURATION=180 \
bash "$ROOT/run_medium_baselines_new5_gui.sh"

echo
echo "ALL_REQUESTED_RUNS_COMPLETE"
echo "V6=$V6_CAMPAIGN"
echo "MEDIUM_BASELINES=$MEDIUM_CAMPAIGN"
