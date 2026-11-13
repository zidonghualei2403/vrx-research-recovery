#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEED="${1:-20261011}"
MAX_DURATION="${MAX_EXPERIMENT_DURATION:-300}"
RUN_TIMEOUT_VALUE="${RUN_TIMEOUT:-390}"
HEADLESS_VALUE="${HEADLESS:-true}"
SAC_AUTHORITY="${RESIDUAL_SAC_AUTHORITY:-0.20}"

echo "Strong + extreme matched harbor comparison suite"
echo "seed=$SEED max_duration=${MAX_DURATION}s headless=$HEADLESS_VALUE"

for sea_state in strong_wind_wave extreme_wind_wave; do
  echo
  echo "===== START $sea_state ====="
  MAX_EXPERIMENT_DURATION="$MAX_DURATION" \
  RUN_TIMEOUT="$RUN_TIMEOUT_VALUE" \
  HEADLESS="$HEADLESS_VALUE" \
  RESIDUAL_SAC_AUTHORITY="$SAC_AUTHORITY" \
    bash "$ROOT/run_harbor_horizontal_comparison.sh" "$sea_state" "$SEED"
  echo "===== DONE  $sea_state ====="
done

echo
echo "STRONG_EXTREME_HORIZONTAL_SUITE=COMPLETE"
