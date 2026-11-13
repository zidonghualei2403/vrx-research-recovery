#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEED="${1:-20261014}"
SEA_STATES=(calm medium_wind_wave strong_wind_wave extreme_wind_wave)

for sea in "${SEA_STATES[@]}"; do
  case "$sea" in
    calm)
      max_duration=240
      run_timeout=330
      ;;
    medium_wind_wave|strong_wind_wave)
      max_duration=300
      run_timeout=390
      ;;
    extreme_wind_wave)
      max_duration=360
      run_timeout=450
      ;;
  esac

  printf '%s\n' "$sea" > "$ROOT/.matched_baselines_current"
  MAX_EXPERIMENT_DURATION="$max_duration" \
  RUN_TIMEOUT="$run_timeout" \
  HEADLESS=true \
  RUN_ATTEMPTS=4 \
    bash "$ROOT/run_harbor_horizontal_comparison.sh" "$sea" "$SEED"
done

rm -f "$ROOT/.matched_baselines_current"
touch "$ROOT/.matched_baselines_complete"
echo "MATCHED_BASELINES=COMPLETE seed=$SEED"
