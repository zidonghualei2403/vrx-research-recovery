#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEED="${1:-20261011}"
MAX_DURATION="${MAX_EXPERIMENT_DURATION:-300}"
RUN_TIMEOUT_VALUE="${RUN_TIMEOUT:-390}"
HEADLESS_VALUE="${HEADLESS:-false}"

mkdir -p "$ROOT/logs" "$ROOT/results/extreme_wind_wave_horizontal_seed${SEED}"

stamp="$(date +%Y%m%d_%H%M%S)"
log="$ROOT/logs/extreme_horizontal_standalone_${SEED}_${stamp}.log"

nohup env \
  MAX_EXPERIMENT_DURATION="$MAX_DURATION" \
  RUN_TIMEOUT="$RUN_TIMEOUT_VALUE" \
  HEADLESS="$HEADLESS_VALUE" \
  RUN_ATTEMPTS="${RUN_ATTEMPTS:-3}" \
  RUN_RETRY_DELAY="${RUN_RETRY_DELAY:-15}" \
  RESIDUAL_SAC_AUTHORITY="${RESIDUAL_SAC_AUTHORITY:-0.20}" \
  bash "$ROOT/run_harbor_horizontal_comparison.sh" \
    extreme_wind_wave "$SEED" > "$log" 2>&1 &

pid=$!
printf '%s\n' "$pid" > "$ROOT/results/extreme_wind_wave_horizontal_seed${SEED}/batch.pid"
echo "Extreme-only comparison started."
echo "PID: $pid"
echo "LOG: $log"
echo "HEADLESS: $HEADLESS_VALUE"

