#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEA_STATE="${1:-medium_wind_wave}"
SEED="${2:-20261011}"
MAX_DURATION="${MAX_EXPERIMENT_DURATION:-240}"
MIN_COMPLETE_DURATION="$(awk -v duration="$MAX_DURATION" 'BEGIN { print duration - 10.0 }')"
RUN_TIMEOUT="${RUN_TIMEOUT:-330}"
HEADLESS_MODE="${HEADLESS:-true}"
MAX_ATTEMPTS="${RUN_ATTEMPTS:-3}"
RETRY_DELAY="${RUN_RETRY_DELAY:-12}"
METHODS=(distance_star risk_star conditional_csd gnn_gru gnn_gru_sac)

case "$SEA_STATE" in
  calm|medium_wind_wave|strong_wind_wave|extreme_wind_wave) ;;
  *)
    echo "Sea state must be calm, medium_wind_wave, strong_wind_wave, or extreme_wind_wave" >&2
    exit 2 ;;
esac

OUTPUT_DIR="$ROOT/results/${SEA_STATE}_horizontal_seed${SEED}"
mkdir -p "$ROOT/logs" "$OUTPUT_DIR"

echo "Matched harbor horizontal comparison"
echo "sea_state=$SEA_STATE seed=$SEED max_duration=${MAX_DURATION}s headless=$HEADLESS_MODE"

for method in "${METHODS[@]}"; do
  if python3 "$ROOT/summarize_calm_horizontal.py" \
      --sea-state "$SEA_STATE" --seed "$SEED" \
      --minimum-duration "$MIN_COMPLETE_DURATION" \
      --is-finished "$method" >/dev/null 2>&1; then
    echo "SKIP $method: a complete matched run already exists."
    continue
  fi

  printf '%s\n' "$method" > "$OUTPUT_DIR/current_method.txt"
  method_done=false
  for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
    # A completed Gazebo launch can leave transport and bridge processes alive
    # briefly. Starting the next world during that interval causes stale or
    # missing pose samples and a false STARTUP_VALIDATION_FAILED result.
    bash "$ROOT/cleanup_vrx_batch_processes.sh"
    sleep 4

    stamp="$(date +%Y%m%d_%H%M%S)"
    log="$ROOT/logs/${SEA_STATE}_horizontal_${method}_${SEED}_${stamp}_attempt${attempt}.log"
    echo "RUN  $method attempt=$attempt/$MAX_ATTEMPTS -> $log"

    set +e
    timeout --signal=INT --kill-after=25s "${RUN_TIMEOUT}s" \
      env MAX_EXPERIMENT_DURATION="$MAX_DURATION" \
          RESIDUAL_SAC_AUTHORITY="${RESIDUAL_SAC_AUTHORITY:-0.20}" \
          HEADLESS="$HEADLESS_MODE" \
      bash "$ROOT/run_harbor_gnn_gru_vrx.sh" \
        "$SEA_STATE" "$SEED" "$method" > "$log" 2>&1
    status=$?
    set -e

    bash "$ROOT/cleanup_vrx_batch_processes.sh" || true

    if python3 "$ROOT/summarize_calm_horizontal.py" \
        --sea-state "$SEA_STATE" --seed "$SEED" \
        --minimum-duration "$MIN_COMPLETE_DURATION" \
        --is-finished "$method" >/dev/null 2>&1; then
      echo "DONE $method attempt=$attempt"
      method_done=true
      break
    fi

    echo "RETRY $method: attempt=$attempt launcher_status=$status" >&2
    echo "The failed run is preserved in $log" >&2
    if (( attempt < MAX_ATTEMPTS )); then
      sleep "$RETRY_DELAY"
    fi
  done

  if [[ "$method_done" != true ]]; then
    echo "FAIL $method after $MAX_ATTEMPTS attempts. Last log: $log" >&2
    echo "Completed methods are preserved; run this script again to resume." >&2
    python3 "$ROOT/summarize_calm_horizontal.py" \
      --sea-state "$SEA_STATE" --seed "$SEED" --print-table || true
    exit 1
  fi
  sleep 8
done

rm -f "$OUTPUT_DIR/current_method.txt"
python3 "$ROOT/summarize_calm_horizontal.py" \
  --sea-state "$SEA_STATE" --seed "$SEED" \
  --minimum-duration "$MIN_COMPLETE_DURATION" --print-table
echo "HARBOR_HORIZONTAL_COMPARISON=COMPLETE"
