#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEED="${1:-20261011}"
MAX_DURATION="${MAX_EXPERIMENT_DURATION:-180}"
RUN_TIMEOUT="${RUN_TIMEOUT:-260}"
HEADLESS_MODE="${HEADLESS:-true}"
METHODS=(distance_star risk_star conditional_csd gnn_gru gnn_gru_sac)

mkdir -p "$ROOT/logs" "$ROOT/results/calm_horizontal_seed${SEED}"

echo "Calm-water matched horizontal comparison"
echo "seed=$SEED max_duration=${MAX_DURATION}s headless=$HEADLESS_MODE"

for method in "${METHODS[@]}"; do
  if python3 "$ROOT/summarize_calm_horizontal.py" \
      --seed "$SEED" --is-finished "$method" >/dev/null 2>&1; then
    echo "SKIP $method: a complete matched run already exists."
    continue
  fi

  stamp="$(date +%Y%m%d_%H%M%S)"
  log="$ROOT/logs/calm_horizontal_${method}_${SEED}_${stamp}.log"
  state="$ROOT/results/calm_horizontal_seed${SEED}/current_method.txt"
  printf '%s\n' "$method" > "$state"
  echo "RUN  $method -> $log"

  set +e
  timeout --signal=INT --kill-after=25s "${RUN_TIMEOUT}s" \
    env MAX_EXPERIMENT_DURATION="$MAX_DURATION" \
        RESIDUAL_SAC_AUTHORITY="${RESIDUAL_SAC_AUTHORITY:-0.20}" \
        HEADLESS="$HEADLESS_MODE" \
    bash "$ROOT/run_harbor_gnn_gru_vrx.sh" calm "$SEED" "$method" \
    > "$log" 2>&1
  status=$?
  set -e

  if python3 "$ROOT/summarize_calm_horizontal.py" \
      --seed "$SEED" --is-finished "$method" >/dev/null 2>&1; then
    echo "DONE $method"
  else
    echo "FAIL $method (launcher status=$status). See $log" >&2
    echo "Saved completed methods are preserved. Run this script again to resume." >&2
    python3 "$ROOT/summarize_calm_horizontal.py" --seed "$SEED" --print-table || true
    exit 1
  fi
  sleep 5
done

rm -f "$ROOT/results/calm_horizontal_seed${SEED}/current_method.txt"
python3 "$ROOT/summarize_calm_horizontal.py" --seed "$SEED" --print-table
echo "CALM_HORIZONTAL_COMPARISON=COMPLETE"
