#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEED="${1:-20261014}"
METHOD="gnn_gru_sac"
SEA_STATES=(calm medium_wind_wave strong_wind_wave extreme_wind_wave)

mkdir -p "$ROOT/logs"

is_complete() {
  local sea="$1"
  local minimum=290
  [[ "$sea" == "calm" ]] && minimum=170
  python3 "$ROOT/summarize_calm_horizontal.py" \
    --sea-state "$sea" --seed "$SEED" \
    --minimum-duration "$minimum" --is-finished "$METHOD" \
    >/dev/null 2>&1
}

for sea in "${SEA_STATES[@]}"; do
  if is_complete "$sea"; then
    echo "SKIP $sea: complete V6 result already exists."
    continue
  fi

  case "$sea" in
    calm) max_duration=240; run_timeout=330 ;;
    medium_wind_wave|strong_wind_wave) max_duration=300; run_timeout=390 ;;
    extreme_wind_wave) max_duration=360; run_timeout=450 ;;
  esac

  success=false
  for attempt in 1 2; do
    stamp="$(date +%Y%m%d_%H%M%S)"
    log="$ROOT/logs/v6_${sea}_${SEED}_${stamp}_attempt${attempt}.log"
    printf '%s\n' "$sea" > "$ROOT/.v6_suite_current"
    echo "RUN $sea attempt=$attempt/2 -> $log"

    set +e
    timeout --signal=INT --kill-after=25s "${run_timeout}s" \
      env MAX_EXPERIMENT_DURATION="$max_duration" \
          RESIDUAL_SAC_AUTHORITY=0.35 \
          HEADLESS=true \
      bash "$ROOT/run_harbor_gnn_gru_vrx.sh" \
        "$sea" "$SEED" "$METHOD" >"$log" 2>&1
    status=$?
    set -e

    if is_complete "$sea"; then
      echo "DONE $sea"
      success=true
      break
    fi
    echo "RETRY $sea: launcher status=$status; partial CSV is preserved." >&2
    sleep 10
  done

  if [[ "$success" != true ]]; then
    echo "STOPPED at $sea after two attempts." >&2
    echo "Run this script again to retry only unfinished sea states." >&2
    exit 1
  fi
  sleep 5
done

rm -f "$ROOT/.v6_suite_current"
touch "$ROOT/.v6_suite_complete"
echo "V6_SEA_SUITE=COMPLETE"

