#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEED="${1:-20261017}"
METHOD="gnn_gru_sac"
SEA_STATES=(medium_wind_wave strong_wind_wave extreme_wind_wave)

mkdir -p "$ROOT/logs"

is_complete() {
  local sea="$1"
  python3 "$ROOT/summarize_calm_horizontal.py" \
    --sea-state "$sea" \
    --seed "$SEED" \
    --minimum-duration 290 \
    --is-finished "$METHOD" >/dev/null 2>&1
}

cleanup_run() {
  bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
}

trap cleanup_run EXIT
trap 'cleanup_run; exit 130' INT TERM

for sea in "${SEA_STATES[@]}"; do
  if is_complete "$sea"; then
    echo "SKIP $sea: a complete locked-V6 result already exists for seed $SEED."
    continue
  fi

  case "$sea" in
    medium_wind_wave|strong_wind_wave)
      run_timeout=390
      ;;
    extreme_wind_wave)
      run_timeout=450
      ;;
  esac

  success=false
  for attempt in 1 2; do
    cleanup_run
    sleep 3
    stamp="$(date +%Y%m%d_%H%M%S)"
    log="$ROOT/logs/v6_paper_${sea}_${SEED}_${stamp}_attempt${attempt}.log"
    printf '%s\n' "$sea" > "$ROOT/.v6_disturbed_current"
    echo "RUN $sea attempt=$attempt/2 -> $log"

    set +e
    timeout --signal=INT --kill-after=25s "${run_timeout}s" \
      env HEADLESS="${HEADLESS:-true}" \
      bash "$ROOT/run_locked_v6.sh" "$sea" "$SEED" >"$log" 2>&1
    status=$?
    set -e

    cleanup_run
    if is_complete "$sea"; then
      echo "DONE $sea"
      success=true
      break
    fi

    echo "RETRY $sea: launcher status=$status; partial CSV is preserved." >&2
    sleep 8
  done

  if [[ "$success" != true ]]; then
    echo "STOPPED at $sea after two attempts." >&2
    echo "Run the start command again to retry only unfinished sea states." >&2
    exit 1
  fi
done

rm -f "$ROOT/.v6_disturbed_current"
touch "$ROOT/.v6_disturbed_complete"
echo "V6_DISTURBED_SEA_SUITE=COMPLETE"
