#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
METHOD="${1:-distance_star}"
SEED="${2:-20261123}"
DURATION="${3:-180}"

case "$METHOD" in
  distance_star|risk_star|conditional_csd|gnn_gru) ;;
  *)
    echo "Method must be distance_star, risk_star, conditional_csd, or gnn_gru" >&2
    exit 2
    ;;
esac

echo "Manual medium-sea baseline validation"
echo "method=$METHOD seed=$SEED duration=${DURATION}s"
echo "wamv_5/6 use the same stable-shuttle setting as the accepted experiments."

bash "$ROOT/cleanup_vrx_batch_processes.sh"

set +e
STABILIZE_SHUTTLES=true \
HEADLESS=false \
MAX_EXPERIMENT_DURATION="$DURATION" \
  bash "$ROOT/run_harbor_gnn_gru_vrx.sh" \
    medium_wind_wave "$SEED" "$METHOD"
launcher_status=$?
set -e

run_dir="$(find "$ROOT/results" -mindepth 1 -maxdepth 1 -type d \
  -name "harbor_${METHOD}_medium_wind_wave_seed${SEED}_*" \
  -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)"

if [[ -z "$run_dir" || ! -s "$run_dir/six_vessel_driver.csv" ]]; then
  echo "No valid trajectory CSV was produced (launcher status=$launcher_status)." >&2
  exit 1
fi

python3 "$ROOT/audit_autonomous_run.py" \
  "$run_dir/six_vessel_driver.csv" \
  --output-dir "$run_dir/body_audit" \
  --fleet-time-limit "$DURATION"

echo "BASELINE_RESULT=$run_dir"
