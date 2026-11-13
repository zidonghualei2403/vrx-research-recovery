#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEA_STATE="${1:-calm}"
SEED="${2:-20261011}"
# Reciprocal shuttle traffic creates two legitimate waiting windows for some
# routes. 120 s can stop a still-converging vessel during the second window.
MAX_DURATION="${MAX_EXPERIMENT_DURATION:-240.0}"
mkdir -p "$ROOT/logs" "$ROOT/results"

latest_result() {
  find "$ROOT/results" -maxdepth 1 -type d \
    -name "harbor_gnn_gru_${SEA_STATE}_seed${SEED}_*" \
    -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-
}

before="$(latest_result || true)"

echo "Validating the newly exported shuttle GNN-GRU in VRX."
echo "sea_state=$SEA_STATE seed=$SEED duration=$MAX_DURATION"
latest=""
csv=""
for attempt in 1 2; do
  echo "VRX startup attempt $attempt/2"
  if ! HEADLESS=true MAX_EXPERIMENT_DURATION="$MAX_DURATION" \
      bash "$ROOT/run_harbor_gnn_gru_vrx.sh" "$SEA_STATE" "$SEED" gnn_gru; then
    echo "VRX launch returned an error on attempt $attempt." >&2
  fi

  latest="$(latest_result || true)"
  csv="$latest/six_vessel_driver.csv"
  rows=0
  if [[ -f "$csv" ]]; then
    rows="$(wc -l < "$csv")"
  fi
  if [[ -n "$latest" && "$latest" != "$before" && "$rows" -gt 1 ]]; then
    break
  fi

  echo "Attempt $attempt produced no control samples (GPS/IMU startup miss)."
  if [[ "$attempt" -lt 2 ]]; then
    echo "Waiting 8 s, then retrying from a fresh VRX launch..."
    sleep 8
    before="$latest"
  fi
done

if [[ -z "$latest" || ! -f "$csv" || "$rows" -le 1 ]]; then
  echo "VRX sensor startup failed twice; no algorithm result was recorded." >&2
  exit 3
fi

python3 "$ROOT/analyze_shuttle_validation.py" "$csv" --require-navigation-success
