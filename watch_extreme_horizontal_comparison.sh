#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEED="${1:-20261011}"

exec python3 "$ROOT/watch_harbor_horizontal_comparison.py" \
  --sea-state extreme_wind_wave \
  --seed "$SEED" \
  --max-duration "${MAX_EXPERIMENT_DURATION:-300}"

