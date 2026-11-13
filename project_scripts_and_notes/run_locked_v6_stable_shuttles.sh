#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "LOCKED V6 + sea-state-invariant noncooperative shuttle routes"
echo "wamv_5/6: no direct wind/current forcing; patrol feedback remains active."

STABILIZE_SHUTTLES=true \
  bash "$ROOT/run_locked_v6.sh" "${1:-calm}" "${2:-20261014}"
