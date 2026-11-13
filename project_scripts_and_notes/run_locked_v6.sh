#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEA_STATE="${1:-calm}"
SEED="${2:-20261014}"

DRIVER="$ROOT/install_v6/vrx_csd_shadow/lib/vrx_csd_shadow/six_vessel_crossing_driver.py"
SAC_MODEL="$ROOT/install_v6/vrx_csd_shadow/share/vrx_csd_shadow/models/harbor_residual_sac_actor.json"
GNN_MODEL="$ROOT/install_v6/vrx_csd_shadow/share/vrx_csd_shadow/models/harbor_gnn_gru_best.json"

verify_file() {
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "REFUSING TO RUN: locked V6 artifact changed: $path" >&2
    echo "Expected $expected, got $actual" >&2
    exit 3
  fi
}

verify_file e4dfd79a85e94d899f35786487888f16732472037a0cb4e604843842555c25c9 "$DRIVER"
verify_file 82d19bbf6b63dd1695eec438ab3e368c4958faf4ac6fd2cd85bca18263357148 "$SAC_MODEL"
verify_file 862294f60ad23bb2b4ea76d181e664bc585357b8507c4dcce31d077e4422ec92 "$GNN_MODEL"

case "$SEA_STATE" in
  calm) MAX_DURATION=240 ;;
  medium_wind_wave|strong_wind_wave) MAX_DURATION=300 ;;
  extreme_wind_wave) MAX_DURATION=360 ;;
  *)
    echo "Usage: $0 calm|medium_wind_wave|strong_wind_wave|extreme_wind_wave [seed]" >&2
    exit 2
    ;;
esac

echo "LOCKED METHOD: verified Harbor V6 (GNN-GRU + residual SAC)"
echo "Sea state: $SEA_STATE; seed: $SEED"
echo "V7/V8 safety-gate experiments are intentionally not loaded."

MAX_EXPERIMENT_DURATION="$MAX_DURATION" \
RESIDUAL_SAC_AUTHORITY=0.35 \
HEADLESS="${HEADLESS:-false}" \
bash "$ROOT/run_harbor_gnn_gru_vrx.sh" \
  "$SEA_STATE" "$SEED" gnn_gru_sac
