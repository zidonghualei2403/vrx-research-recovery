#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/hzh/Documents/Codex/2026-08-13/hao/vrx_harbor_sac_v6_aligned_ws_v1"
cd "$ROOT"
bash cleanup_vrx_batch_processes.sh
bash run_medium_rescue_r07.sh r07_primary_seed20261018 20261018
