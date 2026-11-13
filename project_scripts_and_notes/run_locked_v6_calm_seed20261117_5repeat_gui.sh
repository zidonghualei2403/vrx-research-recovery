#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CAMPAIGN_OVERRIDE="$ROOT/locked_v6_calm_seed20261117_5repeat_20261102" \
SEEDS_OVERRIDE="20261117" \
TARGET_REPEATS_OVERRIDE=5 \
  bash "$ROOT/run_locked_v6_calm_5seed_10repeat_gui.sh"
