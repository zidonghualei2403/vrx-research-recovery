#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BATCH="$ROOT/autonomous_medium_iterations_20261018/candidate_64_c60_wamv1_post_conflict_recovery/evidence/batch_calm_medium_3seed_gui_v1"
mkdir -p "$BATCH"
touch "$BATCH/STOP_REQUESTED"
echo "Stop requested. The active cell will be closed at the next boundary; completed cells remain saved."
