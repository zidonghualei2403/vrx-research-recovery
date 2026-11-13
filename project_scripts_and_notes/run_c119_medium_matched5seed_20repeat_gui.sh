#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CANDIDATE="$ROOT/autonomous_medium_iterations_20261018/candidate_119_c117_wamv2_live_spatial_crossing"
VALIDATOR="$CANDIDATE/validate_candidate119_gui.sh"
CAMPAIGN="$ROOT/c119_medium_matched5seed_20repeat_20261110"
RESULTS="$CAMPAIGN/results.tsv"
CURRENT="$CAMPAIGN/current.txt"
STOP_REQUEST="$CAMPAIGN/STOP_REQUESTED"
PID_FILE="$CAMPAIGN/runner.pid"
SEEDS=(20261101 20261102 20261110 20261117 20261123)
TARGET_REPEATS=20

mkdir -p "$CAMPAIGN/logs"

cleanup() {
  bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
  rm -f "$CURRENT" "$PID_FILE"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

append_result() {
  local seed="$1" repeat="$2" source="$3" run_dir="$4"
  python3 - "$RESULTS" "$seed" "$repeat" "$source" "$run_dir" <<'PY'
import json
import math
import sys
from pathlib import Path

results = Path(sys.argv[1])
seed = sys.argv[2]
repeat = int(sys.argv[3])
source = sys.argv[4]
run_dir = Path(sys.argv[5]).resolve()
metrics_path = run_dir / "body_audit" / "metrics.json"
if not metrics_path.is_file():
    raise SystemExit(f"Missing audit metrics: {metrics_path}")

m = json.loads(metrics_path.read_text())
fleet_time = m.get("fleet_completion_s")
fleet_ok = isinstance(fleet_time, (int, float)) and math.isfinite(fleet_time) and fleet_time <= 130.0
stall_count = len(m.get("distant_stalls_20s", []))
safety_pass = (
    bool(m.get("six_vessels_valid"))
    and m.get("arrival_count") == 4
    and m.get("vessel_overlap_frames") == 0
    and m.get("wall_overlap_frames") == 0
    and stall_count == 0
)
status = "PASS" if safety_pass and fleet_ok else ("SAFE_SLOW" if safety_pass else "FAIL")
row = [
    seed, str(repeat), source, status,
    str(m.get("arrival_count", "")),
    "" if fleet_time is None else str(fleet_time),
    str(m.get("vessel_overlap_frames", "")),
    str(m.get("wall_overlap_frames", "")),
    str(m.get("minimum_vessel_clearance_m", "")),
    str(m.get("minimum_wall_clearance_m", "")),
    str(stall_count), str(run_dir),
]
with results.open("a") as handle:
    handle.write("\t".join(row) + "\n")
PY
}

bootstrap_existing() {
  python3 - "$RESULTS" \
    "$ROOT/c119_medium_5seed_10repeat_20261102/results.tsv" \
    "$ROOT/c119_medium_seed20261117_10repeat_20261102/results.tsv" <<'PY'
import csv
import sys
from pathlib import Path

dest = Path(sys.argv[1])
sources = [Path(p) for p in sys.argv[2:]]
wanted = {"20261101", "20261102", "20261110", "20261117", "20261123"}
rows = []
for path in sources:
    if not path.is_file():
        raise SystemExit(f"Missing historical C119 results: {path}")
    with path.open() as handle:
        for row in csv.DictReader(handle, delimiter="\t"):
            if row["seed"] in wanted and 1 <= int(row["repeat"]) <= 10:
                rows.append(row)

keyed = {(row["seed"], int(row["repeat"])): row for row in rows}
missing = [(seed, repeat) for seed in sorted(wanted) for repeat in range(1, 11)
           if (seed, repeat) not in keyed]
if missing:
    raise SystemExit(f"Historical C119 bootstrap is incomplete: {missing}")

fields = ["seed", "repeat", "source", "status", "arrival", "fleet_time_s",
          "ship_overlap", "wall_overlap", "min_ship_gap_m", "min_wall_gap_m",
          "stall20_count", "run_dir"]
with dest.open("w", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=fields, delimiter="\t")
    writer.writeheader()
    for seed in sorted(wanted):
        for repeat in range(1, 11):
            row = keyed[(seed, repeat)]
            row["source"] = "existing_c119_confirmatory"
            writer.writerow({field: row.get(field, "") for field in fields})
PY
}

if [[ ! -s "$RESULTS" ]]; then
  bootstrap_existing
fi

has_repeat() {
  local seed="$1" repeat="$2"
  awk -F '\t' -v s="$seed" -v r="$repeat" \
    'NR > 1 && $1 == s && $2 == r {found=1} END {exit !found}' "$RESULTS"
}

rm -f "$STOP_REQUEST" "$CAMPAIGN/CAMPAIGN_FINISHED"
echo "$$" > "$PID_FILE"
echo "C119 medium matched campaign: 5 seeds x 20 repeats"
echo "Seeds: ${SEEDS[*]}"
echo "The first 10 repeats per seed are frozen historical C119 runs."
echo "Only repeats 11-20 will be launched. Ctrl+C stops safely."

for ((repeat=11; repeat<=TARGET_REPEATS; repeat++)); do
  for seed in "${SEEDS[@]}"; do
    if [[ -f "$STOP_REQUEST" ]]; then
      echo "Stop requested. Completed C119 runs are preserved."
      exit 0
    fi
    if has_repeat "$seed" "$repeat"; then
      continue
    fi

    printf 'medium_wind_wave,c119,%s,%s\n' "$seed" "$repeat" > "$CURRENT"
    echo "[$seed repeat $repeat/$TARGET_REPEATS] Starting C119 GUI validation"
    log="$CAMPAIGN/logs/seed${seed}_repeat$(printf '%02d' "$repeat")_$(date +%Y%m%d_%H%M%S).log"

    set +e
    bash "$VALIDATOR" medium_wind_wave "$seed" 180 2>&1 | tee "$log"
    status=${PIPESTATUS[0]}
    set -e

    if [[ "$status" -ne 0 ]]; then
      echo "Infrastructure/launch failure (exit=$status); this attempt is not counted."
      exit "$status"
    fi
    run_dir="$(cat "$CANDIDATE/latest_gui_run.txt" 2>/dev/null || true)"
    if [[ -z "$run_dir" || ! -s "$run_dir/body_audit/metrics.json" ]]; then
      echo "No valid audited run was produced; this attempt is not counted."
      exit 4
    fi
    append_result "$seed" "$repeat" c119_matched_confirmatory "$run_dir"
    tail -n 1 "$RESULTS"
    sleep 3
  done
done

python3 "$ROOT/summarize_c119_medium_matched.py" "$RESULTS"
touch "$CAMPAIGN/CAMPAIGN_FINISHED"
echo "C119_MEDIUM_MATCHED_FINISHED=$CAMPAIGN"
