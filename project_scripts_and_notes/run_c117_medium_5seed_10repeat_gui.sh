#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CANDIDATE="$ROOT/autonomous_medium_iterations_20261018/candidate_117_c116_wamv1_lower_wall_margin"
VALIDATOR="$CANDIDATE/validate_candidate117_gui.sh"
CAMPAIGN="$ROOT/c117_medium_5seed_10repeat_20261102"
RESULTS="$CAMPAIGN/results.tsv"
SEEDS=(20261101 20261102 20261110 20261121 20261123)
TARGET_REPEATS=10

declare -A BASELINE_RUNS=(
  [20261101]="$CANDIDATE/evidence/harbor_gnn_gru_sac_medium_wind_wave_seed20261101_20261102_094402"
  [20261102]="$CANDIDATE/evidence/harbor_gnn_gru_sac_medium_wind_wave_seed20261102_20261102_095630"
  [20261110]="$CANDIDATE/evidence/harbor_gnn_gru_sac_medium_wind_wave_seed20261110_20261102_105823"
  [20261121]="$CANDIDATE/evidence/harbor_gnn_gru_sac_medium_wind_wave_seed20261121_20261102_100714"
  [20261123]="$CANDIDATE/evidence/harbor_gnn_gru_sac_medium_wind_wave_seed20261123_20261102_093918"
)

mkdir -p "$CAMPAIGN/logs"

cleanup() {
  bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
}
trap 'cleanup; exit 130' INT TERM

append_result() {
  local seed="$1"
  local repeat="$2"
  local source="$3"
  local run_dir="$4"
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
strict_pass = (
    bool(m.get("six_vessels_valid"))
    and m.get("arrival_count") == 4
    and m.get("vessel_overlap_frames") == 0
    and m.get("wall_overlap_frames") == 0
    and fleet_ok
)

row = [
    seed,
    str(repeat),
    source,
    "PASS" if strict_pass else "FAIL",
    str(m.get("arrival_count", "")),
    "" if fleet_time is None else str(fleet_time),
    str(m.get("vessel_overlap_frames", "")),
    str(m.get("wall_overlap_frames", "")),
    str(m.get("minimum_vessel_clearance_m", "")),
    str(m.get("minimum_wall_clearance_m", "")),
    str(len(m.get("distant_stalls_20s", []))),
    str(run_dir),
]
with results.open("a") as handle:
    handle.write("\t".join(row) + "\n")
PY
}

if [[ ! -s "$RESULTS" ]]; then
  printf '%s\n' $'seed\trepeat\tsource\tstatus\tarrival\tfleet_time_s\tship_overlap\twall_overlap\tmin_ship_gap_m\tmin_wall_gap_m\tstall20_count\trun_dir' > "$RESULTS"
  for seed in "${SEEDS[@]}"; do
    append_result "$seed" 1 existing_strict_pass "${BASELINE_RUNS[$seed]}"
  done
fi

has_repeat() {
  local seed="$1"
  local repeat="$2"
  awk -F '\t' -v s="$seed" -v r="$repeat" 'NR > 1 && $1 == s && $2 == r {found=1} END {exit !found}' "$RESULTS"
}

echo "C117 medium-wind/wave confirmatory campaign"
echo "Seeds: ${SEEDS[*]}"
echo "Target: ${TARGET_REPEATS} counted runs per seed; repeat 1 is the frozen existing pass."
echo "Every new behavioral result is retained, including failures."
echo "Results: $RESULTS"

for ((repeat=2; repeat<=TARGET_REPEATS; repeat++)); do
  for seed in "${SEEDS[@]}"; do
    if has_repeat "$seed" "$repeat"; then
      echo "SKIP seed=$seed repeat=$repeat (already recorded)"
      continue
    fi

    echo
    echo "[$seed repeat $repeat/$TARGET_REPEATS] Starting original C117 GUI validation"
    log="$CAMPAIGN/logs/seed${seed}_repeat$(printf '%02d' "$repeat")_$(date +%Y%m%d_%H%M%S).log"

    set +e
    bash "$VALIDATOR" medium_wind_wave "$seed" 180 2>&1 | tee "$log"
    status=${PIPESTATUS[0]}
    set -e

    if [[ "$status" -ne 0 ]]; then
      echo "Infrastructure/launch failure for seed=$seed repeat=$repeat (exit=$status)."
      echo "This attempt is not counted. Fix the launch issue and rerun this script to resume."
      cleanup
      exit "$status"
    fi

    run_dir="$(cat "$CANDIDATE/latest_gui_run.txt" 2>/dev/null || true)"
    if [[ -z "$run_dir" || ! -s "$run_dir/body_audit/metrics.json" ]]; then
      echo "No valid audited run was produced; this attempt is not counted."
      cleanup
      exit 4
    fi

    append_result "$seed" "$repeat" new_confirmatory_run "$run_dir"
    tail -n 1 "$RESULTS"
    sleep 3
  done
done

cleanup

python3 - "$RESULTS" <<'PY'
import csv
import statistics
import sys
from collections import defaultdict

rows = list(csv.DictReader(open(sys.argv[1]), delimiter="\t"))
groups = defaultdict(list)
for row in rows:
    groups[row["seed"]].append(row)

print("\nC117 medium 5-seed x 10-repeat summary")
print("seed       pass/total  safe-arrival times (mean, range)")
for seed in sorted(groups):
    group = groups[seed]
    passed = [r for r in group if r["status"] == "PASS"]
    times = [float(r["fleet_time_s"]) for r in passed if r["fleet_time_s"]]
    if times:
        timing = f"{statistics.mean(times):.2f}s ({min(times):.2f}-{max(times):.2f})"
    else:
        timing = "NA"
    print(f"{seed}  {len(passed)}/{len(group)}       {timing}")
PY

echo "CAMPAIGN_COMPLETE=$CAMPAIGN"
