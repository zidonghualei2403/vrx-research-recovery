#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CANDIDATE="$ROOT/autonomous_medium_iterations_20261018/candidate_119_c117_wamv2_live_spatial_crossing"
VALIDATOR="$CANDIDATE/validate_candidate119_gui.sh"
CAMPAIGN="$ROOT/c119_medium_seed20261117_10repeat_20261102"
RESULTS="$CAMPAIGN/results.tsv"
SEEDS=(20261117)
TARGET_REPEATS=10
EXISTING_RUN="$CANDIDATE/evidence/harbor_gnn_gru_sac_medium_wind_wave_seed20261117_20261102_164446"

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
stall_count = len(m.get("distant_stalls_20s", []))
safety_pass = (
    bool(m.get("six_vessels_valid"))
    and m.get("arrival_count") == 4
    and m.get("vessel_overlap_frames") == 0
    and m.get("wall_overlap_frames") == 0
    and stall_count == 0
)
strict_pass = safety_pass and fleet_ok
status = "PASS" if strict_pass else ("SAFE_SLOW" if safety_pass else "FAIL")

row = [
    seed,
    str(repeat),
    source,
    status,
    str(m.get("arrival_count", "")),
    "" if fleet_time is None else str(fleet_time),
    str(m.get("vessel_overlap_frames", "")),
    str(m.get("wall_overlap_frames", "")),
    str(m.get("minimum_vessel_clearance_m", "")),
    str(m.get("minimum_wall_clearance_m", "")),
    str(stall_count),
    str(run_dir),
]
with results.open("a") as handle:
    handle.write("\t".join(row) + "\n")
PY
}

if [[ ! -s "$RESULTS" ]]; then
  printf '%s\n' $'seed\trepeat\tsource\tstatus\tarrival\tfleet_time_s\tship_overlap\twall_overlap\tmin_ship_gap_m\tmin_wall_gap_m\tstall20_count\trun_dir' > "$RESULTS"
  append_result 20261117 1 existing_c119_gui_pass "$EXISTING_RUN"
fi

has_repeat() {
  local seed="$1"
  local repeat="$2"
  awk -F '\t' -v s="$seed" -v r="$repeat" 'NR > 1 && $1 == s && $2 == r {found=1} END {exit !found}' "$RESULTS"
}

echo "C119 medium-wind/wave replacement-seed campaign"
echo "Seeds: ${SEEDS[*]}"
echo "Target: ${TARGET_REPEATS} C119 runs; the audited GUI pass is repeat 1."
echo "Every new behavioral result is retained, including failures."
echo "Results: $RESULTS"

for ((repeat=2; repeat<=TARGET_REPEATS; repeat++)); do
  for seed in "${SEEDS[@]}"; do
    if has_repeat "$seed" "$repeat"; then
      echo "SKIP seed=$seed repeat=$repeat (already recorded)"
      continue
    fi

    echo
    echo "[$seed repeat $repeat/$TARGET_REPEATS] Starting original C119 GUI validation"
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

    append_result "$seed" "$repeat" c119_confirmatory_run "$run_dir"
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

print("\nC119 medium seed 20261117 x 10-repeat summary")
print("seed       <=130 pass  safe/total  safe times (mean, range)")
for seed in sorted(groups):
    group = groups[seed]
    passed = [r for r in group if r["status"] == "PASS"]
    safe = [r for r in group if r["status"] in ("PASS", "SAFE_SLOW")]
    times = [float(r["fleet_time_s"]) for r in safe if r["fleet_time_s"]]
    if times:
        timing = f"{statistics.mean(times):.2f}s ({min(times):.2f}-{max(times):.2f})"
    else:
        timing = "NA"
    print(f"{seed}  {len(passed)}/{len(group)}       {len(safe)}/{len(group)}       {timing}")
PY

echo "CAMPAIGN_COMPLETE=$CAMPAIGN"
