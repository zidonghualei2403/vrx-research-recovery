#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAMPAIGN="${CAMPAIGN_OVERRIDE:-$ROOT/locked_v6_calm_5seed_10repeat_20261102}"
RESULTS="$CAMPAIGN/results.tsv"
read -r -a SEEDS <<< "${SEEDS_OVERRIDE:-20261101 20261102 20261110 20261117 20261123}"
TARGET_REPEATS="${TARGET_REPEATS_OVERRIDE:-10}"
SEA_STATE=calm

mkdir -p "$CAMPAIGN/logs" "$CAMPAIGN/manifests"

cleanup() {
  bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
}
trap 'cleanup; exit 130' INT TERM

if [[ ! -s "$RESULTS" ]]; then
  printf '%s\n' $'seed\trepeat\tstatus\tarrival\tfleet_time_s\tship_overlap\twall_overlap\tmin_ship_gap_m\tmin_wall_gap_m\tstall20_count\tunder_115_2s\tunder_130s\trun_dir' > "$RESULTS"
fi

has_repeat() {
  local seed="$1"
  local repeat="$2"
  awk -F '\t' -v s="$seed" -v r="$repeat" \
    'NR > 1 && $1 == s && $2 == r {found=1} END {exit !found}' "$RESULTS"
}

append_result() {
  local seed="$1"
  local repeat="$2"
  local run_dir="$3"
  python3 - "$RESULTS" "$seed" "$repeat" "$run_dir" <<'PY'
import json
import math
import sys
from pathlib import Path

results = Path(sys.argv[1])
seed = sys.argv[2]
repeat = int(sys.argv[3])
run_dir = Path(sys.argv[4]).resolve()
metrics_path = run_dir / "body_audit" / "metrics.json"

if not metrics_path.is_file():
    raise SystemExit(f"Missing audit metrics: {metrics_path}")

m = json.loads(metrics_path.read_text())
fleet_time = m.get("fleet_completion_s")
finite_time = isinstance(fleet_time, (int, float)) and math.isfinite(fleet_time)
stall_count = len(m.get("distant_stalls_20s", []))
safety_pass = (
    bool(m.get("six_vessels_valid"))
    and m.get("arrival_count") == 4
    and m.get("vessel_overlap_frames") == 0
    and m.get("wall_overlap_frames") == 0
    and stall_count == 0
)
under_115 = finite_time and fleet_time <= 115.2
under_130 = finite_time and fleet_time <= 130.0
status = "PASS" if safety_pass and under_130 else ("SAFE_SLOW" if safety_pass else "FAIL")

row = [
    seed,
    str(repeat),
    status,
    str(m.get("arrival_count", "")),
    "" if fleet_time is None else str(fleet_time),
    str(m.get("vessel_overlap_frames", "")),
    str(m.get("wall_overlap_frames", "")),
    str(m.get("minimum_vessel_clearance_m", "")),
    str(m.get("minimum_wall_clearance_m", "")),
    str(stall_count),
    str(int(under_115)),
    str(int(under_130)),
    str(run_dir),
]
with results.open("a") as handle:
    handle.write("\t".join(row) + "\n")
PY
}

echo "Locked V6 calm-water confirmatory campaign"
echo "Seeds: ${SEEDS[*]}"
echo "Target: $TARGET_REPEATS fresh GUI runs per seed ($(( ${#SEEDS[@]} * TARGET_REPEATS )) runs total)."
echo "Controller and models: checksum-protected install_v6; no candidate code is used."
echo "Noncooperative shuttles: stable routes, no direct wind/current forcing."
echo "Results: $RESULTS"

for ((repeat=1; repeat<=TARGET_REPEATS; repeat++)); do
  for seed in "${SEEDS[@]}"; do
    if has_repeat "$seed" "$repeat"; then
      echo "SKIP seed=$seed repeat=$repeat (already recorded)"
      continue
    fi

    echo
    echo "[$seed repeat $repeat/$TARGET_REPEATS] Starting locked V6 calm GUI validation"
    stamp="$(date +%Y%m%d_%H%M%S)"
    log="$CAMPAIGN/logs/seed${seed}_repeat$(printf '%02d' "$repeat")_${stamp}.log"
    manifest="$CAMPAIGN/manifests/seed${seed}_repeat$(printf '%02d' "$repeat")_${stamp}.sha256"
    prefix="harbor_gnn_gru_sac_${SEA_STATE}_seed${seed}_"
    before="$(mktemp)"
    after="$(mktemp)"

    find "$ROOT/results" -mindepth 1 -maxdepth 1 -type d -name "${prefix}*" \
      -printf '%f\n' | sort > "$before"

    sha256sum \
      "$ROOT/install_v6/vrx_csd_shadow/lib/vrx_csd_shadow/six_vessel_crossing_driver.py" \
      "$ROOT/install_v6/vrx_csd_shadow/share/vrx_csd_shadow/models/harbor_residual_sac_actor.json" \
      "$ROOT/install_v6/vrx_csd_shadow/share/vrx_csd_shadow/models/harbor_gnn_gru_best.json" \
      > "$manifest"

    cleanup
    set +e
    HEADLESS=false bash "$ROOT/run_locked_v6_stable_shuttles.sh" "$SEA_STATE" "$seed" \
      2>&1 | tee "$log"
    launch_status=${PIPESTATUS[0]}
    set -e

    find "$ROOT/results" -mindepth 1 -maxdepth 1 -type d -name "${prefix}*" \
      -printf '%f\n' | sort > "$after"
    new_name="$(comm -13 "$before" "$after" | tail -1)"
    rm -f "$before" "$after"

    if [[ "$launch_status" -ne 0 || -z "$new_name" ]]; then
      echo "Launch/infrastructure failure for seed=$seed repeat=$repeat (exit=$launch_status)."
      echo "This attempt is not counted. Rerun this script after fixing the launch issue."
      cleanup
      if [[ "$launch_status" -ne 0 ]]; then
        exit "$launch_status"
      fi
      exit 4
    fi

    run_dir="$ROOT/results/$new_name"
    if [[ ! -s "$run_dir/six_vessel_driver.csv" ]]; then
      echo "No valid trajectory CSV was produced; this attempt is not counted."
      cleanup
      exit 4
    fi

    python3 "$ROOT/audit_autonomous_run.py" \
      "$run_dir/six_vessel_driver.csv" \
      --output-dir "$run_dir/body_audit" \
      --fleet-time-limit 130.0

    cp -f "$manifest" "$run_dir/locked_v6_checksums.sha256"
    printf 'HEADLESS=false bash %q calm %q\n' \
      "$ROOT/run_locked_v6_stable_shuttles.sh" "$seed" > "$run_dir/reproduction_command.txt"

    append_result "$seed" "$repeat" "$run_dir"
    tail -n 1 "$RESULTS"
    cleanup
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

print("\nLocked V6 calm repeat summary")
print("seed       <=115.2  <=130  safe/total  safe times (mean, range)")
for seed in sorted(groups):
    group = groups[seed]
    safe = [r for r in group if r["status"] in ("PASS", "SAFE_SLOW")]
    under_115 = sum(r["under_115_2s"] == "1" and r["status"] != "FAIL" for r in group)
    under_130 = sum(r["under_130s"] == "1" and r["status"] != "FAIL" for r in group)
    times = [float(r["fleet_time_s"]) for r in safe if r["fleet_time_s"]]
    timing = (
        f"{statistics.mean(times):.2f}s ({min(times):.2f}-{max(times):.2f})"
        if times else "NA"
    )
    print(
        f"{seed}  {under_115}/{len(group)}     {under_130}/{len(group)}   "
        f"{len(safe)}/{len(group)}       {timing}"
    )
PY

echo "CAMPAIGN_COMPLETE=$CAMPAIGN"
