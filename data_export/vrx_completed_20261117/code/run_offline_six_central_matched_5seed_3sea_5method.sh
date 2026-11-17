#!/usr/bin/env bash
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HARBOR="/home/hzh/Documents/Codex/2026-08-13/hao/vrx_harbor_sac_v6_aligned_ws_v1"
BATCH="$ROOT/results/central_v7_matched_5seed_3sea_5method_20261117"
OLD="$ROOT/results/central_v7_matched_baselines_3seed_3sea_20261117"
ACTOR="$ROOT/install/vrx_csd_shadow/share/vrx_csd_shadow/models/central_sac_v6_selected_50k_actor.json"
mkdir -p "$BATCH/logs"
exec 9>"$BATCH/runner.lock"
flock -n 9 || { echo 'Matched baseline runner already active' >&2; exit 3; }
echo $$ > "$BATCH/runner.pid"
ACTIVE_PARTITION=""
close_partition() {
  [[ -n "$ACTIVE_PARTITION" ]] || return 0
  python3 - "$ACTIVE_PARTITION" <<'PY'
import os,signal,sys,time
from pathlib import Path
target=('GZ_PARTITION='+sys.argv[1]).encode()
def matching():
    found=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit(): continue
        try:
            env=(p/'environ').read_bytes().split(b'\0')
            cmd=(p/'cmdline').read_bytes().replace(b'\0',b' ')
        except (OSError,ProcessLookupError): continue
        if target in env and (b'gz sim' in cmd or b'gz-sim' in cmd): found.append(int(p.name))
    return found
for pid in matching():
    try: os.kill(pid,signal.SIGTERM)
    except ProcessLookupError: pass
for _ in range(50):
    if not matching(): break
    time.sleep(.1)
for pid in matching():
    try: os.kill(pid,signal.SIGKILL)
    except ProcessLookupError: pass
if matching(): raise SystemExit('Gazebo partition remains: '+sys.argv[1])
PY
  ACTIVE_PARTITION=""
}
cleanup() {
  bash "$HARBOR/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
  close_partition || true
  rm -f "$BATCH/current.txt" "$BATCH/runner.pid"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
fail() { printf '%s\n' "$*" | tee "$BATCH/BLOCKED" >&2; exit 4; }
while [[ -f "$OLD/runner.pid" ]]; do sleep 5; done
[[ -s "$ACTOR" ]] || fail "Missing local V6 actor: $ACTOR"
[[ -f "$ROOT/results/central_v7_gnn_gru_sac_3seed_3sea_20261116/FINISHED" ]] || fail 'New-method 9-run batch is incomplete.'
[[ -f "$BATCH/results.tsv" ]] || printf 'sea_state\tseed\tmethod\tstatus\tarrival6\tmission_arrival_s\tobservation_duration_s\tmin_pair_m\toverlap_samples\tmedian_interval_s\trun_dir\n' > "$BATCH/results.tsv"
sha256sum "$ROOT/install/vrx_csd_shadow/lib/vrx_csd_shadow/six_vessel_crossing_driver.py" "$ROOT/install/vrx_csd_shadow/lib/vrx_csd_shadow/csd_shadow_graph.py" "$ROOT/install/vrx_csd_shadow/share/vrx_csd_shadow/models/harbor_gnn_gru_best.json" "$ACTOR" > "$BATCH/code_and_model.sha256"
rm -f "$BATCH/FINISHED" "$BATCH/BLOCKED"
for sea in calm medium_wind_wave strong_wind_wave; do
  for seed in 20261101 20261102 20261110 20261117 20261123; do
    for method in distance_star risk_star conditional_csd learned_gnn_gru_graph_shield gnn_gru_residual_sac; do
      if awk -F '\t' -v s="$sea" -v n="$seed" -v m="$method" 'NR>1 && $1==s && $2==n && $3==m && $4=="VALID" {found=1} END{exit !found}' "$BATCH/results.tsv"; then continue; fi
      [[ -f "$BATCH/STOP_REQUESTED" ]] && { echo 'Stopped by request'; exit 0; }
      cleanup
      sleep 5
      echo $$ > "$BATCH/runner.pid"
      printf '%s\t%s\t%s\n' "$sea" "$seed" "$method" > "$BATCH/current.txt"
      log="$BATCH/logs/${sea}_seed${seed}_${method}_$(date +%Y%m%d_%H%M%S).log"
      partition="central_v7_base_${sea}_${seed}_${method}_$$"
      ACTIVE_PARTITION="$partition"
      domain="$((100 + seed % 100))"
      echo "MATCHED_START sea=$sea seed=$seed method=$method"
      GZ_PARTITION="$partition" ROS_DOMAIN_ID="$domain" HEADLESS=false RESIDUAL_SAC_MODEL_PATH="$ACTOR" \
        MAX_EXPERIMENT_DURATION=300 timeout --signal=INT --kill-after=30s 30m \
        bash "$ROOT/run_six_wamv_crossing_shadow.sh" "$sea" csd_safe central "$method" "$seed" > "$log" 2>&1
      rc=$?
      run_dir="$(sed -n 's/^Results: //p' "$log" | tail -n 1)"
      [[ "$rc" -eq 0 && -n "$run_dir" && -f "$run_dir/six_vessel_driver.csv" ]] || fail "Run failed: sea=$sea seed=$seed method=$method exit=$rc log=$log"
      python3 "$ROOT/validate_six_vessel_run.py" "$run_dir" --encounter central --json-out "$run_dir/start_validation.json" >> "$log" 2>&1 || fail "Invalid trajectory: $run_dir"
      python3 "$ROOT/analyze_six_vessel_comparison.py" "$run_dir" --output-dir "$run_dir/v7_audit" >> "$log" 2>&1 || fail "Audit failed: $run_dir"
      python3 - "$BATCH/results.tsv" "$sea" "$seed" "$method" "$run_dir" <<'PY' || fail "Metric write or cadence check failed: $run_dir"
import csv,statistics,sys
from pathlib import Path
out,sea,seed,method,run_dir=Path(sys.argv[1]),*sys.argv[2:5],Path(sys.argv[5])
with (run_dir/'six_vessel_driver.csv').open(newline='') as f:
    t=[float(r['time']) for r in csv.DictReader(f) if r['vessel']=='wamv_1']
intervals=[b-a for a,b in zip(t,t[1:])]
if not intervals: raise SystemExit('No update intervals')
with (run_dir/'v7_audit/run_summary.csv').open(newline='') as f: m=next(csv.DictReader(f))
median=statistics.median(intervals)
if median>0.9: raise SystemExit(f'Control interval too slow: {median:.3f}s')
with out.open('a',newline='') as f:
    csv.writer(f,delimiter='\t').writerow([sea,seed,method,'VALID',m['arrival_success_count_6m'],m['mission_arrival_time_6m_s'],m['duration_s'],m['minimum_pair_distance_m'],m['overlap_pair_samples_under_5p4m'],f'{median:.3f}',str(run_dir)])
print(f'MATCHED_DONE sea={sea} seed={seed} method={method} arrival={m["arrival_success_count_6m"]}/6 mission={m["mission_arrival_time_6m_s"]}s')
PY
      close_partition || fail "Gazebo partition cleanup failed: $partition"
    done
  done
done
touch "$BATCH/FINISHED"
echo "MATCHED_BASELINES_FINISHED=$BATCH"
