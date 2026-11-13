#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CANDIDATE="$ROOT/autonomous_medium_rescue_20261019/candidate_r07_c05_wall_padding"
RUN_NAME="${1:-r07_primary_seed20261018}"
SEED="${2:-20261018}"
RUN_DIR="$ROOT/autonomous_medium_rescue_20261019/$RUN_NAME"

if [[ -e "$RUN_DIR/RUN_FINISHED" || -e "$RUN_DIR/RUN_FAILED" ]]; then
  echo "Immutable rescue run already exists: $RUN_DIR" >&2
  exit 4
fi
mkdir -p "$RUN_DIR/source_snapshot" "$RUN_DIR/raw" "$RUN_DIR/ros_logs" "$RUN_DIR/gz_home"
cp -f "$CANDIDATE/ws/src/vrx_csd_shadow/scripts/six_vessel_crossing_driver.py" \
  "$RUN_DIR/source_snapshot/six_vessel_crossing_driver.py"
cp -f "$CANDIDATE/build.log" "$RUN_DIR/build.log"
diff -u \
  "$ROOT/autonomous_medium_iterations_20261018/candidate_05_static_wall_dynamics/ws/src/vrx_csd_shadow/scripts/six_vessel_crossing_driver.py" \
  "$CANDIDATE/ws/src/vrx_csd_shadow/scripts/six_vessel_crossing_driver.py" \
  > "$RUN_DIR/source.diff" || true
sha256sum \
  "$CANDIDATE/ws/install/vrx_csd_shadow/lib/vrx_csd_shadow/six_vessel_crossing_driver.py" \
  "$CANDIDATE/ws/install/vrx_csd_shadow/share/vrx_csd_shadow/models/harbor_residual_sac_actor.json" \
  "$CANDIDATE/ws/install/vrx_csd_shadow/share/vrx_csd_shadow/models/harbor_gnn_gru_best.json" \
  > "$RUN_DIR/checksums.sha256"

HOME="$RUN_DIR/gz_home" ROS_LOG_DIR="$RUN_DIR/ros_logs" HEADLESS=true \
  bash "$CANDIDATE/run_candidate_stable_shuttles.sh" \
  medium_wind_wave "$SEED" > "$RUN_DIR/launch.log" 2>&1 &
child=$!
echo "$child" > "$RUN_DIR/child.pid"
while kill -0 "$child" 2>/dev/null; do
  printf '%s,r07,medium_wind_wave,%s\n' "$(date +%s)" "$child" \
    > "$RUN_DIR/heartbeat.txt"
  sleep 5
done

set +e
wait "$child"
status=$?
set -e
rm -f "$RUN_DIR/heartbeat.txt"
echo "$status" > "$RUN_DIR/exit_status.txt"
result_dir="$(sed -n 's/^Results: //p' "$RUN_DIR/launch.log" | tail -1)"
printf '%s\n' "$result_dir" > "$RUN_DIR/result_dir.txt"
if [[ -n "$result_dir" && -d "$result_dir" ]]; then
  for name in six_vessel_driver.csv experiment_config.csv csd_shadow_graph.csv; do
    [[ -f "$result_dir/$name" ]] && cp -f "$result_dir/$name" "$RUN_DIR/raw/$name"
  done
fi
if [[ "$status" -eq 0 && -s "$RUN_DIR/raw/six_vessel_driver.csv" ]]; then
  python3 "$ROOT/audit_autonomous_run.py" \
    "$RUN_DIR/raw/six_vessel_driver.csv" --output-dir "$RUN_DIR"
  touch "$RUN_DIR/RUN_FINISHED"
else
  touch "$RUN_DIR/RUN_FAILED"
fi
exit "$status"
