#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CANDIDATE="$ROOT/autonomous_medium_iterations_20261018/candidate_15_local_wall_normal_nudge"
ITER_NAME="${1:-iter_15_local_wall_normal_nudge}"
ITER_DIR="$ROOT/autonomous_medium_iterations_20261018/$ITER_NAME"
SEED="${2:-20261018}"
HEARTBEAT="$ITER_DIR/heartbeat.txt"
LAUNCH_LOG="$ITER_DIR/launch.log"

if [[ -e "$ITER_DIR/RUN_FINISHED" || -e "$ITER_DIR/RUN_FAILED" ]]; then
  echo "Immutable iteration already exists: $ITER_DIR" >&2
  exit 4
fi
mkdir -p "$ITER_DIR/source_snapshot" "$ITER_DIR/raw" "$ITER_DIR/ros_logs" "$ITER_DIR/gz_home"
cp -f "$CANDIDATE/ws/src/vrx_csd_shadow/scripts/six_vessel_crossing_driver.py" \
  "$ITER_DIR/source_snapshot/six_vessel_crossing_driver.py"
cp -f "$CANDIDATE/build.log" "$ITER_DIR/build.log"
diff -u \
  "$ROOT/autonomous_medium_iterations_20261018/candidate_05_static_wall_dynamics/ws/src/vrx_csd_shadow/scripts/six_vessel_crossing_driver.py" \
  "$CANDIDATE/ws/src/vrx_csd_shadow/scripts/six_vessel_crossing_driver.py" \
  > "$ITER_DIR/source.diff" || true
sha256sum \
  "$CANDIDATE/ws/install/vrx_csd_shadow/lib/vrx_csd_shadow/six_vessel_crossing_driver.py" \
  "$CANDIDATE/ws/install/vrx_csd_shadow/share/vrx_csd_shadow/models/harbor_residual_sac_actor.json" \
  "$CANDIDATE/ws/install/vrx_csd_shadow/share/vrx_csd_shadow/models/harbor_gnn_gru_best.json" \
  > "$ITER_DIR/checksums.sha256"

cleanup() { rm -f "$HEARTBEAT"; }
trap cleanup EXIT

HOME="$ITER_DIR/gz_home" ROS_LOG_DIR="$ITER_DIR/ros_logs" \
HEADLESS=true bash "$CANDIDATE/run_candidate_stable_shuttles.sh" \
  medium_wind_wave "$SEED" > "$LAUNCH_LOG" 2>&1 &
child=$!
echo "$child" > "$ITER_DIR/child.pid"
while kill -0 "$child" 2>/dev/null; do
  printf '%s,candidate_15_local_wall_normal_nudge,medium_wind_wave,%s\n' \
    "$(date +%s)" "$child" > "$HEARTBEAT"
  sleep 5
done

set +e
wait "$child"
status=$?
set -e
echo "$status" > "$ITER_DIR/exit_status.txt"
run_dir="$(sed -n 's/^Results: //p' "$LAUNCH_LOG" | tail -1)"
printf '%s\n' "$run_dir" > "$ITER_DIR/run_dir.txt"
if [[ -n "$run_dir" && -d "$run_dir" ]]; then
  for name in six_vessel_driver.csv experiment_config.csv csd_shadow_graph.csv; do
    [[ -f "$run_dir/$name" ]] && cp -f "$run_dir/$name" "$ITER_DIR/raw/$name"
  done
fi

if [[ "$status" -eq 0 && -s "$ITER_DIR/raw/six_vessel_driver.csv" ]]; then
  python3 "$ROOT/audit_autonomous_run.py" \
    "$ITER_DIR/raw/six_vessel_driver.csv" --output-dir "$ITER_DIR"
  touch "$ITER_DIR/RUN_FINISHED"
else
  touch "$ITER_DIR/RUN_FAILED"
fi
exit "$status"
