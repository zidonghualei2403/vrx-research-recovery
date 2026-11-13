#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ITER_NAME="${1:-iter_00}"
ITER_DIR="$ROOT/autonomous_medium_iterations_20261018/$ITER_NAME"
HEARTBEAT="$ITER_DIR/heartbeat.txt"
LAUNCH_LOG="$ITER_DIR/launch.log"
SEED=20261018

mkdir -p "$ITER_DIR/source_snapshot" "$ITER_DIR/raw"
mkdir -p "$ITER_DIR/ros_logs" "$ITER_DIR/gz_home"

cp -f "$ROOT/install_v6/vrx_csd_shadow/lib/vrx_csd_shadow/six_vessel_crossing_driver.py" \
  "$ITER_DIR/source_snapshot/six_vessel_crossing_driver.py"
cp -f "$ROOT/AUTONOMOUS_MEDIUM_ITERATION.md" "$ITER_DIR/"
cp -f "$ROOT/V6_LOCKED_BASELINE.md" "$ITER_DIR/"
sha256sum \
  "$ROOT/install_v6/vrx_csd_shadow/lib/vrx_csd_shadow/six_vessel_crossing_driver.py" \
  "$ROOT/install_v6/vrx_csd_shadow/share/vrx_csd_shadow/models/harbor_residual_sac_actor.json" \
  "$ROOT/install_v6/vrx_csd_shadow/share/vrx_csd_shadow/models/harbor_gnn_gru_best.json" \
  > "$ITER_DIR/checksums.sha256"

cat > "$ITER_DIR/command.txt" <<'EOF'
HEADLESS=true bash run_locked_v6_stable_shuttles.sh medium_wind_wave 20261018
EOF
printf 'No rebuild: checksum-protected install_v6 baseline reproduced unchanged.\n' \
  > "$ITER_DIR/build.log"

cleanup() {
  rm -f "$HEARTBEAT"
}
trap cleanup EXIT

HOME="$ITER_DIR/gz_home" ROS_LOG_DIR="$ITER_DIR/ros_logs" \
HEADLESS=true bash "$ROOT/run_locked_v6_stable_shuttles.sh" \
  medium_wind_wave "$SEED" > "$LAUNCH_LOG" 2>&1 &
child=$!
echo "$child" > "$ITER_DIR/child.pid"

while kill -0 "$child" 2>/dev/null; do
  printf '%s,locked_v6,medium_wind_wave,%s\n' "$(date +%s)" "$child" > "$HEARTBEAT"
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
  touch "$ITER_DIR/RUN_FINISHED"
else
  touch "$ITER_DIR/RUN_FAILED"
fi

exit "$status"
