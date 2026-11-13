#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CSV="$ROOT/autonomous_medium_iterations_20261018/candidate_05_static_wall_dynamics/evidence/harbor_gnn_gru_sac_medium_wind_wave_seed20261018_20261018_214041/six_vessel_driver.csv"

# ROS setup files may inspect optional variables that are unset. Temporarily
# disable nounset while sourcing them, then restore strict checking.
set +u
source /opt/ros/jazzy/setup.bash
source /home/hzh/桌面/vrx_ws/install/setup.bash
source "$ROOT/install_replay/setup.bash"
set -u

export ROS_LOG_DIR="/tmp/ros_logs_c05_historical_replay"
mkdir -p "$ROS_LOG_DIR"

echo "Historical C05 GUI replay"
echo "Recorded fleet completion: 107.811 s"
echo "This is a read-only replay of the immutable CSV, not a new experiment."

ros2 launch vrx_csd_shadow historical_c05_gui_replay.launch.py \
  csv_path:="$CSV" \
  playback_speed:="${PLAYBACK_SPEED:-1.0}" \
  hold_seconds:="${HOLD_SECONDS:-30.0}"
