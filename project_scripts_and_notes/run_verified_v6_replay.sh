#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_WORLD="${V6_SOURCE_WORLD:-$ROOT/results/harbor_gnn_gru_sac_calm_seed20261011_20261012_092309/csd_harbor.sdf}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${V6_REPLAY_LOG_DIR:-$ROOT/results/verified_v6_replay_${STAMP}}"
RUN_GZ_PARTITION="${VRX_GZ_PARTITION:-vrx_v6_verified_${STAMP}_$$}"
RUN_ROS_DOMAIN_ID="${VRX_ROS_DOMAIN_ID:-$((100 + ($$ % 100)))}"

if [[ ! -s "$SOURCE_WORLD" ]]; then
  echo "Verified V6 world not found: $SOURCE_WORLD" >&2
  exit 1
fi

export GZ_PARTITION="$RUN_GZ_PARTITION"
export ROS_DOMAIN_ID="$RUN_ROS_DOMAIN_ID"
set +u
source /opt/ros/jazzy/setup.bash
source /home/hzh/桌面/vrx_ws/install/setup.bash
source "$ROOT/install_v6/setup.bash"
set -u
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/install_v6/vrx_csd_shadow/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

mkdir -p /tmp/ros_logs "$LOG_DIR"
cp -f "$SOURCE_WORLD" "$LOG_DIR/csd_harbor.sdf"
printf 'variant,source_world,gz_partition,ros_domain_id,control_mode,graph_mode,residual_sac_authority\nV6_verified,%s,%s,%s,csd_safe,gnn_gru_residual_sac,%s\n' \
  "$SOURCE_WORLD" "$RUN_GZ_PARTITION" "$RUN_ROS_DOMAIN_ID" "${RESIDUAL_SAC_AUTHORITY:-0.35}" \
  > "$LOG_DIR/experiment_config.csv"

echo "V6 verified replay: exact saved PASS world"
echo "World: $SOURCE_WORLD"
echo "Results: $LOG_DIR"

ROS_LOG_DIR=/tmp/ros_logs ros2 launch vrx_csd_shadow harbor_gnn_gru_vrx.launch.py \
  log_dir:="$LOG_DIR" \
  control_mode:=csd_safe \
  graph_mode:=gnn_gru_residual_sac \
  encounter_mode:=harbor \
  max_experiment_duration:="${MAX_EXPERIMENT_DURATION:-240.0}" \
  goal_slowdown_radius:="${GOAL_SLOWDOWN_RADIUS:-5.0}" \
  environment_uncertainty:=0.0 \
  sea_state:=calm \
  residual_sac_authority:="${RESIDUAL_SAC_AUTHORITY:-0.35}" \
  headless:="${HEADLESS:-false}" \
  world_file:="$LOG_DIR/csd_harbor.sdf"
