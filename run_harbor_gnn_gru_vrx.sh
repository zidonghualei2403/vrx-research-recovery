#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEA_STATE="${1:-calm}"
SEED="${2:-20261009}"
METHOD="${3:-gnn_gru}"
STAMP="$(date +%Y%m%d_%H%M%S)"

# Isolate every experiment from stale Gazebo transports and ROS 2 nodes left
# behind by an interrupted run. Without this, a new bridge can consume GPS/IMU
# samples from an older harbor world that uses the same vessel and topic names.
RUN_GZ_PARTITION="${VRX_GZ_PARTITION:-vrx_harbor_${STAMP}_$$}"
RUN_ROS_DOMAIN_ID="${VRX_ROS_DOMAIN_ID:-$((100 + ($$ % 100)))}"
export GZ_PARTITION="$RUN_GZ_PARTITION"
export ROS_DOMAIN_ID="$RUN_ROS_DOMAIN_ID"

case "$METHOD" in
  distance_star)
    CONTROL_MODE=csd_safe; GRAPH_MODE=distance_star ;;
  risk_star)
    CONTROL_MODE=csd_safe; GRAPH_MODE=risk_star ;;
  conditional_csd)
    CONTROL_MODE=csd_safe; GRAPH_MODE=conditional_csd ;;
  gnn_gru)
    CONTROL_MODE=csd_safe; GRAPH_MODE=learned_gnn_gru_graph_shield ;;
  gnn_gru_sac)
    CONTROL_MODE=csd_safe; GRAPH_MODE=gnn_gru_residual_sac ;;
  *)
    echo "Method must be distance_star, risk_star, conditional_csd, gnn_gru, or gnn_gru_sac" >&2
    exit 2 ;;
esac

case "$SEA_STATE" in
  calm)
    WAVE_GAIN=0.0; WAVE_PERIOD=5.0; WAVE_DIRECTION=0.0; WAVE_STEEPNESS=0.0
    WIND_SPEED=0.0; WIND_GUST=0.0 ;;
  medium_wind_wave)
    WAVE_GAIN=0.85; WAVE_PERIOD=3.8; WAVE_DIRECTION=0.80; WAVE_STEEPNESS=0.90
    WIND_SPEED=14.0; WIND_GUST=2.0 ;;
  strong_wind_wave)
    WAVE_GAIN=1.40; WAVE_PERIOD=4.2; WAVE_DIRECTION=0.80; WAVE_STEEPNESS=0.95
    WIND_SPEED=17.0; WIND_GUST=2.5 ;;
  extreme_wind_wave)
    WAVE_GAIN=2.0; WAVE_PERIOD=4.5; WAVE_DIRECTION=0.80; WAVE_STEEPNESS=0.95
    WIND_SPEED=20.0; WIND_GUST=3.0 ;;
  *)
    echo "Usage: $0 calm|medium_wind_wave|strong_wind_wave|extreme_wind_wave [seed]" >&2
    exit 2 ;;
esac

UNCERTAINTY="$(awk -v wave="$WAVE_GAIN" -v wind="$WIND_SPEED" 'BEGIN {
  u=0.5*wave/1.4+0.5*wind/17.0; if(u<0)u=0; if(u>1)u=1; printf "%.6f",u
}')"
LOG_DIR="$ROOT/results/harbor_${METHOD}_${SEA_STATE}_seed${SEED}_${STAMP}"
RUNTIME_WORLD="$LOG_DIR/csd_harbor.sdf"

source /opt/ros/jazzy/setup.bash
VRX_WS_INSTALL="${VRX_WS_INSTALL:-$HOME/vrx_ws/install}"
source "$VRX_WS_INSTALL/setup.bash"
# VRX generates per-vessel URDF files inside the vrx_gazebo package at launch.
# Use a project-local package overlay when the original install is read-only.
VRX_MUTABLE_OVERLAY="${VRX_MUTABLE_OVERLAY:-$ROOT/vrx_mutable_overlay_v1}"
if [[ -d "$VRX_MUTABLE_OVERLAY/share/vrx_gazebo" ]]; then
  export AMENT_PREFIX_PATH="$VRX_MUTABLE_OVERLAY:${AMENT_PREFIX_PATH:-}"
fi
source "$ROOT/install_v6/setup.bash"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/install_v6/vrx_csd_shadow/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

mkdir -p /tmp/ros_logs "$LOG_DIR"
sed \
  -e "s/key: \"direction\" value { type: DOUBLE double_value: [0-9.]* }/key: \"direction\" value { type: DOUBLE double_value: $WAVE_DIRECTION }/" \
  -e "s/key: \"gain\" value { type: DOUBLE double_value: [0-9.]* }/key: \"gain\" value { type: DOUBLE double_value: $WAVE_GAIN }/" \
  -e "s/key: \"period\" value { type: DOUBLE double_value: [0-9.]* }/key: \"period\" value { type: DOUBLE double_value: $WAVE_PERIOD }/" \
  -e "s/key: \"steepness\" value { type: DOUBLE double_value: [0-9.]* }/key: \"steepness\" value { type: DOUBLE double_value: $WAVE_STEEPNESS }/" \
  -e "s#<wind_mean_velocity>[0-9.]*</wind_mean_velocity>#<wind_mean_velocity>$WIND_SPEED</wind_mean_velocity>#" \
  -e "s#<var_wind_gain_constants>[0-9.]*</var_wind_gain_constants>#<var_wind_gain_constants>$WIND_GUST</var_wind_gain_constants>#" \
  -e "s#<random_seed>[0-9]*</random_seed>#<random_seed>$SEED</random_seed>#" \
  "$ROOT/install_v6/vrx_csd_shadow/share/vrx_csd_shadow/worlds/csd_harbor.sdf" \
  > "$RUNTIME_WORLD"

# The two noncooperative shuttle vessels are prescribed traffic actors.  For
# matched comparisons their crossing lines must not change with sea state.
# Keep wave rendering and the real WAM-V dynamics, but remove direct wind and
# current forcing from wamv_5/6; their patrol feedback rejects the remaining
# small wave-induced pose motion.
STABLE_SHUTTLES="${STABILIZE_SHUTTLES:-false}"
if [[ "$STABLE_SHUTTLES" == "true" ]]; then
  sed -i \
    -e '/<wind_obj><name>wamv_5<\/name>/d' \
    -e '/<wind_obj><name>wamv_6<\/name>/d' \
    -e '/<current_obj><link_name>wamv_5\/base_link<\/link_name>/d' \
    -e '/<current_obj><link_name>wamv_6\/base_link<\/link_name>/d' \
    "$RUNTIME_WORLD"
fi

printf 'sea_state,seed,wave_gain,wind_speed_mps,environment_uncertainty,stable_shuttles,gz_partition,ros_domain_id\n%s,%s,%s,%s,%s,%s,%s,%s\n' \
  "$SEA_STATE" "$SEED" "$WAVE_GAIN" "$WIND_SPEED" "$UNCERTAINTY" \
  "$STABLE_SHUTTLES" \
  "$RUN_GZ_PARTITION" "$RUN_ROS_DOMAIN_ID" \
  > "$LOG_DIR/experiment_config.csv"

echo "Mixed-obstacle harbor VRX method comparison"
echo "4 cooperative WAM-V + 2 noncooperative WAM-V + 2 rectangular walls"
echo "Method: $METHOD ($CONTROL_MODE / $GRAPH_MODE)"
echo "Sea state: $SEA_STATE; seed: $SEED"
echo "Isolation: GZ_PARTITION=$RUN_GZ_PARTITION ROS_DOMAIN_ID=$RUN_ROS_DOMAIN_ID"
echo "Results: $LOG_DIR"

ROS_LOG_DIR=/tmp/ros_logs ros2 launch vrx_csd_shadow harbor_gnn_gru_vrx.launch.py \
  log_dir:="$LOG_DIR" \
  control_mode:="$CONTROL_MODE" \
  graph_mode:="$GRAPH_MODE" \
  encounter_mode:=harbor \
  max_experiment_duration:="${MAX_EXPERIMENT_DURATION:-360.0}" \
  goal_slowdown_radius:="${GOAL_SLOWDOWN_RADIUS:-5.0}" \
  environment_uncertainty:="$UNCERTAINTY" \
  sea_state:="$SEA_STATE" \
  residual_sac_authority:="${RESIDUAL_SAC_AUTHORITY:-0.35}" \
  headless:="${HEADLESS:-false}" \
  world_file:="$RUNTIME_WORLD"
