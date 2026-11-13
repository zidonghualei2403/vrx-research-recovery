#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VRX_INSTALL_ROOT="${VRX_INSTALL_ROOT:-/home/hzh/桌面/vrx_ws/install}"
SEA_STATE="${1:-medium_wind_wave}"
SEED="${2:-20261017}"
STAMP="$(date +%Y%m%d_%H%M%S)"

case "$SEA_STATE" in
  calm)
    WAVE_GAIN=0.0; WAVE_PERIOD=5.0; WAVE_DIRECTION=0.0; WAVE_STEEPNESS=0.0
    WIND_SPEED=0.0; WIND_GUST=0.0; MAX_DURATION=240 ;;
  medium_wind_wave)
    WAVE_GAIN=0.85; WAVE_PERIOD=3.8; WAVE_DIRECTION=0.80; WAVE_STEEPNESS=0.90
    WIND_SPEED=14.0; WIND_GUST=2.0; MAX_DURATION=300 ;;
  strong_wind_wave)
    WAVE_GAIN=1.40; WAVE_PERIOD=4.2; WAVE_DIRECTION=0.80; WAVE_STEEPNESS=0.95
    WIND_SPEED=17.0; WIND_GUST=2.5; MAX_DURATION=300 ;;
  extreme_wind_wave)
    WAVE_GAIN=2.0; WAVE_PERIOD=4.5; WAVE_DIRECTION=0.80; WAVE_STEEPNESS=0.95
    WIND_SPEED=20.0; WIND_GUST=3.0; MAX_DURATION=420 ;;
  *)
    echo "Usage: $0 calm|medium_wind_wave|strong_wind_wave|extreme_wind_wave [seed]" >&2
    exit 2 ;;
esac

INSTALL="$ROOT/install_safetyfix_v3"
if [[ ! -f "$INSTALL/setup.bash" ]]; then
  echo "Missing $INSTALL/setup.bash; build the safetyfix workspace first." >&2
  exit 3
fi

UNCERTAINTY="$(awk -v wave="$WAVE_GAIN" -v wind="$WIND_SPEED" 'BEGIN {
  u=0.5*wave/1.4+0.5*wind/17.0; if(u<0)u=0; if(u>1)u=1; printf "%.6f",u
}')"
RUN_GZ_PARTITION="${VRX_GZ_PARTITION:-vrx_harbor_safetyfix_${STAMP}_$$}"
RUN_ROS_DOMAIN_ID="${VRX_ROS_DOMAIN_ID:-$((100 + ($$ % 100)))}"
export GZ_PARTITION="$RUN_GZ_PARTITION"
export ROS_DOMAIN_ID="$RUN_ROS_DOMAIN_ID"

LOG_DIR="$ROOT/results/safetyfix_v3_${SEA_STATE}_seed${SEED}_${STAMP}"
RUNTIME_WORLD="$LOG_DIR/csd_harbor.sdf"

set +u
source /opt/ros/jazzy/setup.bash
source "$VRX_INSTALL_ROOT/setup.bash"
source "$INSTALL/setup.bash"
set -u
export GZ_SIM_SYSTEM_PLUGIN_PATH="$INSTALL/vrx_csd_shadow/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

mkdir -p /tmp/ros_logs "$LOG_DIR"
sed \
  -e "s/key: \"direction\" value { type: DOUBLE double_value: [0-9.]* }/key: \"direction\" value { type: DOUBLE double_value: $WAVE_DIRECTION }/" \
  -e "s/key: \"gain\" value { type: DOUBLE double_value: [0-9.]* }/key: \"gain\" value { type: DOUBLE double_value: $WAVE_GAIN }/" \
  -e "s/key: \"period\" value { type: DOUBLE double_value: [0-9.]* }/key: \"period\" value { type: DOUBLE double_value: $WAVE_PERIOD }/" \
  -e "s/key: \"steepness\" value { type: DOUBLE double_value: [0-9.]* }/key: \"steepness\" value { type: DOUBLE double_value: $WAVE_STEEPNESS }/" \
  -e "s#<wind_mean_velocity>[0-9.]*</wind_mean_velocity>#<wind_mean_velocity>$WIND_SPEED</wind_mean_velocity>#" \
  -e "s#<var_wind_gain_constants>[0-9.]*</var_wind_gain_constants>#<var_wind_gain_constants>$WIND_GUST</var_wind_gain_constants>#" \
  -e "s#<random_seed>[0-9]*</random_seed>#<random_seed>$SEED</random_seed>#" \
  -e '/<wind_obj><name>wamv_5<\/name>/d' \
  -e '/<wind_obj><name>wamv_6<\/name>/d' \
  "$INSTALL/vrx_csd_shadow/share/vrx_csd_shadow/worlds/csd_harbor.sdf" \
  > "$RUNTIME_WORLD"

printf 'controller,sea_state,seed,wave_gain,wind_speed_mps,environment_uncertainty,patrol_wind_exempt,gz_partition,ros_domain_id\n%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
  "v6_safetyfix_v3_inflated_hull_priority" "$SEA_STATE" "$SEED" "$WAVE_GAIN" "$WIND_SPEED" \
  "$UNCERTAINTY" "true" "$RUN_GZ_PARTITION" "$RUN_ROS_DOMAIN_ID" \
  > "$LOG_DIR/experiment_config.csv"

echo "Harbor V6 safetyfix v3 (inflated safety hull + wall barrier + deadlock priority)"
echo "Sea state: $SEA_STATE; seed: $SEED; max duration: ${MAX_EXPERIMENT_DURATION:-$MAX_DURATION} s"
echo "The two prescribed shuttle vessels are exempt from wind drift."
echo "Results: $LOG_DIR"

ROS_LOG_DIR=/tmp/ros_logs ros2 launch vrx_csd_shadow harbor_gnn_gru_vrx.launch.py \
  log_dir:="$LOG_DIR" \
  control_mode:=csd_safe \
  graph_mode:=gnn_gru_residual_sac \
  encounter_mode:=harbor \
  max_experiment_duration:="${MAX_EXPERIMENT_DURATION:-$MAX_DURATION}" \
  goal_slowdown_radius:="${GOAL_SLOWDOWN_RADIUS:-5.0}" \
  environment_uncertainty:="$UNCERTAINTY" \
  sea_state:="$SEA_STATE" \
  residual_sac_authority:="${RESIDUAL_SAC_AUTHORITY:-0.35}" \
  headless:="${HEADLESS:-false}" \
  world_file:="$RUNTIME_WORLD"
