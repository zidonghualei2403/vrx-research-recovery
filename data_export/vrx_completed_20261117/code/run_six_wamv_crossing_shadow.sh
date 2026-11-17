#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
DISTURBANCE="${1:-calm}"
CONTROLLER="${2:-baseline}"
ENCOUNTER="${3:-offset}"
GRAPH_MODE="${4:-conditional_csd}"
RANDOM_SEED="${5:-20260928}"
MAX_EXPERIMENT_DURATION="${MAX_EXPERIMENT_DURATION:-300.0}"
GOAL_SLOWDOWN_RADIUS="${GOAL_SLOWDOWN_RADIUS:-14.0}"
HEADLESS="${HEADLESS:-false}"

if [[ "$CONTROLLER" != "baseline" && "$CONTROLLER" != "csd_safe" ]]; then
  echo "Controller must be baseline or csd_safe." >&2
  exit 2
fi
if [[ "$ENCOUNTER" != "offset" && "$ENCOUNTER" != "central" ]]; then
  echo "Encounter must be offset or central." >&2
  exit 2
fi
case "$GRAPH_MODE" in
  conditional_csd|distance_star|risk_star|full_graph|conditional_no_gru|conditional_no_conformal|gnn_conditional_csd|gnn_coordinated_csd|gnn_residual_csd|gnn_uncertainty_residual_csd|learned_gnn_gru_graph_shield|gnn_gru_residual_sac)
    ;;
  *)
    echo "Unsupported graph mode: $GRAPH_MODE" >&2
    exit 2
    ;;
esac

if [[ ! "$RANDOM_SEED" =~ ^[0-9]+$ ]]; then
  echo "Random seed must be a non-negative integer." >&2
  exit 2
fi

case "$DISTURBANCE" in
  calm)
    WAVE_GAIN="0.0"
    WAVE_PERIOD="5.0"
    WAVE_DIRECTION="0.0"
    WAVE_STEEPNESS="0.0"
    WIND_SPEED="0.0"
    WIND_DIRECTION="240.0"
    WIND_GUST_GAIN="0.0"
    CURRENT_X="0.0"
    CURRENT_Y="0.0"
    ;;
  wind)
    WAVE_GAIN="0.0"
    WAVE_PERIOD="5.0"
    WAVE_DIRECTION="0.0"
    WAVE_STEEPNESS="0.0"
    WIND_SPEED="6.0"
    WIND_DIRECTION="240.0"
    WIND_GUST_GAIN="0.8"
    CURRENT_X="0.0"
    CURRENT_Y="0.0"
    ;;
  current)
    WAVE_GAIN="0.0"
    WAVE_PERIOD="5.0"
    WAVE_DIRECTION="0.0"
    WAVE_STEEPNESS="0.0"
    WIND_SPEED="0.0"
    WIND_DIRECTION="240.0"
    WIND_GUST_GAIN="0.0"
    CURRENT_X="0.25"
    CURRENT_Y="0.43"
    ;;
  wind_current)
    WAVE_GAIN="0.0"
    WAVE_PERIOD="5.0"
    WAVE_DIRECTION="0.0"
    WAVE_STEEPNESS="0.0"
    WIND_SPEED="6.0"
    WIND_DIRECTION="240.0"
    WIND_GUST_GAIN="0.8"
    CURRENT_X="0.25"
    CURRENT_Y="0.43"
    ;;
  moderate)
    WAVE_GAIN="0.30"
    WAVE_PERIOD="5.0"
    WAVE_DIRECTION="0.40"
    WAVE_STEEPNESS="0.45"
    WIND_SPEED="0.0"
    WIND_DIRECTION="240.0"
    WIND_GUST_GAIN="0.0"
    CURRENT_X="0.0"
    CURRENT_Y="0.0"
    ;;
  strong)
    WAVE_GAIN="0.60"
    WAVE_PERIOD="4.2"
    WAVE_DIRECTION="0.80"
    WAVE_STEEPNESS="0.75"
    WIND_SPEED="0.0"
    WIND_DIRECTION="240.0"
    WIND_GUST_GAIN="0.0"
    CURRENT_X="0.0"
    CURRENT_Y="0.0"
    ;;
  medium_wind_wave)
    WAVE_GAIN="0.85"
    WAVE_PERIOD="3.8"
    WAVE_DIRECTION="0.80"
    WAVE_STEEPNESS="0.90"
    WIND_SPEED="14.0"
    WIND_DIRECTION="240.0"
    WIND_GUST_GAIN="2.0"
    CURRENT_X="0.0"
    CURRENT_Y="0.0"
    ;;
  strong_wind_wave)
    WAVE_GAIN="1.40"
    WAVE_PERIOD="4.2"
    WAVE_DIRECTION="0.80"
    WAVE_STEEPNESS="0.95"
    WIND_SPEED="17.0"
    WIND_DIRECTION="240.0"
    WIND_GUST_GAIN="2.5"
    CURRENT_X="0.0"
    CURRENT_Y="0.0"
    ;;
  extreme_wind_wave)
    WAVE_GAIN="2.0"
    WAVE_PERIOD="4.5"
    WAVE_DIRECTION="0.80"
    WAVE_STEEPNESS="0.95"
    WIND_SPEED="20.0"
    WIND_DIRECTION="240.0"
    WIND_GUST_GAIN="3.0"
    CURRENT_X="0.0"
    CURRENT_Y="0.0"
    ;;
  wind_current_wave)
    WAVE_GAIN="0.30"
    WAVE_PERIOD="5.0"
    WAVE_DIRECTION="0.40"
    WAVE_STEEPNESS="0.55"
    WIND_SPEED="6.0"
    WIND_DIRECTION="240.0"
    WIND_GUST_GAIN="0.8"
    CURRENT_X="0.25"
    CURRENT_Y="0.43"
    ;;
  *)
    echo "Usage: $0 DISTURBANCE [baseline|csd_safe] [offset|central] [GRAPH_MODE] [RANDOM_SEED]" >&2
    echo "GRAPH_MODE includes learned_gnn_gru_graph_shield for the final learned topology." >&2
    exit 2
    ;;
esac

# Normalized exogenous motion uncertainty. In simulation this comes from the
# configured sea state; on hardware it can be supplied by a disturbance
# observer using wind/wave measurements and GPS/IMU motion residuals.
ENVIRONMENT_UNCERTAINTY="$(awk -v wave="$WAVE_GAIN" -v wind="$WIND_SPEED" 'BEGIN {
  u = 0.5 * wave / 1.4 + 0.5 * wind / 17.0;
  if (u < 0) u = 0; if (u > 1) u = 1; printf "%.6f", u
}')"

LOG_DIR="$ROOT/results/six_wamv_${DISTURBANCE}_${CONTROLLER}_${GRAPH_MODE}_${ENCOUNTER}_seed${RANDOM_SEED}_$STAMP"
# VRX derives sensor topic names from the world filename. Keep this stem equal
# to the <world name="csd_six_vessel"> value inside the SDF.
RUNTIME_WORLD="$LOG_DIR/csd_six_vessel.sdf"

source /opt/ros/jazzy/setup.bash
source /home/hzh/桌面/vrx_ws/install/setup.bash
source "$ROOT/install/setup.bash"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/install/vrx_csd_shadow/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

mkdir -p /tmp/ros_logs "$LOG_DIR"
sed \
  -e "s/key: \"direction\" value { type: DOUBLE double_value: [0-9.]* }/key: \"direction\" value { type: DOUBLE double_value: $WAVE_DIRECTION }/" \
  -e "s/key: \"gain\" value { type: DOUBLE double_value: [0-9.]* }/key: \"gain\" value { type: DOUBLE double_value: $WAVE_GAIN }/" \
  -e "s/key: \"period\" value { type: DOUBLE double_value: [0-9.]* }/key: \"period\" value { type: DOUBLE double_value: $WAVE_PERIOD }/" \
  -e "s/key: \"steepness\" value { type: DOUBLE double_value: [0-9.]* }/key: \"steepness\" value { type: DOUBLE double_value: $WAVE_STEEPNESS }/" \
  -e "s#<wind_direction>[0-9.]*</wind_direction>#<wind_direction>$WIND_DIRECTION</wind_direction>#" \
  -e "s#<wind_mean_velocity>[0-9.]*</wind_mean_velocity>#<wind_mean_velocity>$WIND_SPEED</wind_mean_velocity>#" \
  -e "s#<var_wind_gain_constants>[0-9.]*</var_wind_gain_constants>#<var_wind_gain_constants>$WIND_GUST_GAIN</var_wind_gain_constants>#" \
  -e "s#<random_seed>[0-9]*</random_seed>#<random_seed>$RANDOM_SEED</random_seed>#" \
  -e "s#<current_velocity>[-0-9.]* [-0-9.]* 0.0</current_velocity>#<current_velocity>$CURRENT_X $CURRENT_Y 0.0</current_velocity>#" \
  "$ROOT/install/vrx_csd_shadow/share/vrx_csd_shadow/worlds/csd_six_vessel.sdf" \
  > "$RUNTIME_WORLD"

printf 'disturbance,controller,graph_mode,encounter,wave_gain,wave_period_s,wave_direction_rad,wave_steepness,wind_speed_mps,wind_direction_deg,wind_gust_gain,current_x_mps,current_y_mps,environment_uncertainty,random_seed\n%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
  "$DISTURBANCE" "$CONTROLLER" "$GRAPH_MODE" "$ENCOUNTER" "$WAVE_GAIN" "$WAVE_PERIOD" "$WAVE_DIRECTION" "$WAVE_STEEPNESS" \
  "$WIND_SPEED" "$WIND_DIRECTION" "$WIND_GUST_GAIN" "$CURRENT_X" "$CURRENT_Y" "$ENVIRONMENT_UNCERTAINTY" "$RANDOM_SEED" \
  > "$LOG_DIR/experiment_config.csv"

echo "Six-WAM-V central encounter"
echo "Disturbance: $DISTURBANCE"
echo "Controller: $CONTROLLER"
echo "Graph mode: $GRAPH_MODE"
echo "Random seed: $RANDOM_SEED"
echo "Encounter: $ENCOUNTER"
echo "Wind: $WIND_SPEED m/s at $WIND_DIRECTION deg; current: ($CURRENT_X, $CURRENT_Y) m/s"
echo "Results: $LOG_DIR"
if [[ "$GRAPH_MODE" == "learned_gnn_gru_graph_shield" ]]; then
  echo "Final method: harbor-trained GNN-GRU topology + graph controller + ship-domain shield."
else
  echo "The graph publishes edge advice; only csd_safe uses it for constrained control."
fi

ROS_LOG_DIR=/tmp/ros_logs ros2 launch vrx_csd_shadow six_wamv_crossing_shadow.launch.py \
  log_dir:="$LOG_DIR" \
  control_mode:="$CONTROLLER" \
  graph_mode:="$GRAPH_MODE" \
  encounter_mode:="$ENCOUNTER" \
  max_experiment_duration:="$MAX_EXPERIMENT_DURATION" \
  goal_slowdown_radius:="$GOAL_SLOWDOWN_RADIUS" \
  environment_uncertainty:="$ENVIRONMENT_UNCERTAINTY" \
  residual_sac_model_path:="${RESIDUAL_SAC_MODEL_PATH:-__disabled__}" \
  residual_sac_authority:="${RESIDUAL_SAC_AUTHORITY:-1.0}" \
  residual_advantage_margin:="${RESIDUAL_ADVANTAGE_MARGIN:-0.0}" \
  sea_state:="$DISTURBANCE" \
  headless:="$HEADLESS" \
  world_file:="$RUNTIME_WORLD"
