#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEED="${1:-20261018}"
BATCH_ID="paper_matched_body_aware_seed${SEED}"
BATCH_DIR="$ROOT/results/$BATCH_ID"
STATUS="$BATCH_DIR/status.csv"
PID_FILE="$ROOT/.paper_vrx_batch_${SEED}.pid"
HEARTBEAT="$BATCH_DIR/heartbeat.txt"
METHODS=(distance_star risk_star conditional_csd gnn_gru gnn_gru_sac)
SEA_STATES=(calm medium_wind_wave strong_wind_wave extreme_wind_wave)

cleanup_batch_state() {
  rm -f "$BATCH_DIR/current.txt"
  rm -f "$HEARTBEAT"
  bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
  if [[ -f "$PID_FILE" ]] && [[ "$(cat "$PID_FILE" 2>/dev/null || true)" == "$$" ]]; then
    rm -f "$PID_FILE"
  fi
}
trap cleanup_batch_state EXIT
trap 'exit 130' INT TERM

mkdir -p "$BATCH_DIR" "$ROOT/logs"
if [[ ! -f "$STATUS" ]]; then
  echo "sea_state,method,status,attempt,run_dir" > "$STATUS"
fi

duration_for() {
  case "$1" in
    calm) echo 180 ;;
    medium_wind_wave) echo 220 ;;
    strong_wind_wave) echo 260 ;;
    extreme_wind_wave) echo 300 ;;
  esac
}

is_done() {
  grep -q "^$1,$2,DONE," "$STATUS"
}

stop_process_group() {
  local pid="$1"
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  sleep 5
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  sleep 3
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
}

run_cell_with_watchdog() {
  local sea="$1"
  local method="$2"
  local max_duration="$3"
  local run_timeout="$4"
  local log="$5"
  local deadline cell_pid status reason

  deadline=$(( $(date +%s) + run_timeout ))
  reason="process_exit"

  setsid env STABILIZE_SHUTTLES=true \
      HEADLESS=true \
      MAX_EXPERIMENT_DURATION="$max_duration" \
      RESIDUAL_SAC_AUTHORITY=0.35 \
    bash "$ROOT/run_harbor_gnn_gru_vrx.sh" \
      "$sea" "$SEED" "$method" >"$log" 2>&1 &
  cell_pid=$!

  while kill -0 "$cell_pid" 2>/dev/null; do
    printf '%s,%s,%s,%s\n' "$(date +%s)" "$sea" "$method" "$cell_pid" > "$HEARTBEAT"
    if grep -Eq 'Exception sending a multicast message:Network is unreachable|getifaddrs: Operation not permitted' "$log"; then
      reason="gazebo_network_error"
      stop_process_group "$cell_pid"
      break
    fi
    if (( $(date +%s) >= deadline )); then
      reason="watchdog_timeout"
      stop_process_group "$cell_pid"
      break
    fi
    sleep 5
  done

  set +e
  wait "$cell_pid" 2>/dev/null
  status=$?
  set -e
  echo "WATCHDOG_RESULT reason=$reason status=$status" >> "$log"
  return "$status"
}

for sea in "${SEA_STATES[@]}"; do
  max_duration="$(duration_for "$sea")"
  run_timeout=$((max_duration + 100))
  for method in "${METHODS[@]}"; do
    if is_done "$sea" "$method"; then
      echo "SKIP $sea $method: already complete"
      continue
    fi

    printf '%s,%s\n' "$sea" "$method" > "$BATCH_DIR/current.txt"
    success=false
    for attempt in 1 2; do
      bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
      sleep 4
      stamp="$(date +%Y%m%d_%H%M%S)"
      log="$BATCH_DIR/${sea}_${method}_attempt${attempt}_${stamp}.log"
      echo "RUN $sea $method attempt=$attempt/2"

      set +e
      run_cell_with_watchdog \
        "$sea" "$method" "$max_duration" "$run_timeout" "$log"
      launcher_status=$?
      set -e

      bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
      run_dir="$(sed -n 's/^Results: //p' "$log" | tail -1)"
      if [[ -n "$run_dir" ]] && python3 "$ROOT/validate_paper_vrx_run.py" \
          "$run_dir" --max-duration "$max_duration"; then
        echo "$sea,$method,DONE,$attempt,$run_dir" >> "$STATUS"
        success=true
        break
      fi
      echo "$sea,$method,RETRY,$attempt,${run_dir:-none}" >> "$STATUS"
      echo "RETRY $sea $method: launcher_status=$launcher_status" >&2
      sleep 8
    done

    if [[ "$success" != true ]]; then
      echo "$sea,$method,FAILED,2,${run_dir:-none}" >> "$STATUS"
      echo "FAILED $sea $method after two attempts; continuing with the next cell." >&2
    fi
  done
done

python3 "$ROOT/audit_existing_vrx_results.py" \
  --seed "$SEED" --output "$BATCH_DIR/body_aware_audit.csv"
touch "$BATCH_DIR/BATCH_FINISHED"
echo "PAPER_VRX_BATCH_FINISHED=$BATCH_DIR"
