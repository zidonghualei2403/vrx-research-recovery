#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BATCH_NAME="${BASELINE_BATCH_NAME:-baseline_5seed_50runs_gui_verified_20261101}"
BATCH_DIR="$ROOT/results/$BATCH_NAME"
STATUS="$BATCH_DIR/status.csv"
CURRENT="$BATCH_DIR/current.txt"
HEARTBEAT="$BATCH_DIR/heartbeat.txt"
STOP_REQUEST="$BATCH_DIR/STOP_REQUESTED"
PID_FILE="$ROOT/.baseline_5seed_50runs.pid"

SEAS=(calm medium_wind_wave)
METHODS=(distance_star risk_star conditional_csd gnn_gru)
SEEDS=(20261011 20261014 20261017 20261018 20261027)
REPEATS_PER_SEED="${REPEATS_PER_SEED:-10}"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-2}"
BASELINE_HEADLESS="${BASELINE_HEADLESS:-false}"

mkdir -p "$BATCH_DIR" "$ROOT/logs"
if [[ ! -f "$STATUS" ]]; then
  echo "sea_state,method,seed,repeat,status,attempt,run_dir,started_at,finished_at" > "$STATUS"
fi

cleanup() {
  bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
  rm -f "$CURRENT" "$HEARTBEAT"
  if [[ -f "$PID_FILE" ]] && [[ "$(cat "$PID_FILE" 2>/dev/null || true)" == "$$" ]]; then
    rm -f "$PID_FILE"
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

duration_for() {
  case "$1" in
    calm) echo 180 ;;
    medium_wind_wave) echo 240 ;;
    *) return 2 ;;
  esac
}

already_done() {
  local sea="$1" method="$2" seed="$3" repeat="$4"
  grep -q "^${sea},${method},${seed},${repeat},DONE," "$STATUS"
}

stop_group() {
  local pid="$1"
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  sleep 5
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  sleep 3
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
}

validate_run() {
  local run_dir="$1" max_duration="$2"
  [[ -n "$run_dir" && -s "$run_dir/six_vessel_driver.csv" ]] || return 1
  python3 "$ROOT/validate_paper_vrx_run.py" "$run_dir" --max-duration "$max_duration"
}

run_one() {
  local sea="$1" method="$2" seed="$3" repeat="$4" max_duration="$5" log="$6"
  local timeout=$((max_duration + 120))
  local deadline=$(( $(date +%s) + timeout ))
  local pid status reason="process_exit"

  setsid env \
    STABILIZE_SHUTTLES=true \
    HEADLESS="$BASELINE_HEADLESS" \
    MAX_EXPERIMENT_DURATION="$max_duration" \
    RESIDUAL_SAC_AUTHORITY=0.35 \
    bash "$ROOT/run_harbor_gnn_gru_vrx.sh" "$sea" "$seed" "$method" \
      >"$log" 2>&1 &
  pid=$!

  while kill -0 "$pid" 2>/dev/null; do
    printf '%s,%s,%s,%s,%s,%s\n' \
      "$(date +%s)" "$sea" "$method" "$seed" "$repeat" "$pid" > "$HEARTBEAT"
    if [[ -f "$STOP_REQUEST" ]]; then
      reason="stop_requested"
      stop_group "$pid"
      break
    fi
    if grep -Eq 'Network is unreachable|getifaddrs: Operation not permitted' "$log"; then
      reason="gazebo_network_error"
      stop_group "$pid"
      break
    fi
    if (( $(date +%s) >= deadline )); then
      reason="watchdog_timeout"
      stop_group "$pid"
      break
    fi
    sleep 5
  done

  set +e
  wait "$pid" 2>/dev/null
  status=$?
  set -e
  echo "WATCHDOG_RESULT reason=$reason status=$status" >> "$log"
  return "$status"
}

rm -f "$STOP_REQUEST" "$BATCH_DIR/BATCH_FINISHED"
total=$(( ${#SEAS[@]} * ${#METHODS[@]} * ${#SEEDS[@]} * REPEATS_PER_SEED ))
echo "BASELINE_BATCH_START total=$total repeats_per_seed=$REPEATS_PER_SEED headless=$BASELINE_HEADLESS" | tee -a "$BATCH_DIR/batch.log"

for sea in "${SEAS[@]}"; do
  max_duration="$(duration_for "$sea")"
  for method in "${METHODS[@]}"; do
    for seed in "${SEEDS[@]}"; do
      for repeat in $(seq 1 "$REPEATS_PER_SEED"); do
        if [[ -f "$STOP_REQUEST" ]]; then
          echo "Stop requested; completed runs are preserved." | tee -a "$BATCH_DIR/batch.log"
          exit 0
        fi
        if already_done "$sea" "$method" "$seed" "$repeat"; then
          echo "SKIP $sea $method seed=$seed repeat=$repeat" >> "$BATCH_DIR/batch.log"
          continue
        fi

        printf '%s,%s,%s,%s\n' "$sea" "$method" "$seed" "$repeat" > "$CURRENT"
        success=false
        run_dir=""
        for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
          bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
          sleep 4
          stamp="$(date +%Y%m%d_%H%M%S)"
          started_at="$(date --iso-8601=seconds)"
          log="$BATCH_DIR/${sea}_${method}_seed${seed}_repeat$(printf '%02d' "$repeat")_attempt${attempt}_${stamp}.log"
          echo "RUN $sea $method seed=$seed repeat=$repeat attempt=$attempt/$MAX_ATTEMPTS" | tee -a "$BATCH_DIR/batch.log"

          set +e
          run_one "$sea" "$method" "$seed" "$repeat" "$max_duration" "$log"
          launcher_status=$?
          set -e
          run_dir="$(sed -n 's/^Results: //p' "$log" | tail -1)"
          bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true

          if validate_run "$run_dir" "$max_duration"; then
            finished_at="$(date --iso-8601=seconds)"
            echo "$sea,$method,$seed,$repeat,DONE,$attempt,$run_dir,$started_at,$finished_at" >> "$STATUS"
            success=true
            break
          fi

          finished_at="$(date --iso-8601=seconds)"
          echo "$sea,$method,$seed,$repeat,RETRY,$attempt,${run_dir:-none},$started_at,$finished_at" >> "$STATUS"
          echo "RETRY infrastructure/trajectory failure launcher=$launcher_status" | tee -a "$BATCH_DIR/batch.log"
          sleep 8
        done

        if [[ "$success" != true ]]; then
          echo "$sea,$method,$seed,$repeat,FAILED,$MAX_ATTEMPTS,${run_dir:-none},$started_at,$finished_at" >> "$STATUS"
        fi
      done
    done
  done
done

python3 "$ROOT/summarize_baseline_5seed_50runs.py" "$BATCH_DIR"
touch "$BATCH_DIR/BATCH_FINISHED"
echo "BASELINE_5SEED_50RUNS_FINISHED=$BATCH_DIR" | tee -a "$BATCH_DIR/batch.log"
