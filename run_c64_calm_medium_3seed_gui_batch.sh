#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CANDIDATE="$ROOT/autonomous_medium_iterations_20261018/candidate_64_c60_wamv1_post_conflict_recovery"
BATCH="$CANDIDATE/evidence/batch_calm_medium_3seed_gui_v1"
STATUS="$BATCH/status.csv"
CURRENT="$BATCH/current.txt"
HEARTBEAT="$BATCH/heartbeat.txt"
STOP_REQUEST="$BATCH/STOP_REQUESTED"

SEAS=(calm medium_wind_wave)
SEEDS=(20261014 20261017 20261018)
METHODS=(distance_star risk_star conditional_csd gnn_gru gnn_gru_sac)

mkdir -p "$BATCH" "$ROOT/logs"
touch "$BATCH/batch.log"
if [[ ! -f "$STATUS" ]]; then
  echo "sea_state,seed,method,status,attempt,run_dir" > "$STATUS"
fi

cleanup() {
  bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
  rm -f "$CURRENT" "$HEARTBEAT"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

duration_for() {
  case "$1" in
    calm) echo 180 ;;
    medium_wind_wave) echo 220 ;;
  esac
}

already_done() {
  grep -q "^$1,$2,$3,DONE," "$STATUS"
}

stop_group() {
  local pid="$1"
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  sleep 5
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  sleep 3
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
}

wait_for_existing_vrx() {
  local deadline=$((SECONDS + 900))
  while pgrep -f 'six_vessel_crossing_driver.py|gz sim.*csd_harbor' >/dev/null 2>&1; do
    echo "Waiting for the currently running VRX comparison to finish..." | tee -a "$BATCH/batch.log"
    if (( SECONDS >= deadline )); then
      echo "Existing run exceeded 15 minutes; cleaning this experiment family." | tee -a "$BATCH/batch.log"
      bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
      break
    fi
    sleep 10
  done
}

validate_run() {
  local run_dir="$1"
  local max_duration="$2"
  [[ -n "$run_dir" && -s "$run_dir/six_vessel_driver.csv" ]] || return 1
  python3 "$ROOT/validate_paper_vrx_run.py" "$run_dir" --max-duration "$max_duration"
}

run_one() {
  local sea="$1" seed="$2" method="$3" max_duration="$4" log="$5"
  local timeout=$((max_duration + 120))
  local deadline=$(( $(date +%s) + timeout ))
  local pid status=0 reason=process_exit

  setsid env \
    STABILIZE_SHUTTLES=true \
    HEADLESS=false \
    MAX_EXPERIMENT_DURATION="$max_duration" \
    RESIDUAL_SAC_AUTHORITY=0.35 \
    bash "$CANDIDATE/run_candidate_vrx.sh" "$sea" "$seed" "$method" \
      >"$log" 2>&1 &
  pid=$!

  while kill -0 "$pid" 2>/dev/null; do
    printf '%s,%s,%s,%s,%s\n' "$(date +%s)" "$sea" "$seed" "$method" "$pid" > "$HEARTBEAT"
    if grep -Eq 'Network is unreachable|getifaddrs: Operation not permitted' "$log"; then
      reason=network_error
      stop_group "$pid"
      break
    fi
    if (( $(date +%s) >= deadline )); then
      reason=watchdog_timeout
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

wait_for_existing_vrx
rm -f "$STOP_REQUEST"

for sea in "${SEAS[@]}"; do
  max_duration="$(duration_for "$sea")"
  for seed in "${SEEDS[@]}"; do
    for method in "${METHODS[@]}"; do
      if [[ -f "$STOP_REQUEST" ]]; then
        echo "Stop requested; completed cells are preserved." | tee -a "$BATCH/batch.log"
        exit 0
      fi
      if already_done "$sea" "$seed" "$method"; then
        echo "SKIP $sea seed=$seed $method" | tee -a "$BATCH/batch.log"
        continue
      fi

      printf '%s,%s,%s\n' "$sea" "$seed" "$method" > "$CURRENT"
      success=false
      run_dir=""
      for attempt in 1 2; do
        bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
        sleep 4
        stamp="$(date +%Y%m%d_%H%M%S)"
        log="$BATCH/${sea}_seed${seed}_${method}_attempt${attempt}_${stamp}.log"
        echo "RUN $sea seed=$seed $method attempt=$attempt" | tee -a "$BATCH/batch.log"

        set +e
        run_one "$sea" "$seed" "$method" "$max_duration" "$log"
        launcher_status=$?
        set -e
        run_dir="$(sed -n 's/^Results: //p' "$log" | tail -1)"

        bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
        if validate_run "$run_dir" "$max_duration"; then
          audit_dir="$run_dir/body_audit"
          python3 "$ROOT/audit_autonomous_run.py" \
            "$run_dir/six_vessel_driver.csv" \
            --output-dir "$audit_dir" --fleet-time-limit 115.2 >/dev/null 2>&1 || true
          echo "$sea,$seed,$method,DONE,$attempt,$run_dir" >> "$STATUS"
          success=true
          break
        fi

        echo "$sea,$seed,$method,RETRY,$attempt,${run_dir:-none}" >> "$STATUS"
        echo "Invalid infrastructure run (launcher=$launcher_status); retrying." | tee -a "$BATCH/batch.log"
        sleep 8
      done

      if [[ "$success" != true ]]; then
        echo "$sea,$seed,$method,FAILED,2,${run_dir:-none}" >> "$STATUS"
      fi
    done
  done
done

python3 "$ROOT/summarize_c64_gui_batch.py" "$BATCH"
touch "$BATCH/BATCH_FINISHED"
echo "C64_CALM_MEDIUM_3SEED_GUI_FINISHED=$BATCH" | tee -a "$BATCH/batch.log"
