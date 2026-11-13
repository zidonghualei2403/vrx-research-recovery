#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BATCH_DIR="$ROOT/results/medium_baselines_new5_gui_20261104"
STATUS="$BATCH_DIR/status.csv"
CURRENT="$BATCH_DIR/current.txt"
STOP_REQUEST="$BATCH_DIR/STOP_REQUESTED"
PID_FILE="$BATCH_DIR/runner.pid"

METHODS=(distance_star risk_star conditional_csd gnn_gru)
SEEDS=(20261101 20261102 20261110 20261117 20261123)
REPEATS_PER_SEED="${REPEATS_PER_SEED:-10}"
DURATION="${MAX_EXPERIMENT_DURATION:-180}"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-3}"

mkdir -p "$BATCH_DIR/logs"
if [[ ! -f "$STATUS" ]]; then
  echo "sea_state,method,seed,repeat,status,run_dir,started_at,finished_at" > "$STATUS"
fi

cleanup() {
  bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
  rm -f "$CURRENT" "$PID_FILE"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

already_done() {
  local method="$1" seed="$2" repeat="$3"
  grep -q "^medium_wind_wave,${method},${seed},${repeat},DONE," "$STATUS"
}

validate_run() {
  local run_dir="$1"
  [[ -n "$run_dir" && -s "$run_dir/six_vessel_driver.csv" ]] || return 1
  python3 "$ROOT/validate_paper_vrx_run.py" "$run_dir" --max-duration "$DURATION"
}

rm -f "$STOP_REQUEST" "$BATCH_DIR/BATCH_FINISHED"
echo "$$" > "$PID_FILE"
total=$(( ${#METHODS[@]} * ${#SEEDS[@]} * REPEATS_PER_SEED ))
echo "Medium baseline GUI batch: $total runs"
echo "Seeds: ${SEEDS[*]}"
echo "Close neither this terminal nor the active Gazebo window. Ctrl+C stops safely."

for method in "${METHODS[@]}"; do
  for seed in "${SEEDS[@]}"; do
    for repeat in $(seq 1 "$REPEATS_PER_SEED"); do
      if [[ -f "$STOP_REQUEST" ]]; then
        echo "Stop requested. Completed runs are preserved."
        exit 0
      fi
      if already_done "$method" "$seed" "$repeat"; then
        continue
      fi

      success=false
      for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
        printf 'medium_wind_wave,%s,%s,%s,attempt%s\n' \
          "$method" "$seed" "$repeat" "$attempt" > "$CURRENT"
        bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
        sleep 5

        stamp="$(date +%Y%m%d_%H%M%S)"
        started_at="$(date --iso-8601=seconds)"
        log="$BATCH_DIR/logs/${method}_seed${seed}_repeat$(printf '%02d' "$repeat")_attempt${attempt}_${stamp}.log"
        echo "RUN method=$method seed=$seed repeat=$repeat/$REPEATS_PER_SEED attempt=$attempt/$MAX_ATTEMPTS"

        set +e
        STABILIZE_SHUTTLES=true \
        HEADLESS=false \
        MAX_EXPERIMENT_DURATION="$DURATION" \
        RESIDUAL_SAC_AUTHORITY=0.35 \
          bash "$ROOT/run_harbor_gnn_gru_vrx.sh" medium_wind_wave "$seed" "$method" \
          2>&1 | tee "$log"
        launcher_status=${PIPESTATUS[0]}

        run_dir="$(sed -n 's/^Results: //p' "$log" | tail -1)"
        finished_at="$(date --iso-8601=seconds)"
        bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true

        if validate_run "$run_dir"; then
          echo "medium_wind_wave,$method,$seed,$repeat,DONE,$run_dir,$started_at,$finished_at" >> "$STATUS"
          success=true
          break
        fi

        echo "medium_wind_wave,$method,$seed,$repeat,RETRY,${run_dir:-none},$started_at,$finished_at" >> "$STATUS"
        echo "Invalid/incomplete launch (launcher=$launcher_status); cleaning and retrying the same run."
      done

      if [[ "$success" != true ]]; then
        echo "medium_wind_wave,$method,$seed,$repeat,FAILED,${run_dir:-none},$started_at,$finished_at" >> "$STATUS"
        echo "The same run failed infrastructure validation $MAX_ATTEMPTS consecutive times; stopping for review."
        exit 1
      fi
      python3 "$ROOT/summarize_baseline_5seed_50runs.py" "$BATCH_DIR" >/dev/null 2>&1 || true
      sleep 3
    done
  done
done

python3 "$ROOT/summarize_baseline_5seed_50runs.py" "$BATCH_DIR"
touch "$BATCH_DIR/BATCH_FINISHED"
echo "MEDIUM_BASELINES_NEW5_FINISHED=$BATCH_DIR"
