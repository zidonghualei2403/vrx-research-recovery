#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAMPAIGN="${CALM_BASELINE_CAMPAIGN:-$ROOT/results/calm_baselines_5seed_10repeat_gui_20261103}"
STATUS="$CAMPAIGN/status.csv"
CURRENT="$CAMPAIGN/current.csv"
HEARTBEAT="$CAMPAIGN/heartbeat.csv"
STOP_REQUEST="$CAMPAIGN/STOP_REQUESTED"
RUNNER_PID="$CAMPAIGN/runner.pid"
CHILD_PID="$CAMPAIGN/child.pid"

METHODS=(distance_star risk_star conditional_csd gnn_gru)
SEEDS=(20261101 20261102 20261110 20261117 20261123)
REPEATS_PER_SEED="${REPEATS_PER_SEED:-10}"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-2}"
DURATION="${CALM_DURATION:-180}"

mkdir -p "$CAMPAIGN/logs"
if [[ ! -f "$STATUS" ]]; then
  echo "method,seed,repeat,status,attempt,run_dir,started_at,finished_at" > "$STATUS"
fi

already_done() {
  local method="$1" seed="$2" repeat="$3"
  grep -q "^${method},${seed},${repeat},DONE," "$STATUS"
}

stop_child() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return 0
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  sleep 4
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
}

cleanup() {
  local child=""
  child="$(cat "$CHILD_PID" 2>/dev/null || true)"
  stop_child "$child"
  bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
  rm -f "$CURRENT" "$HEARTBEAT" "$CHILD_PID"
  if [[ "$(cat "$RUNNER_PID" 2>/dev/null || true)" == "$$" ]]; then
    rm -f "$RUNNER_PID"
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

validate_run() {
  local run_dir="$1"
  [[ -n "$run_dir" && -s "$run_dir/six_vessel_driver.csv" ]] || return 1
  python3 "$ROOT/validate_paper_vrx_run.py" "$run_dir" --max-duration "$DURATION" \
    >/dev/null 2>&1
}

rm -f "$STOP_REQUEST" "$CAMPAIGN/FINISHED"
echo "$$" > "$RUNNER_PID"
total=$(( ${#METHODS[@]} * ${#SEEDS[@]} * REPEATS_PER_SEED ))
echo "Calm baseline campaign: total=$total (4 methods x 5 seeds x $REPEATS_PER_SEED repeats)"
echo "Results: $CAMPAIGN"

for method in "${METHODS[@]}"; do
  for seed in "${SEEDS[@]}"; do
    for repeat in $(seq 1 "$REPEATS_PER_SEED"); do
      if [[ -f "$STOP_REQUEST" ]]; then
        echo "Stop requested. Completed runs are preserved; run this script again to resume."
        exit 0
      fi
      if already_done "$method" "$seed" "$repeat"; then
        continue
      fi

      success=false
      run_dir=""
      for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
        [[ ! -f "$STOP_REQUEST" ]] || break
        bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
        sleep 4
        stamp="$(date +%Y%m%d_%H%M%S)"
        started_at="$(date --iso-8601=seconds)"
        log="$CAMPAIGN/logs/${method}_seed${seed}_repeat$(printf '%02d' "$repeat")_attempt${attempt}_${stamp}.log"
        echo "$method,$seed,$repeat,$attempt" > "$CURRENT"
        echo "RUN method=$method seed=$seed repeat=$repeat/$REPEATS_PER_SEED attempt=$attempt/$MAX_ATTEMPTS"

        setsid env STABILIZE_SHUTTLES=true HEADLESS=false \
          MAX_EXPERIMENT_DURATION="$DURATION" RESIDUAL_SAC_AUTHORITY=0.35 \
          bash "$ROOT/run_harbor_gnn_gru_vrx.sh" calm "$seed" "$method" \
          >"$log" 2>&1 &
        child=$!
        echo "$child" > "$CHILD_PID"
        deadline=$(( $(date +%s) + DURATION + 120 ))
        while kill -0 "$child" 2>/dev/null; do
          echo "$(date +%s),$method,$seed,$repeat,$attempt,$child" > "$HEARTBEAT"
          if [[ -f "$STOP_REQUEST" ]]; then
            stop_child "$child"
            break
          fi
          if grep -Eq 'Network is unreachable|getifaddrs: Operation not permitted' "$log"; then
            stop_child "$child"
            break
          fi
          if (( $(date +%s) >= deadline )); then
            stop_child "$child"
            break
          fi
          sleep 3
        done
        wait "$child" 2>/dev/null || true
        rm -f "$CHILD_PID"

        run_dir="$(sed -n 's/^Results: //p' "$log" | tail -1)"
        bash "$ROOT/cleanup_vrx_batch_processes.sh" >/dev/null 2>&1 || true
        finished_at="$(date --iso-8601=seconds)"
        if validate_run "$run_dir"; then
          python3 "$ROOT/audit_autonomous_run.py" \
            "$run_dir/six_vessel_driver.csv" \
            --output-dir "$run_dir/body_audit" \
            --fleet-time-limit "$DURATION" >/dev/null 2>&1 || true
          echo "$method,$seed,$repeat,DONE,$attempt,$run_dir,$started_at,$finished_at" >> "$STATUS"
          success=true
          break
        fi
        echo "$method,$seed,$repeat,RETRY,$attempt,${run_dir:-none},$started_at,$finished_at" >> "$STATUS"
      done

      if [[ -f "$STOP_REQUEST" ]]; then
        echo "Stop requested. Completed runs are preserved; run this script again to resume."
        exit 0
      fi
      if [[ "$success" != true ]]; then
        echo "$method,$seed,$repeat,FAILED,$MAX_ATTEMPTS,${run_dir:-none},$started_at,$finished_at" >> "$STATUS"
      fi
    done
  done
done

touch "$CAMPAIGN/FINISHED"
echo "CALM_BASELINE_CAMPAIGN_FINISHED=$CAMPAIGN"
