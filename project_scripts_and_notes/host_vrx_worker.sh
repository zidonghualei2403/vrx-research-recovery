#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE="$ROOT/autonomous_medium_iterations_20261018/host_bridge"
REQUESTS="$BRIDGE/requests"
PROCESSING="$BRIDGE/processing"
DONE="$BRIDGE/done"
FAILED="$BRIDGE/failed"
LOGS="$BRIDGE/logs"
HEARTBEAT="$BRIDGE/worker_heartbeat.txt"
CURRENT="$BRIDGE/current_job.txt"
LOCK="$BRIDGE/worker.lock"

mkdir -p "$REQUESTS" "$PROCESSING" "$DONE" "$FAILED" "$LOGS"

exec 9>"$LOCK"
if ! flock -n 9; then
  echo "A host VRX worker is already running."
  exit 0
fi

cleanup() {
  rm -f "$HEARTBEAT" "$CURRENT"
}
trap cleanup EXIT

echo "Host VRX worker started. Keep this terminal open."
echo "Queue: $REQUESTS"

while true; do
  printf '%s,%s\n' "$(date +%s)" "idle" > "$HEARTBEAT"

  job="$(find "$REQUESTS" -maxdepth 1 -type f -name 'job_[0-9][0-9][0-9][0-9]_*.sh' -printf '%f\n' | sort | head -1)"
  if [[ -z "$job" ]]; then
    if [[ -f "$BRIDGE/STOP" ]]; then
      echo "Stop requested; host VRX worker is exiting."
      rm -f "$BRIDGE/STOP"
      exit 0
    fi
    sleep 3
    continue
  fi

  src="$REQUESTS/$job"
  active="$PROCESSING/$job"
  # A stale cleanup may race with the worker after a reboot. If another
  # process already claimed this request, simply rescan the queue.
  if ! mv "$src" "$active" 2>/dev/null; then
    sleep 1
    continue
  fi
  printf '%s\n' "$job" > "$CURRENT"
  log="$LOGS/${job%.sh}.log"
  status_file="$PROCESSING/${job%.sh}.status"

  echo "Running $job"
  set +e
  # The worker lock belongs only to this process. Do not let ROS / Gazebo
  # descendants inherit fd 9 and keep the lock alive after worker exit.
  bash "$active" 9>&- > "$log" 2>&1 &
  child=$!
  while kill -0 "$child" 2>/dev/null; do
    printf '%s,%s,%s\n' "$(date +%s)" "running" "$job" > "$HEARTBEAT"
    sleep 5
  done
  wait "$child"
  status=$?
  set -e

  printf '%s\n' "$status" > "$status_file"
  if [[ "$status" -eq 0 ]]; then
    # Archival helpers may have moved these files already. Missing source
    # files therefore mean the job is already archived, not worker failure.
    [[ ! -e "$active" ]] || mv "$active" "$DONE/$job"
    [[ ! -e "$status_file" ]] || mv "$status_file" "$DONE/${job%.sh}.status"
    echo "Completed $job"
  else
    [[ ! -e "$active" ]] || mv "$active" "$FAILED/$job"
    [[ ! -e "$status_file" ]] || mv "$status_file" "$FAILED/${job%.sh}.status"
    echo "Failed $job with exit status $status"
  fi
  rm -f "$CURRENT"
done
