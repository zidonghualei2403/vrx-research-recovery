#!/usr/bin/env bash
# Wait for the full local campaign, then export and publish only audited data.
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAMPAIGN="/home/hzh/Documents/Codex/2026-08-13/hao/vrx_central_sac_v7_unified_ws_v1/results/central_v7_matched_5seed_3sea_5method_20261117"
LOG="$REPO/data_export/upload_20261117.log"
exec 9>"$REPO/data_export/upload_20261117.lock"
flock -n 9 || { echo 'Upload watcher already active'; exit 3; }
echo $$ > "$REPO/data_export/upload_20261117.pid"
trap 'test ! -e "$REPO/data_export/upload_20261117.pid" || unlink "$REPO/data_export/upload_20261117.pid"' EXIT
fail() { echo "$*" | tee "$REPO/data_export/UPLOAD_BLOCKED_20261117"; exit 4; }
echo "Waiting for the 75-run V7 campaign: $CAMPAIGN" | tee -a "$LOG"
last_runner_pid=""
while [[ ! -f "$CAMPAIGN/FINISHED" ]]; do
  [[ -f "$CAMPAIGN/BLOCKED" ]] && fail 'V7 campaign blocked; nothing uploaded.'
  if [[ -f "$CAMPAIGN/runner.pid" ]]; then
    pid="$(cat "$CAMPAIGN/runner.pid")"
    [[ -n "$pid" ]] || { sleep 2; continue; }
    last_runner_pid="$pid"
    [[ -d "/proc/$pid" ]] || fail 'V7 campaign runner ended before FINISHED.'
  elif [[ -n "$last_runner_pid" && ! -d "/proc/$last_runner_pid" ]]; then
    fail 'V7 campaign runner ended before FINISHED.'
  fi
  [[ -f "$REPO/data_export/STOP_UPLOAD_20261117" ]] && { echo 'Upload stopped by request'; exit 0; }
  sleep 30
done
while [[ -f "$CAMPAIGN/runner.pid" ]]; do sleep 5; done
echo 'Campaign finished; checking and packaging 339 local runs.' | tee -a "$LOG"
python3 "$REPO/tools/export_completed_20261117.py" >> "$LOG" 2>&1 || fail "Exporter failed; see $LOG"
python3 - "$REPO/data_export/vrx_completed_20261117/manifest.csv" <<'PY' || fail 'Archive manifest check failed.'
import csv,sys
with open(sys.argv[1],newline='') as f:r=list(csv.DictReader(f))
assert len(r)==339 and len({(x['cohort'],x['sea_state'],x['method'],x['seed'],x['repeat']) for x in r})==339
PY
cd "$REPO"
git add README.md tools/export_completed_20261117.py tools/upload_after_v7_75.sh data_export/vrx_completed_20261117
if git diff --cached --name-only | grep -Ev '^(README\.md|tools/(export_completed_20261117\.py|upload_after_v7_75\.sh)|data_export/vrx_completed_20261117/)' >/dev/null; then
  fail 'Unexpected staged path; upload stopped.'
fi
for attempt in $(seq 1 240); do
  [[ -f "$REPO/data_export/STOP_UPLOAD_20261117" ]] && { echo 'Upload stopped by request'; exit 0; }
  if timeout 45s git fetch origin main >> "$LOG" 2>&1; then break; fi
  [[ "$attempt" -eq 240 ]] && fail 'Network fetch failed for eight hours; archive remains local.'
  echo "Network unavailable; retrying fetch ($attempt/240)." >> "$LOG"
  sleep 120
done
git merge-base --is-ancestor origin/main HEAD || fail 'Remote main changed; upload needs manual merge.'
git diff --cached --quiet && { echo 'Snapshot already committed.' | tee -a "$LOG"; exit 0; }
git commit -m 'Archive five-method six-vessel VRX campaign and completed sea-state runs' >> "$LOG" 2>&1 || fail 'Commit failed; archive remains local.'
for attempt in $(seq 1 240); do
  [[ -f "$REPO/data_export/STOP_UPLOAD_20261117" ]] && { echo 'Upload stopped by request'; exit 0; }
  if timeout 90s git push origin HEAD:main >> "$LOG" 2>&1; then
    git rev-parse HEAD > "$REPO/data_export/UPLOAD_COMPLETE_20261117"
    echo "UPLOAD_COMPLETE commit=$(git rev-parse --short HEAD)" | tee -a "$LOG"
    exit 0
  fi
  [[ "$attempt" -eq 240 ]] && fail 'Push failed for eight hours; local commit is preserved.'
  echo "Push unavailable; retrying ($attempt/240)." >> "$LOG"
  sleep 120
done
