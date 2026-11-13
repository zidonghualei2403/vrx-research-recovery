#!/usr/bin/env python3
from __future__ import annotations

import csv
import os
import time
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parent
BATCH_NAME = os.environ.get("BASELINE_BATCH_NAME", "baseline_5seed_50runs_gui_verified_20261101")
BATCH = ROOT / "results" / BATCH_NAME
STATUS = BATCH / "status.csv"
PID_FILE = ROOT / ".baseline_5seed_50runs.pid"
TOTAL = 2 * 4 * 5 * 10


def alive() -> bool:
    try:
        os.kill(int(PID_FILE.read_text().strip()), 0)
        return True
    except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError):
        return False


while True:
    rows = []
    if STATUS.is_file():
        with STATUS.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
    done_keys = {
        (row["sea_state"], row["method"], row["seed"], row["repeat"])
        for row in rows if row["status"] == "DONE"
    }
    failed_keys = {
        (row["sea_state"], row["method"], row["seed"], row["repeat"])
        for row in rows if row["status"] == "FAILED"
    } - done_keys
    retries = sum(row["status"] == "RETRY" for row in rows)
    by_group = Counter((key[0], key[1]) for key in done_keys)
    current = "waiting"
    current_path = BATCH / "current.txt"
    if current_path.is_file():
        current = current_path.read_text(errors="ignore").strip().replace(",", " / ")
    heartbeat = ""
    heartbeat_path = BATCH / "heartbeat.txt"
    if heartbeat_path.is_file():
        try:
            age = max(0, int(time.time()) - int(heartbeat_path.read_text().split(",", 1)[0]))
            heartbeat = f" heartbeat={age}s"
        except (OSError, ValueError):
            heartbeat = " heartbeat=invalid"

    width = 36
    filled = int(width * len(done_keys) / TOTAL)
    bar = "#" * filled + "-" * (width - filled)
    print("\033[2J\033[H", end="")
    print(f"Baseline batch [{bar}] {len(done_keys)}/{TOTAL} ({100*len(done_keys)/TOTAL:.1f}%)")
    print(f"failed={len(failed_keys)} retries={retries} current={current}{heartbeat}")
    for sea in ("calm", "medium_wind_wave"):
        values = "  ".join(
            f"{method}={by_group[(sea, method)]}/50"
            for method in ("distance_star", "risk_star", "conditional_csd", "gnn_gru")
        )
        print(f"{sea}: {values}")

    if (BATCH / "BATCH_FINISHED").is_file():
        print(f"Finished: {BATCH}")
        break
    if not alive():
        print("Batch is not running. Saved progress will resume with the start script.")
        break
    time.sleep(5)
