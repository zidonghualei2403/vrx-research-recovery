#!/usr/bin/env python3
import csv
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BATCH = ROOT / "autonomous_medium_iterations_20261018" / "candidate_64_c60_wamv1_post_conflict_recovery" / "evidence" / "batch_calm_medium_3seed_gui_v1"
STATUS = BATCH / "status.csv"
CURRENT = BATCH / "current.txt"
TOTAL = 2 * 3 * 5

while True:
    done = failed = 0
    if STATUS.exists():
        with STATUS.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        latest = {}
        for row in rows:
            latest[(row["sea_state"], row["seed"], row["method"])] = row["status"]
        done = sum(value == "DONE" for value in latest.values())
        failed = sum(value == "FAILED" for value in latest.values())
    current = CURRENT.read_text(errors="ignore").strip().replace(",", " / ") if CURRENT.exists() else "waiting"
    width = 40
    filled = int(width * done / TOTAL)
    bar = "#" * filled + "-" * (width - filled)
    print(f"\rC64 GUI batch [{bar}] {done}/{TOTAL}  failed={failed}  current={current:<55}", end="", flush=True)
    if (BATCH / "BATCH_FINISHED").exists():
        print("\nBatch finished. See comparison_summary.csv")
        break
    time.sleep(5)
