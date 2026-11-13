#!/usr/bin/env python3
from __future__ import annotations

import csv
import time
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parent
BATCH = ROOT / "results" / "medium_baselines_new5_gui_20261104"
STATUS = BATCH / "status.csv"
CURRENT = BATCH / "current.txt"
METHODS = ("distance_star", "risk_star", "conditional_csd", "gnn_gru")
TOTAL = 4 * 5 * 10


while True:
    rows = []
    if STATUS.is_file():
        with STATUS.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
    done = {
        (row["method"], row["seed"], row["repeat"])
        for row in rows
        if row["status"] == "DONE"
    }
    failed = {
        (row["method"], row["seed"], row["repeat"])
        for row in rows
        if row["status"] == "FAILED"
    } - done
    counts = Counter(key[0] for key in done)
    current = CURRENT.read_text(errors="ignore").strip().replace(",", " / ") if CURRENT.is_file() else "idle"
    width = 36
    filled = int(width * len(done) / TOTAL)
    print("\033[2J\033[H", end="")
    print(f"Medium baselines [{'#' * filled}{'-' * (width - filled)}] {len(done)}/{TOTAL}")
    print(f"current={current}  failed={len(failed)}")
    print("  ".join(f"{method}={counts[method]}/50" for method in METHODS))
    summary = BATCH / "summary_by_sea_method.csv"
    if summary.is_file():
        print(f"summary={summary}")
    if (BATCH / "BATCH_FINISHED").is_file():
        print("Finished.")
        break
    time.sleep(5)
