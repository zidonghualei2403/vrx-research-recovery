#!/usr/bin/env python3
import csv
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent
V6_RESULTS = ROOT / "locked_v6_calm_extra50_20261108" / "results.tsv"
MEDIUM = ROOT / "results" / "medium_baselines_new5_gui_20261104"
METHODS = ("distance_star", "risk_star", "conditional_csd", "gnn_gru")


def v6_count():
    if not V6_RESULTS.is_file():
        return 0
    with V6_RESULTS.open() as handle:
        return max(0, sum(1 for line in handle if line.strip()) - 1)


def medium_counts():
    status = MEDIUM / "status.csv"
    if not status.is_file():
        return Counter()
    with status.open(newline="") as handle:
        done = {
            (row["method"], row["seed"], row["repeat"])
            for row in csv.DictReader(handle)
            if row["status"] == "DONE"
        }
    return Counter(method for method, _, _ in done)


while True:
    v6 = v6_count()
    counts = medium_counts()
    medium_total = sum(counts.values())
    current = (MEDIUM / "current.txt").read_text(errors="ignore").strip() if (MEDIUM / "current.txt").is_file() else "idle"
    print("\033[2J\033[H", end="")
    print(f"Locked V6 calm extra campaign: {v6}/50")
    print(f"Medium baselines: {medium_total}/400  current={current}")
    for method in METHODS:
        print(f"  {method:16s} {counts[method]:3d}/100")
    if v6 >= 50 and all(counts[m] >= 100 for m in METHODS):
        print("All requested runs are complete.")
        break
    print("Ctrl+C closes only this progress display.")
    time.sleep(5)
