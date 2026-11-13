#!/usr/bin/env python3
import csv
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BASELINE = ROOT / "results" / "calm_baselines_5seed_10repeat_gui_20261103"
V6 = ROOT / "locked_v6_calm_extra50_20261108"


def baseline_counts():
    status = BASELINE / "status.csv"
    if not status.is_file():
        return Counter()
    with status.open(newline="") as handle:
        rows = csv.DictReader(handle)
        return Counter(row["method"] for row in rows if row["status"] == "DONE")


def v6_count():
    results = V6 / "results.tsv"
    if not results.is_file():
        return 0
    with results.open() as handle:
        return max(0, sum(1 for line in handle if line.strip()) - 1)


while True:
    counts = baseline_counts()
    baseline_done = sum(counts.values())
    v6_done = v6_count()
    print("\033[2J\033[H", end="")
    print("Calm experiment sequence")
    print(f"Stage 1 baselines: {baseline_done}/400")
    for method in ("distance_star", "risk_star", "conditional_csd", "gnn_gru"):
        print(f"  {method:16s} {counts[method]:3d}/100")
    print(f"Stage 2 locked V6 extra: {v6_done}/50")
    if (BASELINE / "FINISHED").is_file() and v6_done >= 50:
        print("All requested runs are complete.")
        break
    print("Ctrl+C closes only this progress display.")
    time.sleep(5)
