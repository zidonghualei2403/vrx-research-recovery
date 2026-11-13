#!/usr/bin/env python3
import csv
import time
from pathlib import Path

root = Path(__file__).resolve().parent
baseline = root / "results/medium_baselines_new5_gui_20261104"
sac = root / "c119_medium_matched5seed_20repeat_20261110"

while True:
    status = baseline / "status.csv"
    done = set()
    if status.is_file():
        for row in csv.DictReader(status.open()):
            if row.get("status") == "DONE":
                done.add((row["method"], row["seed"], row["repeat"]))
    gnn = sum(method == "gnn_gru" for method, _, _ in done)

    results = sac / "results.tsv"
    sac_rows = list(csv.DictReader(results.open(), delimiter="\t")) if results.is_file() else []
    current = ""
    for path in (baseline / "current.txt", sac / "current.txt"):
        if path.is_file():
            current = path.read_text(errors="ignore").strip()
            break

    stage = "GNN-GRU" if gnn < 100 else "C119 SAC"
    print(f"\r{stage}: GNN-GRU {gnn}/100 | C119 SAC {len(sac_rows)}/100 | "
          f"current={current or 'idle'}", end="", flush=True)
    if gnn >= 100 and len(sac_rows) >= 100:
        print("\nAll matched medium-sea runs are complete.")
        break
    time.sleep(5)
