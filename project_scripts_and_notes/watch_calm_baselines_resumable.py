#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import os
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CAMPAIGN = Path(os.environ.get(
    "CALM_BASELINE_CAMPAIGN",
    ROOT / "results" / "calm_baselines_5seed_10repeat_gui_20261103",
))
STATUS = CAMPAIGN / "status.csv"
METHODS = ("distance_star", "risk_star", "conditional_csd", "gnn_gru")
SEEDS = ("20261101", "20261102", "20261110", "20261117", "20261123")
REPEATS = int(os.environ.get("REPEATS_PER_SEED", "10"))
TOTAL = len(METHODS) * len(SEEDS) * REPEATS
PER_METHOD_TOTAL = len(SEEDS) * REPEATS


def process_alive() -> bool:
    try:
        os.kill(int((CAMPAIGN / "runner.pid").read_text().strip()), 0)
        return True
    except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError):
        return False


while True:
    rows = []
    if STATUS.is_file():
        with STATUS.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
    done = {
        (row["method"], row["seed"], row["repeat"])
        for row in rows if row["status"] == "DONE"
    }
    failed = {
        (row["method"], row["seed"], row["repeat"])
        for row in rows if row["status"] == "FAILED"
    } - done
    retries = sum(row["status"] == "RETRY" for row in rows)
    current = "waiting"
    if (CAMPAIGN / "current.csv").is_file():
        current = (CAMPAIGN / "current.csv").read_text(errors="ignore").strip().replace(",", " / ")
    heartbeat = ""
    if (CAMPAIGN / "heartbeat.csv").is_file():
        try:
            stamp = int((CAMPAIGN / "heartbeat.csv").read_text().split(",", 1)[0])
            heartbeat = f"  heartbeat={max(0, int(time.time()) - stamp)}s"
        except (OSError, ValueError):
            heartbeat = "  heartbeat=invalid"

    width = 40
    filled = int(width * len(done) / TOTAL)
    counts = Counter(key[0] for key in done)
    quality = Counter()
    for row in rows:
        key = (row["method"], row["seed"], row["repeat"])
        if row["status"] != "DONE" or key not in done:
            continue
        metrics_path = Path(row["run_dir"]) / "body_audit" / "metrics.json"
        try:
            metrics = json.loads(metrics_path.read_text())
        except (OSError, ValueError):
            continue
        method = row["method"]
        quality[(method, "audited")] += 1
        if metrics.get("arrival_count") == 4:
            quality[(method, "arrival")] += 1
        if metrics.get("vessel_overlap_frames") == 0:
            quality[(method, "ship_safe")] += 1
        if metrics.get("wall_overlap_frames") == 0:
            quality[(method, "wall_safe")] += 1
    print("\033[2J\033[H", end="")
    print(f"Calm baselines [{'#' * filled}{'-' * (width - filled)}] "
          f"{len(done)}/{TOTAL} ({100.0 * len(done) / TOTAL:.1f}%)")
    print(f"current={current}{heartbeat}")
    print(f"failed={len(failed)}  retries={retries}")
    for method in METHODS:
        seed_text = " ".join(
            f"{seed[-4:]}:{sum((method, seed, str(r)) in done for r in range(1, REPEATS + 1))}/{REPEATS}"
            for seed in SEEDS
        )
        audited = quality[(method, "audited")]
        print(f"{method:16s} {counts[method]:3d}/{PER_METHOD_TOTAL}  {seed_text}")
        print(f"{'':16s} audited={audited}  4/4={quality[(method, 'arrival')]}  "
              f"ship0={quality[(method, 'ship_safe')]}  wall0={quality[(method, 'wall_safe')]}")

    if (CAMPAIGN / "FINISHED").is_file():
        print(f"\nFinished: {CAMPAIGN}")
        break
    if not process_alive():
        print("\nStopped. Run the start command again to continue from saved progress.")
        break
    print("\nCtrl+C closes only this progress display.")
    time.sleep(3)
