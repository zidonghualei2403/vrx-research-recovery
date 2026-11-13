#!/usr/bin/env python3
from __future__ import annotations

import csv
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
SEAS = ("calm", "medium_wind_wave", "strong_wind_wave", "extreme_wind_wave")
METHOD = "gnn_gru_sac"
SEED = sys.argv[1] if len(sys.argv) > 1 else "20261014"
COOPERATIVE = tuple(f"wamv_{index}" for index in range(1, 5))


def read_rows(path: Path):
    try:
        with path.open(newline="", encoding="utf-8") as handle:
            return list(csv.DictReader(handle))
    except (FileNotFoundError, OSError, csv.Error):
        return []


def number(row, key, default=math.nan):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def latest_csv(sea: str):
    matches = list(RESULTS.glob(
        f"harbor_{METHOD}_{sea}_seed{SEED}_*/six_vessel_driver.csv"
    ))
    return max(matches, key=lambda path: path.stat().st_mtime) if matches else None


def metrics(path: Path | None):
    rows = read_rows(path) if path else []
    if not rows:
        return 0.0, 0, 0, math.nan, 0
    times = [number(row, "time") for row in rows]
    times = [value for value in times if math.isfinite(value)]
    duration = max(times) - min(times) if times else 0.0
    arrived = 0
    resident = 0
    for name in COOPERATIVE:
        vessel_rows = [row for row in rows if row.get("vessel") == name]
        arrived += int(any(number(row, "arrival_qualified", 0) >= 0.5 for row in vessel_rows))
        resident += int(any(number(row, "residence_qualified", 0) >= 0.5 for row in vessel_rows))
    pair = [number(row, "nearest_vessel_distance") for row in rows]
    pair = [value for value in pair if math.isfinite(value)]
    accepted = sum(number(row, "residual_accepted", 0) >= 0.5 for row in rows)
    return duration, arrived, resident, min(pair) if pair else math.nan, accepted


while True:
    lines = []
    complete = 0
    for sea in SEAS:
        path = latest_csv(sea)
        duration, arrived, resident, pair, accepted = metrics(path)
        done = arrived == 4 and resident == 4
        complete += int(done)
        status = "DONE" if done else ("RUN " if duration > 0 else "WAIT")
        pair_text = f"{pair:.2f}m" if math.isfinite(pair) else "-"
        lines.append(
            f"{status} {sea:<20} t={duration:6.1f}s "
            f"arrival={arrived}/4 residence={resident}/4 "
            f"min_pair={pair_text} accepted={accepted}"
        )
    print("\033[2J\033[H", end="")
    print(f"Harbor V6 VRX sea suite: {complete}/4 sea states complete\n")
    print("\n".join(lines))
    print("\nCtrl+C closes only this progress display.")
    if complete == len(SEAS) or (ROOT / ".v6_suite_complete").exists():
        break
    time.sleep(5)
