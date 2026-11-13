#!/usr/bin/env python3
from __future__ import annotations

import csv
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
SEAS = ("medium_wind_wave", "strong_wind_wave", "extreme_wind_wave")
METHOD = "gnn_gru_sac"
COOPERATIVE = tuple(f"wamv_{index}" for index in range(1, 5))


def read_seed() -> str:
    if len(sys.argv) > 1:
        return sys.argv[1]
    seed_file = ROOT / ".v6_disturbed_seed"
    if seed_file.exists():
        return seed_file.read_text(encoding="utf-8").strip()
    return "20261017"


SEED = read_seed()


def read_rows(path: Path | None):
    if path is None:
        return []
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
    rows = read_rows(path)
    if not rows:
        return 0.0, 0, 0, math.nan, 0, math.nan
    times = [number(row, "time") for row in rows]
    times = [value for value in times if math.isfinite(value)]
    start = min(times) if times else 0.0
    duration = max(times) - start if times else 0.0
    arrived = 0
    resident = 0
    arrival_times = []
    for name in COOPERATIVE:
        vessel_rows = [row for row in rows if row.get("vessel") == name]
        vessel_arrivals = [
            number(row, "time") for row in vessel_rows
            if number(row, "arrival_qualified", 0) >= 0.5
        ]
        if vessel_arrivals:
            arrived += 1
            arrival_times.append(min(vessel_arrivals))
        resident += int(any(
            number(row, "residence_qualified", 0) >= 0.5 for row in vessel_rows
        ))
    pair = [number(row, "nearest_vessel_distance") for row in rows]
    pair = [value for value in pair if math.isfinite(value)]
    accepted = sum(
        number(row, "residual_accepted", 0) >= 0.5 for row in rows
    )
    fleet_arrival = (
        max(arrival_times) - start if len(arrival_times) == 4 else math.nan
    )
    return (
        duration,
        arrived,
        resident,
        min(pair) if pair else math.nan,
        accepted,
        fleet_arrival,
    )


while True:
    lines = []
    finished = 0
    passed = 0
    for sea in SEAS:
        path = latest_csv(sea)
        duration, arrived, resident, pair, accepted, fleet_arrival = metrics(path)
        observed_after_arrival = (
            math.isfinite(fleet_arrival) and duration >= fleet_arrival + 9.0
        )
        success = arrived == 4 and observed_after_arrival
        timed_out = duration >= 350.0 and not success
        ended = success or timed_out
        finished += int(ended)
        passed += int(success)
        if success:
            status = "PASS   "
        elif timed_out:
            status = "TIMEOUT"
        else:
            status = "RUN    " if duration > 0 else "WAIT   "
        pair_text = f"{pair:.2f}m" if math.isfinite(pair) else "-"
        lines.append(
            f"{status} {sea:<20} t={duration:6.1f}s "
            f"arrival={arrived}/4 residence={resident}/4 "
            f"min_center={pair_text} residual={accepted}"
        )

    print("\033[2J\033[H", end="")
    print(
        "Locked V6 disturbed-sea baseline: "
        f"{finished}/3 ended, {passed}/3 passed; seed={SEED}\n"
    )
    print("\n".join(lines))
    print("\nCtrl+C closes only this progress display.")
    if finished == len(SEAS) or (ROOT / ".v6_disturbed_complete").exists():
        break
    time.sleep(5)
