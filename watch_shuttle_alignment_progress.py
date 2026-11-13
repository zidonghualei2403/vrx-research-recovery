#!/usr/bin/env python3
"""Show compact progress for the latest shuttle-alignment VRX validation."""

import csv
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
NAMES = ("wamv_1", "wamv_2", "wamv_3", "wamv_4")


def latest_csv():
    candidates = list(ROOT.glob("results/harbor_gnn_gru_*_seed*/six_vessel_driver.csv"))
    return max(candidates, key=lambda path: path.stat().st_mtime) if candidates else None


def value(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def snapshot(path):
    try:
        with path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
    except (FileNotFoundError, OSError):
        return None
    if not rows:
        return None

    start = min(value(row, "time") for row in rows)
    end = max(value(row, "time") for row in rows)
    lines = []
    arrived = 0
    resident = 0
    for name in NAMES:
        vessel_rows = [row for row in rows if row.get("vessel") == name]
        if not vessel_rows:
            lines.append(f"{name}: waiting for pose")
            continue
        row = vessel_rows[-1]
        arrival = value(row, "arrival_qualified") >= 0.5
        residence = value(row, "residence_qualified") >= 0.5
        arrived += int(arrival)
        resident += int(residence)
        lines.append(
            f"{name}: stage={int(value(row, 'harbor_route_stage', -1))} "
            f"goal={value(row, 'goal_distance'):.1f}m "
            f"arrival={'Y' if arrival else 'N'} residence={'Y' if residence else 'N'}"
        )
    return end - start, arrived, resident, lines


def main():
    path = None
    while path is None:
        path = latest_csv()
        if path is None:
            print("Waiting for a shuttle validation CSV...", flush=True)
            time.sleep(2)
    print(f"watching: {path}")

    while True:
        newest = latest_csv()
        if newest is not None and newest != path:
            path = newest
            print(f"startup retry detected; now watching: {path}")
        result = snapshot(path)
        if result is None:
            if (path.parent / "shuttle_validation.json").exists():
                print("This startup attempt had no sensor samples; waiting for automatic retry.", flush=True)
            time.sleep(2)
            continue
        elapsed, arrived, resident, lines = result
        print("\033[2J\033[H", end="")
        print(f"Shuttle GNN-GRU VRX: {elapsed:.1f}s  arrival={arrived}/4  residence={resident}/4")
        print("\n".join(lines), flush=True)
        if resident == 4:
            print("Navigation criteria met; the launcher will finish its observation window.")
            return
        report_path = path.parent / "shuttle_validation.json"
        if report_path.exists():
            newest = latest_csv()
            if newest is not None and newest != path:
                path = newest
                continue
            if sum(1 for _ in path.open(encoding="utf-8")) <= 1:
                print("This startup attempt had no sensor samples; waiting for automatic retry.")
                time.sleep(3)
                continue
            print(f"Validation has ended; final report: {report_path}")
            return
        time.sleep(3)


if __name__ == "__main__":
    main()
