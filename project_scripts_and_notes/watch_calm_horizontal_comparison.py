#!/usr/bin/env python3
"""Display compact progress for the matched calm-water method comparison."""

import argparse
import csv
import time
from pathlib import Path

from summarize_calm_horizontal import METHODS, RESULTS, ROOT, best_summary


def latest_live_csv(method, seed):
    candidates = list(RESULTS.glob(
        f"harbor_{method}_calm_seed{seed}_*/six_vessel_driver.csv"
    ))
    return max(candidates, key=lambda path: path.stat().st_mtime) if candidates else None


def live_snapshot(path):
    try:
        with path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
    except (FileNotFoundError, OSError, csv.Error):
        return 0.0, 0, 0
    if not rows:
        return 0.0, 0, 0
    times = []
    for row in rows:
        try:
            times.append(float(row["time"]))
        except (KeyError, ValueError):
            pass
    arrived = 0
    resident = 0
    for vessel in ("wamv_1", "wamv_2", "wamv_3", "wamv_4"):
        vessel_rows = [row for row in rows if row.get("vessel") == vessel]
        if not vessel_rows:
            continue
        arrived += any(float(row.get("arrival_qualified", 0)) >= 0.5 for row in vessel_rows)
        resident += any(float(row.get("residence_qualified", 0)) >= 0.5 for row in vessel_rows)
    elapsed = max(times) - min(times) if times else 0.0
    return elapsed, arrived, resident


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", default="20261011")
    parser.add_argument("--max-duration", type=float, default=180.0)
    args = parser.parse_args()
    state = RESULTS / f"calm_horizontal_seed{args.seed}" / "current_method.txt"

    while True:
        summaries = [best_summary(method, args.seed) for method in METHODS]
        done = sum(item.status == "complete" for item in summaries)
        current = state.read_text(encoding="utf-8").strip() if state.exists() else ""
        print("\033[2J\033[H", end="")
        print(f"Calm comparison: {done}/{len(METHODS)} methods complete (seed {args.seed})")
        for item in summaries:
            mark = "DONE" if item.status == "complete" else "WAIT"
            print(
                f"{mark:4s} {item.label:18s} arrival={item.arrivals}/4 "
                f"residence={item.residences}/4"
            )
        if current:
            path = latest_live_csv(current, args.seed)
            elapsed, arrived, resident = live_snapshot(path) if path else (0.0, 0, 0)
            percent = min(100.0, 100.0 * elapsed / args.max_duration)
            width = 36
            filled = int(width * percent / 100.0)
            bar = "#" * filled + "-" * (width - filled)
            print(
                f"\nCurrent {current}: [{bar}] {elapsed:.1f}/{args.max_duration:.0f}s "
                f"arrival={arrived}/4 residence={resident}/4"
            )
        if done == len(METHODS) and not state.exists():
            print("\nAll calm-water methods are complete.")
            return
        print("\nPress Ctrl+C to close only this progress display.", flush=True)
        time.sleep(3)


if __name__ == "__main__":
    main()
