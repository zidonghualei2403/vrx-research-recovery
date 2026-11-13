#!/usr/bin/env python3
"""Display progress for a matched harbor method comparison."""

import argparse
import csv
import time
from pathlib import Path

from summarize_calm_horizontal import (
    METHODS,
    RESULTS,
    SEA_LABELS,
    SEA_STATES,
    best_summary,
)


def batch_running():
    markers = (
        "run_harbor_horizontal_comparison.sh",
        "run_harbor_gnn_gru_vrx.sh",
    )
    for proc in Path("/proc").glob("[0-9]*"):
        try:
            command = (proc / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                errors="ignore"
            )
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
            continue
        if any(marker in command for marker in markers):
            return True
    return False


def latest_live_csv(method, seed, sea_state):
    candidates = list(RESULTS.glob(
        f"harbor_{method}_{sea_state}_seed{seed}_*/six_vessel_driver.csv"
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
    parser.add_argument("--sea-state", choices=SEA_STATES, default="medium_wind_wave")
    parser.add_argument("--seed", default="20261011")
    parser.add_argument("--max-duration", type=float, default=240.0)
    args = parser.parse_args()
    output_dir = RESULTS / f"{args.sea_state}_horizontal_seed{args.seed}"
    state = output_dir / "current_method.txt"

    while True:
        summaries = [
            best_summary(
                method,
                args.seed,
                args.sea_state,
                max(0.0, args.max_duration - 10.0),
            )
            for method in METHODS
        ]
        done = sum(item.status == "complete" for item in summaries)
        current = state.read_text(encoding="utf-8").strip() if state.exists() else ""
        print("\033[2J\033[H", end="")
        print(
            f"{SEA_LABELS[args.sea_state]} comparison: "
            f"{done}/{len(METHODS)} complete (seed {args.seed})"
        )
        for item in summaries:
            mark = "DONE" if item.status == "complete" else "WAIT"
            print(
                f"{mark:4s} {item.label:18s} arrival={item.arrivals}/4 "
                f"residence={item.residences}/4"
            )
        if current:
            path = latest_live_csv(current, args.seed, args.sea_state)
            elapsed, arrived, resident = live_snapshot(path) if path else (0.0, 0, 0)
            percent = min(100.0, 100.0 * elapsed / args.max_duration)
            width = 36
            filled = int(width * percent / 100.0)
            bar = "#" * filled + "-" * (width - filled)
            print(
                f"\nCurrent {current}: [{bar}] {elapsed:.1f}/{args.max_duration:.0f}s "
                f"arrival={arrived}/4 residence={resident}/4"
            )
            current_summary = next(item for item in summaries if item.method == current)
            if current_summary.status == "complete":
                print(
                    "This run has finished and its data are saved. "
                    "If it does not advance, restart the batch launcher to resume."
                )
            elif not batch_running():
                print(
                    "STOPPED: the background comparison is not running. "
                    "Restart the launcher to resume from saved methods."
                )
        if done == len(METHODS) and not state.exists():
            print("\nAll methods are complete.")
            return
        print("\nCtrl+C closes only this progress display.", flush=True)
        time.sleep(3)


if __name__ == "__main__":
    main()
