#!/usr/bin/env python3
"""Watch strong and extreme matched harbor comparison suites."""

import argparse
import csv
import time
from pathlib import Path

from summarize_calm_horizontal import METHODS, RESULTS, SEA_LABELS, best_summary


SEA_STATES = ("strong_wind_wave", "extreme_wind_wave")


def suite_running():
    markers = (
        "run_strong_extreme_horizontal_suite.sh",
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
    paths = list(RESULTS.glob(
        f"harbor_{method}_{sea_state}_seed{seed}_*/six_vessel_driver.csv"
    ))
    return max(paths, key=lambda path: path.stat().st_mtime) if paths else None


def live_elapsed(path):
    if path is None:
        return 0.0, 0, 0
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
        arrived += any(
            float(row.get("arrival_qualified", 0)) >= 0.5 for row in vessel_rows
        )
        resident += any(
            float(row.get("residence_qualified", 0)) >= 0.5 for row in vessel_rows
        )
    return (max(times) - min(times) if times else 0.0), arrived, resident


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", default="20261011")
    parser.add_argument("--max-duration", type=float, default=300.0)
    args = parser.parse_args()
    threshold = max(0.0, args.max_duration - 10.0)

    while True:
        total_done = 0
        current_line = None
        print("\033[2J\033[H", end="")
        print(f"Strong + extreme suite, seed {args.seed}\n")
        for sea_state in SEA_STATES:
            summaries = [
                best_summary(method, args.seed, sea_state, threshold)
                for method in METHODS
            ]
            done = sum(item.status == "complete" for item in summaries)
            total_done += done
            print(f"{SEA_LABELS[sea_state]}: {done}/{len(METHODS)} complete")
            for item in summaries:
                mark = "DONE" if item.status == "complete" else "WAIT"
                print(
                    f"  {mark:4s} {item.label:18s} "
                    f"arrival={item.arrivals}/4 residence={item.residences}/4"
                )

            state = (
                RESULTS / f"{sea_state}_horizontal_seed{args.seed}"
                / "current_method.txt"
            )
            if state.exists():
                method = state.read_text(encoding="utf-8").strip()
                elapsed, arrived, resident = live_elapsed(
                    latest_live_csv(method, args.seed, sea_state)
                )
                percent = min(100.0, 100.0 * elapsed / args.max_duration)
                current_line = (
                    f"Current: {sea_state} / {method}, {elapsed:.1f}s "
                    f"({percent:.1f}%), arrival={arrived}/4, residence={resident}/4"
                )
            print()

        running = suite_running()
        if current_line:
            print(current_line)
            if not running:
                print("STOPPED: the background suite is not running; restart it to resume.")
        if total_done == len(SEA_STATES) * len(METHODS) and not current_line:
            print("All strong and extreme runs are complete.")
            return
        print("Ctrl+C closes only this progress display.", flush=True)
        time.sleep(3)


if __name__ == "__main__":
    main()
