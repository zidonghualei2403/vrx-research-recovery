#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path

from audit_existing_vrx_results import analyse


METRICS = (
    "fleet_arrival_time_s",
    "mean_path_m",
    "min_coop_coop_clearance_m",
    "min_coop_patrol_clearance_m",
    "min_wall_clearance_m",
    "min_ship_domain_margin_m",
)


def finite(values):
    return [value for value in values if isinstance(value, (int, float)) and math.isfinite(value)]


def mean(values):
    data = finite(values)
    return statistics.fmean(data) if data else math.nan


def sd(values):
    data = finite(values)
    return statistics.stdev(data) if len(data) > 1 else (0.0 if data else math.nan)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("batch_dir", type=Path)
    args = parser.parse_args()
    status_path = args.batch_dir / "status.csv"
    if not status_path.is_file():
        raise SystemExit(f"Missing {status_path}")

    latest_done = {}
    with status_path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            if row["status"] == "DONE":
                key = (row["sea_state"], row["method"], row["seed"], row["repeat"])
                latest_done[key] = row

    run_rows = []
    for key, status in sorted(latest_done.items()):
        run_dir = Path(status["run_dir"])
        csv_path = run_dir / "six_vessel_driver.csv"
        if not csv_path.is_file():
            continue
        result = analyse(csv_path)
        run_rows.append({
            "sea_state": key[0], "method": key[1], "seed": key[2], "repeat": key[3],
            **result, "run_dir": str(run_dir),
        })

    run_output = args.batch_dir / "run_metrics.csv"
    if run_rows:
        with run_output.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(run_rows[0]))
            writer.writeheader()
            writer.writerows(run_rows)

    groups = defaultdict(list)
    for row in run_rows:
        groups[(row["sea_state"], row["method"])].append(row)

    summary_rows = []
    for (sea, method), rows in sorted(groups.items()):
        row = {
            "sea_state": sea,
            "method": method,
            "runs": len(rows),
            "arrival_4of4_rate": sum(item["arrival_count"] == 4 for item in rows) / len(rows),
            "residence_4of4_rate": sum(item["residence_count"] == 4 for item in rows) / len(rows),
            "collision_free_rate": sum(item["physical_collision_free"] == 1 for item in rows) / len(rows),
        }
        for metric in METRICS:
            values = [item[metric] for item in rows]
            row[f"{metric}_mean"] = mean(values)
            row[f"{metric}_sd"] = sd(values)
        summary_rows.append(row)

    summary_output = args.batch_dir / "summary_by_sea_method.csv"
    if summary_rows:
        with summary_output.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(summary_rows[0]))
            writer.writeheader()
            writer.writerows(summary_rows)

    print(f"WROTE {run_output}")
    print(f"WROTE {summary_output}")


if __name__ == "__main__":
    main()
