#!/usr/bin/env python3
"""Return success only for a complete six-vessel harbor trajectory CSV."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def as_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--max-duration", type=float, required=True)
    args = parser.parse_args()
    csv_path = args.run_dir / "six_vessel_driver.csv"
    if not csv_path.is_file() or csv_path.stat().st_size == 0:
        raise SystemExit(1)

    names = set()
    arrivals = set()
    times = []
    with csv_path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            name = row.get("vessel", "")
            names.add(name)
            value = as_float(row.get("time"))
            if math.isfinite(value):
                times.append(value)
            if (
                name in {"wamv_1", "wamv_2", "wamv_3", "wamv_4"}
                and row.get("arrival_qualified", "0").strip().lower() in {"1", "true"}
            ):
                arrivals.add(name)

    duration = max(times) - min(times) if times else 0.0
    all_vessels = all(f"wamv_{index}" in names for index in range(1, 7))
    complete = all_vessels and (len(arrivals) == 4 or duration >= args.max_duration - 12.0)
    print(
        f"run={args.run_dir.name} vessels={len(names)} arrivals={len(arrivals)}/4 "
        f"duration={duration:.1f}s complete={int(complete)}"
    )
    raise SystemExit(0 if complete else 1)


if __name__ == "__main__":
    main()
