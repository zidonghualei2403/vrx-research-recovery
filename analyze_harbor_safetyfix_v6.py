#!/usr/bin/env python3
import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path


def number(row, key, default=math.nan):
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return default


def finite_values(rows, key):
    return [value for row in rows
            if math.isfinite(value := number(row, key))]


def main():
    parser = argparse.ArgumentParser(
        description="Summarize a V6 safetyfix V6 asymmetric-right-of-way VRX run.")
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    csv_path = args.run_dir / "six_vessel_driver.csv"
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    with csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit(f"CSV is empty: {csv_path}")

    cooperative = {f"wamv_{index}" for index in range(1, 5)}
    per_vessel = defaultdict(list)
    for row in rows:
        per_vessel[row["vessel"]].append(row)

    arrivals = 0
    residences = 0
    first_arrivals = []
    for name in sorted(cooperative):
        vessel_rows = per_vessel[name]
        if any(number(row, "arrival_qualified", 0.0) >= 0.5
               for row in vessel_rows):
            arrivals += 1
            first = next(number(row, "time") for row in vessel_rows
                         if number(row, "arrival_qualified", 0.0) >= 0.5)
            first_arrivals.append(first)
        if any(number(row, "residence_qualified", 0.0) >= 0.5
               for row in vessel_rows):
            residences += 1

    start_time = min(number(row, "time") for row in rows)
    end_time = max(number(row, "time") for row in rows)
    fleet_arrival = (
        max(first_arrivals) - start_time if len(first_arrivals) == 4
        else math.nan)
    vessel_clearances = finite_values(rows, "physical_vessel_clearance")
    wall_clearances = finite_values(rows, "physical_wall_clearance")
    safety_wall_clearances = finite_values(rows, "safety_wall_clearance")
    min_vessel = min(vessel_clearances)
    min_wall = min(wall_clearances)
    min_safety_wall = (min(safety_wall_clearances)
                       if safety_wall_clearances else math.nan)
    collision_frames = len({number(row, "time") for row in rows
                            if number(row, "physical_vessel_clearance") < 0.0})
    wall_frames = len({number(row, "time") for row in rows
                       if number(row, "physical_wall_clearance") < 0.0})
    override_frames = len({number(row, "time") for row in rows
                           if number(row, "safety_override", 0.0) >= 0.5})
    priority_frames = len({number(row, "time") for row in rows
                           if number(row, "deadlock_priority_yield", 0.0) >= 0.5})

    print(f"CSV: {csv_path}")
    print(f"duration_s: {end_time - start_time:.1f}")
    print(f"arrival: {arrivals}/4")
    print(f"residence: {residences}/4")
    print("fleet_arrival_s: " + (
        f"{fleet_arrival:.1f}" if math.isfinite(fleet_arrival) else "timeout"))
    print(f"min_physical_vessel_clearance_m: {min_vessel:.3f}")
    print(f"min_physical_wall_clearance_m: {min_wall:.3f}")
    if math.isfinite(min_safety_wall):
        print(f"min_inflated_hull_wall_clearance_m: {min_safety_wall:.3f}")
    print(f"vessel_collision_frames: {collision_frames}")
    print(f"wall_collision_frames: {wall_frames}")
    print(f"safety_override_frames: {override_frames}")
    print(f"deadlock_priority_frames: {priority_frames}")
    print("per_vessel:")
    for name in sorted(cooperative):
        vessel_rows = per_vessel[name]
        vessel_gap = min(finite_values(vessel_rows, "physical_vessel_clearance"))
        wall_gap = min(finite_values(vessel_rows, "physical_wall_clearance"))
        safety_gaps = finite_values(vessel_rows, "safety_wall_clearance")
        safety_gap = min(safety_gaps) if safety_gaps else math.nan
        overrides = sum(number(row, "safety_override", 0.0) >= 0.5
                        for row in vessel_rows)
        priority = sum(number(row, "deadlock_priority_yield", 0.0) >= 0.5
                       for row in vessel_rows)
        print(
            f"  {name}: vessel={vessel_gap:.3f} m, wall={wall_gap:.3f} m, "
            f"inflated_wall={safety_gap:.3f} m, overrides={overrides}, "
            f"priority_yield={priority}")
    passed = collision_frames == 0 and wall_frames == 0
    print(f"strict_collision_free: {'PASS' if passed else 'FAIL'}")


if __name__ == "__main__":
    main()
