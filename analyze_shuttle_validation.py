#!/usr/bin/env python3
"""Validate one VRX harbor run before residual-RL training."""

import argparse
import csv
import json
import math
from pathlib import Path


COOPERATIVE = ("wamv_1", "wamv_2", "wamv_3", "wamv_4")
PATROL = ("wamv_5", "wamv_6")
EXPECTED_PATROL_X = {"wamv_5": -25.0, "wamv_6": 25.0}


def number(row, key, default=math.nan):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return default


def flag(row, key):
    return number(row, key, 0.0) >= 0.5


def sign_changes(values):
    signs = []
    for value in values:
        sign = 1 if value > 0 else -1 if value < 0 else 0
        if sign and (not signs or sign != signs[-1]):
            signs.append(sign)
    return max(0, len(signs) - 1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--minimum-patrol-span", type=float, default=18.0)
    parser.add_argument("--maximum-route-error", type=float, default=6.0)
    parser.add_argument("--maximum-thrust-mismatch", type=float, default=0.20)
    parser.add_argument("--require-navigation-success", action="store_true")
    args = parser.parse_args()

    with args.csv_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    by_vessel = {
        vessel: [row for row in rows if row.get("vessel") == vessel]
        for vessel in COOPERATIVE + PATROL
    }

    checks = {}
    checks["all_six_vessels_logged"] = all(by_vessel[name] for name in by_vessel)

    cooperative = {}
    for name in COOPERATIVE:
        vessel_rows = by_vessel[name]
        cooperative[name] = {
            "samples": len(vessel_rows),
            "arrival": any(flag(row, "arrival_qualified") for row in vessel_rows),
            "residence": any(flag(row, "residence_qualified") for row in vessel_rows),
            "minimum_goal_distance_m": min(
                (number(row, "goal_distance") for row in vessel_rows),
                default=math.nan,
            ),
        }

    checks["cooperative_arrival_4_of_4"] = all(
        item["arrival"] for item in cooperative.values()
    )
    checks["cooperative_residence_4_of_4"] = all(
        item["residence"] for item in cooperative.values()
    )

    patrol = {}
    for name in PATROL:
        vessel_rows = by_vessel[name]
        ys = [number(row, "y") for row in vessel_rows]
        xs = [number(row, "x") for row in vessel_rows]
        directions = [number(row, "patrol_direction", 0.0) for row in vessel_rows]
        forward = [
            0.5 * (number(row, "left_thrust", 0.0) + number(row, "right_thrust", 0.0))
            for row in vessel_rows
        ]
        comparable = [
            (command, direction)
            for command, direction in zip(forward, directions)
            if abs(command) >= 45.0 and abs(direction) >= 0.5
        ]
        mismatch = (
            sum(command * direction < 0.0 for command, direction in comparable)
            / len(comparable)
            if comparable else 1.0
        )
        patrol[name] = {
            "samples": len(vessel_rows),
            "y_span_m": max(ys) - min(ys) if ys else 0.0,
            "maximum_x_route_error_m": max(
                (abs(x - EXPECTED_PATROL_X[name]) for x in xs), default=math.inf
            ),
            "direction_changes": sign_changes(directions),
            "thrust_sign_changes": sign_changes(forward),
            "thrust_direction_mismatch_fraction": mismatch,
        }

    checks["patrol_motion_span"] = all(
        item["y_span_m"] >= args.minimum_patrol_span for item in patrol.values()
    )
    checks["patrol_route_tracking"] = all(
        item["maximum_x_route_error_m"] <= args.maximum_route_error
        for item in patrol.values()
    )
    checks["patrol_thrust_direction"] = all(
        item["thrust_direction_mismatch_fraction"] <= args.maximum_thrust_mismatch
        for item in patrol.values()
    )
    checks["patrol_no_chatter"] = all(
        item["thrust_sign_changes"] <= item["direction_changes"] + 4
        for item in patrol.values()
    )

    required = [
        "all_six_vessels_logged",
        "patrol_motion_span",
        "patrol_route_tracking",
        "patrol_thrust_direction",
        "patrol_no_chatter",
    ]
    if args.require_navigation_success:
        required.extend(("cooperative_arrival_4_of_4", "cooperative_residence_4_of_4"))
    passed = all(checks[name] for name in required)

    report = {
        "passed": passed,
        "required_checks": required,
        "checks": checks,
        "cooperative": cooperative,
        "patrol": patrol,
        "csv": str(args.csv_path.resolve()),
    }
    report_path = args.csv_path.parent / "shuttle_validation.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2))
    print(f"SHUTTLE_VALIDATION={'PASS' if passed else 'FAIL'}")
    print(f"report={report_path}")
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
