#!/usr/bin/env python3
"""Audit existing harbor VRX runs with one body-aware metric definition.

This script is deliberately read-only with respect to experiment outputs.  It
selects the latest *complete* run for each method, sea state, and seed, then
recomputes arrival, residence, path length, oriented-hull clearance, wall
clearance, and directional ship-domain margins from the raw trajectory CSV.
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

from evaluate_v6_disturbed_results import (
    HULL_HALF,
    STATIC_RECTANGLES,
    as_float,
    obstacle_domain_clearance,
    rectangle_clearance,
    vessel_domain_clearance,
)


ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
METHODS = ("distance_star", "risk_star", "conditional_csd", "gnn_gru", "gnn_gru_sac")
SEA_STATES = ("calm", "medium_wind_wave", "strong_wind_wave", "extreme_wind_wave")
COOPERATIVE = tuple(f"wamv_{index}" for index in range(1, 5))
PATROL = ("wamv_5", "wamv_6")


def flag(row, key):
    return row.get(key, "0").strip().lower() in {"1", "true", "yes"}


def load_tracks(csv_path):
    tracks = defaultdict(list)
    with csv_path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            values = [as_float(row.get(key)) for key in ("time", "x", "y", "heading")]
            if not all(math.isfinite(value) for value in values):
                continue
            tracks[row.get("vessel", "")].append({
                "time": values[0],
                "x": values[1],
                "y": values[2],
                "heading": values[3],
                "arrival": flag(row, "arrival_qualified"),
                "residence": flag(row, "residence_qualified"),
            })

    for samples in tracks.values():
        samples.sort(key=lambda item: item["time"])
        previous = None
        for sample in samples:
            speed = 0.0
            if previous is not None:
                dt = sample["time"] - previous["time"]
                if dt > 1.0e-6:
                    speed = math.hypot(
                        sample["x"] - previous["x"],
                        sample["y"] - previous["y"],
                    ) / dt
            sample["speed"] = min(3.5, max(0.0, speed))
            previous = sample
    return tracks


def run_status(csv_path):
    tracks = load_tracks(csv_path)
    all_samples = [sample for samples in tracks.values() for sample in samples]
    if not all_samples:
        return 0, 0, 0.0
    start = min(sample["time"] for sample in all_samples)
    end = max(sample["time"] for sample in all_samples)
    arrivals = sum(any(sample["arrival"] for sample in tracks.get(name, ())) for name in COOPERATIVE)
    residences = sum(any(sample["residence"] for sample in tracks.get(name, ())) for name in COOPERATIVE)
    return arrivals, residences, end - start


def latest_complete(method, sea_state, seed):
    ranked = []
    pattern = f"harbor_{method}_{sea_state}_seed{seed}_*"
    for directory in RESULTS.glob(pattern):
        csv_path = directory / "six_vessel_driver.csv"
        if not csv_path.is_file() or csv_path.stat().st_size == 0:
            continue
        arrivals, residences, duration = run_status(csv_path)
        # A run is complete when all cooperative vessels arrived or when it
        # reached a full timeout horizon.  Short startup failures are excluded.
        complete = arrivals == 4 or duration >= 170.0
        ranked.append((complete, directory.stat().st_mtime, arrivals, duration, directory, csv_path))
    complete_runs = [item for item in ranked if item[0]]
    if not complete_runs:
        return None, None
    *_, directory, csv_path = max(complete_runs, key=lambda item: item[1])
    return directory, csv_path


def analyse(csv_path):
    tracks = load_tracks(csv_path)
    all_samples = [sample for samples in tracks.values() for sample in samples]
    if not all_samples:
        raise RuntimeError(f"No valid trajectory samples in {csv_path}")
    start_time = min(sample["time"] for sample in all_samples)
    end_time = max(sample["time"] for sample in all_samples)

    arrival_times = {}
    residence_count = 0
    paths = {}
    for name in COOPERATIVE:
        samples = tracks.get(name, [])
        for sample in samples:
            if sample["arrival"]:
                arrival_times.setdefault(name, sample["time"] - start_time)
        residence_count += int(any(sample["residence"] for sample in samples))
        paths[name] = sum(
            math.hypot(second["x"] - first["x"], second["y"] - first["y"])
            for first, second in zip(samples, samples[1:])
        )

    frames = defaultdict(dict)
    for name, samples in tracks.items():
        for sample in samples:
            frames[round(sample["time"], 3)][name] = sample

    minima = {
        "coop_coop": math.inf,
        "coop_patrol": math.inf,
        "coop_wall": math.inf,
        "domain": math.inf,
    }
    domain_values = []
    domain_frame_minima = []
    domain_frame_times = []

    for frame_time in sorted(frames):
        frame = frames[frame_time]
        cooperative = [name for name in COOPERATIVE if name in frame]
        patrol = [name for name in PATROL if name in frame]
        frame_domain = []

        def vessel(name):
            sample = frame[name]
            return (
                sample["x"], sample["y"], sample["heading"], HULL_HALF, sample["speed"]
            )

        for first_index, first_name in enumerate(cooperative):
            first = vessel(first_name)
            for second_name in cooperative[first_index + 1:]:
                second = vessel(second_name)
                minima["coop_coop"] = min(minima["coop_coop"], rectangle_clearance(first, second))
                margin = vessel_domain_clearance(first, second)
                domain_values.append(margin)
                frame_domain.append(margin)
            for second_name in patrol:
                second = vessel(second_name)
                minima["coop_patrol"] = min(minima["coop_patrol"], rectangle_clearance(first, second))
                margin = vessel_domain_clearance(first, second)
                domain_values.append(margin)
                frame_domain.append(margin)
            for center, half_size in STATIC_RECTANGLES:
                obstacle = (center[0], center[1], 0.0, half_size, 0.0)
                minima["coop_wall"] = min(minima["coop_wall"], rectangle_clearance(first, obstacle))
                margin = obstacle_domain_clearance(first, center, half_size)
                domain_values.append(margin)
                frame_domain.append(margin)
        if frame_domain:
            domain_frame_times.append(frame_time)
            domain_frame_minima.append(min(frame_domain))
            minima["domain"] = min(minima["domain"], domain_frame_minima[-1])

    violation_duration = 0.0
    for index in range(1, len(domain_frame_times)):
        if domain_frame_minima[index - 1] < 0.0:
            violation_duration += max(0.0, domain_frame_times[index] - domain_frame_times[index - 1])

    ordered_domain = sorted(domain_values)
    tail_count = max(1, math.ceil(0.10 * len(ordered_domain)))
    finite_paths = [value for value in paths.values() if value > 0.0]
    return {
        "arrival_count": len(arrival_times),
        "residence_count": residence_count,
        "fleet_arrival_time_s": max(arrival_times.values()) if len(arrival_times) == 4 else math.nan,
        "run_duration_s": end_time - start_time,
        "mean_path_m": sum(finite_paths) / len(finite_paths) if finite_paths else math.nan,
        "min_coop_coop_clearance_m": minima["coop_coop"],
        "min_coop_patrol_clearance_m": minima["coop_patrol"],
        "min_wall_clearance_m": minima["coop_wall"],
        "min_ship_domain_margin_m": minima["domain"],
        "ship_domain_cvar10_margin_m": sum(ordered_domain[:tail_count]) / tail_count,
        "ship_domain_violation_duration_s": violation_duration,
        "physical_collision_free": int(
            minima["coop_coop"] >= 0.0
            and minima["coop_patrol"] >= 0.0
            and minima["coop_wall"] >= 0.0
        ),
    }


def render(value):
    if isinstance(value, float):
        return "nan" if not math.isfinite(value) else f"{value:.3f}"
    return str(value)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", default="20261014")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    output = args.output or RESULTS / f"paper_body_aware_audit_seed{args.seed}.csv"

    rows = []
    for sea_state in SEA_STATES:
        for method in METHODS:
            directory, csv_path = latest_complete(method, sea_state, args.seed)
            if csv_path is None:
                print(f"MISSING {sea_state:20s} {method}")
                continue
            row = {
                "seed": args.seed,
                "sea_state": sea_state,
                "method": method,
                **analyse(csv_path),
                "run_dir": str(directory),
            }
            rows.append(row)
            print(
                f"{sea_state:20s} {method:16s} "
                f"arrival={row['arrival_count']}/4 residence={row['residence_count']}/4 "
                f"fleet={render(row['fleet_arrival_time_s'])}s "
                f"pair={render(min(row['min_coop_coop_clearance_m'], row['min_coop_patrol_clearance_m']))}m "
                f"wall={render(row['min_wall_clearance_m'])}m "
                f"safe={row['physical_collision_free']}"
            )

    output.parent.mkdir(parents=True, exist_ok=True)
    if rows:
        with output.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    print(f"WROTE {output}")


if __name__ == "__main__":
    main()
