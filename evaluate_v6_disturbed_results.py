#!/usr/bin/env python3
"""Offline paper metrics for the locked V6 disturbed-sea VRX runs.

The controller remains unchanged. Physical rectangles are used for collision
clearance, while the speed-dependent asymmetric ellipse is reported only as
an evaluation metric.
"""

from __future__ import annotations

import csv
import math
import sys
from collections import defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
SEA_STATES = ("medium_wind_wave", "strong_wind_wave", "extreme_wind_wave")
COOPERATIVE_NAMES = {f"wamv_{index}" for index in range(1, 5)}
HULL_HALF = (2.7, 1.35)
STATIC_RECTANGLES = (((-11.0, 13.0), (7.0, 3.0)), ((12.0, -13.0), (7.0, 3.0)))


def as_float(value, default=math.nan):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def rectangle_support(heading, half_size, direction):
    forward = (math.cos(heading), math.sin(heading))
    lateral = (-forward[1], forward[0])
    return (
        half_size[0] * abs(forward[0] * direction[0] + forward[1] * direction[1])
        + half_size[1] * abs(lateral[0] * direction[0] + lateral[1] * direction[1])
    )


def rectangle_clearance(first, second):
    """Signed SAT clearance for two oriented rectangles."""
    axes = []
    for heading in (first[2], second[2]):
        axes.extend(((math.cos(heading), math.sin(heading)),
                     (-math.sin(heading), math.cos(heading))))
    delta = (second[0] - first[0], second[1] - first[1])
    gaps = []
    for axis in axes:
        distance = abs(delta[0] * axis[0] + delta[1] * axis[1])
        gaps.append(
            distance
            - rectangle_support(first[2], first[3], axis)
            - rectangle_support(second[2], second[3], axis)
        )
    return max(gaps)


def ship_domain_axes(speed):
    # Same geometry as V9, retained here only for post-run evaluation.
    forward = HULL_HALF[0] + 1.2 + 2.4 * max(0.0, speed)
    aft = HULL_HALF[0] + 0.6 * 1.2
    lateral = HULL_HALF[1] + 1.0 + 0.25 * max(0.0, speed)
    return forward, aft, lateral


def ship_domain_support(heading, speed, direction):
    norm = math.hypot(*direction)
    if norm < 1.0e-9:
        direction = (1.0, 0.0)
    else:
        direction = (direction[0] / norm, direction[1] / norm)
    forward_axis = (math.cos(heading), math.sin(heading))
    lateral_axis = (-forward_axis[1], forward_axis[0])
    longitudinal_component = (
        forward_axis[0] * direction[0] + forward_axis[1] * direction[1]
    )
    lateral_component = (
        lateral_axis[0] * direction[0] + lateral_axis[1] * direction[1]
    )
    forward, aft, lateral = ship_domain_axes(speed)
    longitudinal = forward if longitudinal_component >= 0.0 else aft
    return math.hypot(
        longitudinal * longitudinal_component,
        lateral * lateral_component,
    )


def vessel_domain_clearance(first, second):
    delta = (second[0] - first[0], second[1] - first[1])
    distance = math.hypot(*delta)
    if distance < 1.0e-9:
        direction = (1.0, 0.0)
    else:
        direction = (delta[0] / distance, delta[1] / distance)
    return (
        distance
        - ship_domain_support(first[2], first[4], direction)
        - ship_domain_support(second[2], second[4], (-direction[0], -direction[1]))
    )


def obstacle_domain_clearance(vessel, center, half_size):
    delta = (center[0] - vessel[0], center[1] - vessel[1])
    distance = math.hypot(*delta)
    if distance < 1.0e-9:
        direction = (1.0, 0.0)
    else:
        direction = (delta[0] / distance, delta[1] / distance)
    return (
        distance
        - ship_domain_support(vessel[2], vessel[4], direction)
        - rectangle_support(0.0, half_size, (-direction[0], -direction[1]))
    )


def latest_run(sea_state, seed):
    ranked = []
    for directory in RESULTS.glob(
            f"harbor_gnn_gru_sac_{sea_state}_seed{seed}_*"):
        csv_path = directory / "six_vessel_driver.csv"
        if not csv_path.is_file() or csv_path.stat().st_size == 0:
            continue
        arrivals = set()
        times = []
        with csv_path.open(newline="") as handle:
            for row in csv.DictReader(handle):
                time_value = as_float(row.get("time"))
                if math.isfinite(time_value):
                    times.append(time_value)
                if (row.get("vessel") in COOPERATIVE_NAMES
                        and row.get("arrival_qualified", "0").strip().lower()
                        in {"1", "true"}):
                    arrivals.add(row.get("vessel"))
        duration = max(times) - min(times) if times else 0.0
        complete = len(arrivals) == 4 or duration >= 290.0
        ranked.append((complete, len(arrivals), duration,
                       directory.stat().st_mtime, directory, csv_path))
    if not ranked:
        return None, None
    _, _, _, _, directory, csv_path = max(ranked, key=lambda item: item[:4])
    return directory, csv_path


def analyse(csv_path):
    tracks = defaultdict(list)
    raw_rows = []
    with csv_path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            time_value = as_float(row.get("time"))
            x_value = as_float(row.get("x"))
            y_value = as_float(row.get("y"))
            heading = as_float(row.get("heading"))
            if not all(math.isfinite(value) for value in (time_value, x_value, y_value, heading)):
                continue
            record = {
                "time": time_value,
                "name": row.get("vessel", ""),
                "x": x_value,
                "y": y_value,
                "heading": heading,
                "arrival": row.get("arrival_qualified", "0").strip().lower()
                in {"1", "true"},
            }
            raw_rows.append(record)
            tracks[record["name"]].append(record)

    if not raw_rows:
        raise RuntimeError(f"No valid rows in {csv_path}")

    first_time = min(row["time"] for row in raw_rows)
    for samples in tracks.values():
        samples.sort(key=lambda row: row["time"])
        previous = None
        for sample in samples:
            speed = 0.0
            if previous is not None:
                delta_time = sample["time"] - previous["time"]
                if delta_time > 1.0e-6:
                    speed = math.hypot(
                        sample["x"] - previous["x"],
                        sample["y"] - previous["y"],
                    ) / delta_time
            sample["speed"] = min(3.5, max(0.0, speed))
            previous = sample

    frames = defaultdict(list)
    for samples in tracks.values():
        for sample in samples:
            frames[round(sample["time"], 3)].append(sample)

    physical_pair_margins = []
    physical_wall_margins = []
    domain_margins = []
    frame_domain_minimum = []
    frame_times = []
    for frame_time in sorted(frames):
        samples = frames[frame_time]
        if len(samples) < 2:
            continue
        vessels = [
            (sample["x"], sample["y"], sample["heading"], HULL_HALF, sample["speed"])
            for sample in samples
        ]
        frame_domains = []
        for first_index, first in enumerate(vessels):
            for second in vessels[first_index + 1:]:
                physical_pair_margins.append(rectangle_clearance(first, second))
                margin = vessel_domain_clearance(first, second)
                domain_margins.append(margin)
                frame_domains.append(margin)
            for center, half_size in STATIC_RECTANGLES:
                obstacle = (center[0], center[1], 0.0, half_size, 0.0)
                physical_wall_margins.append(rectangle_clearance(first, obstacle))
                margin = obstacle_domain_clearance(first, center, half_size)
                domain_margins.append(margin)
                frame_domains.append(margin)
        if frame_domains:
            frame_times.append(frame_time)
            frame_domain_minimum.append(min(frame_domains))

    violation_duration = 0.0
    for index in range(1, len(frame_times)):
        if frame_domain_minimum[index - 1] < 0.0:
            violation_duration += max(0.0, frame_times[index] - frame_times[index - 1])

    arrivals = {}
    paths = {}
    for name, samples in tracks.items():
        if name not in COOPERATIVE_NAMES:
            continue
        for sample in samples:
            if sample["arrival"]:
                arrivals.setdefault(name, sample["time"] - first_time)
        paths[name] = sum(
            math.hypot(second["x"] - first["x"], second["y"] - first["y"])
            for first, second in zip(samples, samples[1:])
        )

    sorted_domain = sorted(domain_margins)
    tail_count = max(1, math.ceil(0.10 * len(sorted_domain)))
    return {
        "cooperative_arrivals": len(arrivals),
        "fleet_arrival_time_s": max(arrivals.values()) if len(arrivals) == 4 else math.nan,
        "mean_cooperative_path_m": sum(paths.values()) / max(1, len(paths)),
        "min_physical_vessel_clearance_m": min(physical_pair_margins),
        "min_physical_obstacle_clearance_m": min(physical_wall_margins),
        "min_ship_domain_margin_m": min(domain_margins),
        "ship_domain_cvar10_margin_m": sum(sorted_domain[:tail_count]) / tail_count,
        "ship_domain_violation_duration_s": violation_duration,
        "ship_domain_violation_frames": sum(value < 0.0 for value in frame_domain_minimum),
    }


def format_value(value):
    if isinstance(value, float):
        return "nan" if not math.isfinite(value) else f"{value:.3f}"
    return str(value)


def main():
    seed = sys.argv[1] if len(sys.argv) > 1 else "20261017"
    summaries = []
    for sea_state in SEA_STATES:
        directory, csv_path = latest_run(sea_state, seed)
        if csv_path is None:
            print(f"MISSING {sea_state}: seed {seed}")
            continue
        summary = {"sea_state": sea_state, "seed": seed, **analyse(csv_path), "run_dir": str(directory)}
        summaries.append(summary)
        print(
            f"{sea_state}: arrival={summary['cooperative_arrivals']}/4 "
            f"fleet={format_value(summary['fleet_arrival_time_s'])}s "
            f"hull={format_value(summary['min_physical_vessel_clearance_m'])}m "
            f"wall={format_value(summary['min_physical_obstacle_clearance_m'])}m "
            f"domain={format_value(summary['min_ship_domain_margin_m'])}m "
            f"CVaR10={format_value(summary['ship_domain_cvar10_margin_m'])}m"
        )

    if summaries:
        output = RESULTS / f"v6_disturbed_ship_domain_summary_seed{seed}.csv"
        with output.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(summaries[0]))
            writer.writeheader()
            writer.writerows(summaries)
        print(f"WROTE {output}")


if __name__ == "__main__":
    main()
