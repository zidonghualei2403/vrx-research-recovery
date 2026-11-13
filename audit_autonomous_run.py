#!/usr/bin/env python3
"""Audit one six-vessel VRX trajectory against the autonomous gate."""

from __future__ import annotations

import argparse
import csv
import json
import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

from audit_existing_vrx_results import COOPERATIVE, PATROL, load_tracks
from evaluate_v6_disturbed_results import HULL_HALF, STATIC_RECTANGLES, rectangle_clearance


def load_static_rectangles(csv_path):
    """Read the physical wall geometry used by this run, with legacy fallback."""
    world_path = csv_path.parent / "csd_harbor.sdf"
    if not world_path.exists():
        return STATIC_RECTANGLES
    try:
        root = ET.parse(world_path).getroot()
        rectangles = []
        for model in root.findall(".//model"):
            if not model.get("name", "").startswith("harbor_wall_"):
                continue
            pose = [float(value) for value in model.findtext("pose", "").split()]
            size = [float(value) for value in model.findtext("./link/collision/geometry/box/size", "").split()]
            if len(pose) >= 2 and len(size) >= 2:
                rectangles.append(((pose[0], pose[1]), (0.5 * size[0], 0.5 * size[1])))
        return tuple(rectangles) if len(rectangles) == 2 else STATIC_RECTANGLES
    except (ET.ParseError, OSError, TypeError, ValueError):
        return STATIC_RECTANGLES


def first_true_time(samples, key, start):
    for sample in samples:
        if sample[key]:
            return sample["time"] - start
    return math.nan


def physical_clearances(tracks, static_rectangles):
    frames = defaultdict(dict)
    for name, samples in tracks.items():
        for sample in samples:
            frames[round(sample["time"], 3)][name] = sample

    minimum_pair = math.inf
    minimum_wall = math.inf
    vessel_overlap_frames = 0
    wall_overlap_frames = 0
    overlap_details = []
    for frame_time in sorted(frames):
        frame = frames[frame_time]
        names = [name for name in (*COOPERATIVE, *PATROL) if name in frame]
        frame_pair_overlap = False
        frame_wall_overlap = False

        def vessel(name):
            sample = frame[name]
            return sample["x"], sample["y"], sample["heading"], HULL_HALF, sample["speed"]

        for first_index, first_name in enumerate(names):
            first = vessel(first_name)
            for second_name in names[first_index + 1:]:
                clearance = rectangle_clearance(first, vessel(second_name))
                minimum_pair = min(minimum_pair, clearance)
                if clearance < 0.0:
                    frame_pair_overlap = True
                    overlap_details.append({
                        "time": frame_time,
                        "kind": "vessel",
                        "first": first_name,
                        "second": second_name,
                        "clearance_m": clearance,
                    })
            if first_name not in COOPERATIVE:
                continue
            for wall_index, (center, half_size) in enumerate(static_rectangles):
                obstacle = (center[0], center[1], 0.0, half_size, 0.0)
                clearance = rectangle_clearance(first, obstacle)
                minimum_wall = min(minimum_wall, clearance)
                if clearance < 0.0:
                    frame_wall_overlap = True
                    overlap_details.append({
                        "time": frame_time,
                        "kind": "wall",
                        "first": first_name,
                        "second": f"wall_{wall_index}",
                        "clearance_m": clearance,
                    })
        vessel_overlap_frames += int(frame_pair_overlap)
        wall_overlap_frames += int(frame_wall_overlap)
    return (
        minimum_pair,
        minimum_wall,
        vessel_overlap_frames,
        wall_overlap_frames,
        overlap_details,
    )


def stall_audit(tracks, window_s=20.0):
    stalls = []
    for name in COOPERATIVE:
        samples = tracks.get(name, [])
        left = 0
        for right, sample in enumerate(samples):
            while sample["time"] - samples[left]["time"] > window_s:
                left += 1
            if sample["time"] - samples[left]["time"] < window_s - 0.5:
                continue
            start = samples[left]
            displacement = math.hypot(sample["x"] - start["x"], sample["y"] - start["y"])
            # The CSV has no explicit distance in load_tracks; infer it from
            # recorded goals retained below by the loader extension in main.
            goal_distance = sample.get("goal_distance", math.inf)
            if goal_distance > 10.0 and displacement < 1.0:
                stalls.append({
                    "vessel": name,
                    "start_time": start["time"],
                    "end_time": sample["time"],
                    "displacement_m": displacement,
                    "goal_distance_m": goal_distance,
                })
                break
    return stalls


def attach_goal_distances(csv_path, tracks):
    lookups = defaultdict(dict)
    with csv_path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                lookups[row["vessel"]][round(float(row["time"]), 3)] = float(row["goal_distance"])
            except (KeyError, TypeError, ValueError):
                continue
    for name, samples in tracks.items():
        for sample in samples:
            sample["goal_distance"] = lookups[name].get(round(sample["time"], 3), math.inf)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--fleet-time-limit", type=float, default=115.2)
    args = parser.parse_args()

    tracks = load_tracks(args.csv_path)
    attach_goal_distances(args.csv_path, tracks)
    all_samples = [sample for samples in tracks.values() for sample in samples]
    if not all_samples:
        raise SystemExit(f"No valid samples in {args.csv_path}")
    start = min(sample["time"] for sample in all_samples)
    end = max(sample["time"] for sample in all_samples)
    arrivals = {name: first_true_time(tracks.get(name, []), "arrival", start) for name in COOPERATIVE}
    residences = {name: first_true_time(tracks.get(name, []), "residence", start) for name in COOPERATIVE}
    arrival_count = sum(math.isfinite(value) for value in arrivals.values())
    residence_count = sum(math.isfinite(value) for value in residences.values())
    fleet_time = max(arrivals.values()) if arrival_count == len(COOPERATIVE) else math.nan
    static_rectangles = load_static_rectangles(args.csv_path)
    pair_clearance, wall_clearance, pair_frames, wall_frames, details = physical_clearances(
        tracks, static_rectangles)
    stalls = stall_audit(tracks)
    six_vessels_valid = all(len(tracks.get(f"wamv_{index}", [])) > 10 for index in range(1, 7))
    passed = bool(
        six_vessels_valid
        and arrival_count == 4
        and pair_frames == 0
        and wall_frames == 0
        and not stalls
        and math.isfinite(fleet_time)
        and fleet_time <= args.fleet_time_limit
    )
    metrics = {
        "csv": str(args.csv_path.resolve()),
        "six_vessels_valid": six_vessels_valid,
        "sample_counts": {name: len(samples) for name, samples in sorted(tracks.items())},
        "arrival_count": arrival_count,
        "residence_count": residence_count,
        "arrival_times_s": arrivals,
        "residence_times_s": residences,
        "fleet_completion_s": fleet_time,
        "run_duration_s": end - start,
        "static_rectangles": static_rectangles,
        "vessel_overlap_frames": pair_frames,
        "wall_overlap_frames": wall_frames,
        "minimum_vessel_clearance_m": pair_clearance,
        "minimum_wall_clearance_m": wall_clearance,
        "distant_stalls_20s": stalls,
        "fleet_time_limit_s": args.fleet_time_limit,
        "passed": passed,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / "metrics.json").open("w") as handle:
        json.dump(metrics, handle, indent=2, allow_nan=True)
    with (args.output_dir / "overlap_events.csv").open("w", newline="") as handle:
        fields = ("time", "kind", "first", "second", "clearance_m")
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(details)
    print(json.dumps(metrics, indent=2, allow_nan=True))


if __name__ == "__main__":
    main()
