#!/usr/bin/env python3
"""Compare matched six-vessel VRX runs using trajectory-level safety metrics."""

import argparse
import csv
import itertools
import math
import statistics
from collections import defaultdict
from pathlib import Path


VESSEL_LENGTH_M = 5.4
GOAL_TOLERANCE_M = 5.0
RESIDENCE_RADIUS_M = 6.0
REQUIRED_RESIDENCE_S = 10.0
POST_ARRIVAL_OBSERVATION_S = 10.0
NEAR_MISS_DISTANCE_M = 8.0
CAUTION_DISTANCE_M = 10.0
NOMINAL_THRUST = 300.0


def read_driver(run_dir):
    path = Path(run_dir) / "six_vessel_driver.csv"
    with path.open(newline="") as handle:
        return path, list(csv.DictReader(handle))


def read_config(run_dir):
    path = Path(run_dir) / "experiment_config.csv"
    if not path.exists():
        return {}
    with path.open(newline="") as handle:
        return next(csv.DictReader(handle), {})


def summarize(run_dir):
    path, rows = read_driver(run_dir)
    config = read_config(run_dir)
    if not rows:
        raise ValueError(f"No data rows in {path}")

    by_vessel = defaultdict(list)
    by_time = defaultdict(dict)
    for row in rows:
        name = row["vessel"]
        by_vessel[name].append(row)
        by_time[row["time"]][name] = (float(row["x"]), float(row["y"]))

    minimum_pair_distance = math.inf
    closest_pair = ("", "")
    overlap_pair_samples = 0
    complete_frames = 0
    for positions in by_time.values():
        if len(positions) != len(by_vessel):
            continue
        complete_frames += 1
        for first, second in itertools.combinations(sorted(positions), 2):
            ax, ay = positions[first]
            bx, by = positions[second]
            distance = math.hypot(ax - bx, ay - by)
            if distance < minimum_pair_distance:
                minimum_pair_distance = distance
                closest_pair = (first, second)
            if distance < VESSEL_LENGTH_M:
                overlap_pair_samples += 1

    vessel_rows = []
    run_start = min(float(row["time"]) for row in rows)
    for name in sorted(by_vessel):
        samples = by_vessel[name]
        distances = [float(row["goal_distance"]) for row in samples]
        sample_times = [float(row["time"]) for row in samples]
        points = [(float(row["x"]), float(row["y"])) for row in samples]
        path_length = sum(
            math.hypot(bx - ax, by - ay)
            for (ax, ay), (bx, by) in zip(points, points[1:])
        )
        arrival_indices = [
            index for index, distance in enumerate(distances)
            if distance <= GOAL_TOLERANCE_M
        ]
        region_arrival_indices = [
            index for index, distance in enumerate(distances)
            if distance <= RESIDENCE_RADIUS_M
        ]
        first_arrival_s = (
            sample_times[arrival_indices[0]] - run_start
            if arrival_indices else math.nan
        )
        first_region_arrival_s = (
            sample_times[region_arrival_indices[0]] - run_start
            if region_arrival_indices else math.nan
        )
        longest_residence_s = 0.0
        residence_start = None
        for index, distance in enumerate(distances):
            if distance <= RESIDENCE_RADIUS_M:
                if residence_start is None:
                    residence_start = sample_times[index]
                longest_residence_s = max(
                    longest_residence_s, sample_times[index] - residence_start)
            else:
                residence_start = None
        vessel_rows.append({
            "vessel": name,
            "ever_reached": int(min(distances) <= GOAL_TOLERANCE_M),
            "entered_6m_goal_region": int(bool(region_arrival_indices)),
            "finally_at_goal": int(distances[-1] <= GOAL_TOLERANCE_M),
            "first_arrival_s": first_arrival_s,
            "first_entry_6m_s": first_region_arrival_s,
            "longest_residence_within_6m_s": longest_residence_s,
            "residence_success_10s": int(longest_residence_s >= REQUIRED_RESIDENCE_S),
            "minimum_goal_distance_m": min(distances),
            "final_goal_distance_m": distances[-1],
            "path_length_m": path_length,
        })

    times = [float(row["time"]) for row in rows]
    finite_region_arrivals = [
        row["first_entry_6m_s"] for row in vessel_rows
        if math.isfinite(row["first_entry_6m_s"])
    ]
    mission_arrival_time = (
        max(finite_region_arrivals)
        if len(finite_region_arrivals) == len(vessel_rows) else math.nan
    )
    evaluation_end = (
        run_start + mission_arrival_time + POST_ARRIVAL_OBSERVATION_S
        if math.isfinite(mission_arrival_time) else max(times)
    )

    standardized_minimum_pair_distance = math.inf
    standardized_overlap_samples = 0
    standardized_pair_distances = []
    standardized_near_miss_samples = 0
    standardized_caution_samples = 0
    standardized_risk_exposure = 0.0
    for time_text, positions in by_time.items():
        if float(time_text) > evaluation_end or len(positions) != len(by_vessel):
            continue
        for first, second in itertools.combinations(sorted(positions), 2):
            ax, ay = positions[first]
            bx, by = positions[second]
            distance = math.hypot(ax - bx, ay - by)
            standardized_pair_distances.append(distance)
            standardized_minimum_pair_distance = min(
                standardized_minimum_pair_distance, distance)
            if distance < VESSEL_LENGTH_M:
                standardized_overlap_samples += 1
            if distance < NEAR_MISS_DISTANCE_M:
                standardized_near_miss_samples += 1
                standardized_risk_exposure += NEAR_MISS_DISTANCE_M - distance
            if distance < CAUTION_DISTANCE_M:
                standardized_caution_samples += 1

    standardized_paths = []
    achieved_progresses = []
    energy_proxies = []
    control_variations = []
    for name in sorted(by_vessel):
        samples = [
            row for row in by_vessel[name]
            if float(row["time"]) <= evaluation_end
        ]
        points = [(float(row["x"]), float(row["y"])) for row in samples]
        standardized_paths.append(sum(
            math.hypot(bx - ax, by - ay)
            for (ax, ay), (bx, by) in zip(points, points[1:])
        ))
        if samples:
            initial_goal_distance = math.hypot(
                float(samples[0]["goal_x"]) - float(samples[0]["x"]),
                float(samples[0]["goal_y"]) - float(samples[0]["y"]),
            )
            final_goal_distance = float(samples[-1]["goal_distance"])
            achieved_progresses.append(max(
                0.0, initial_goal_distance - final_goal_distance))
        energy = 0.0
        variation = 0.0
        previous = None
        previous_time = None
        for sample in samples:
            current_time = float(sample["time"])
            command = (
                float(sample["left_thrust"]), float(sample["right_thrust"])
            )
            if previous is not None:
                dt = max(current_time - previous_time, 0.0)
                energy += 0.5 * dt * sum(
                    (value / NOMINAL_THRUST) ** 2 for value in command)
                variation += sum(
                    abs(value - old) / NOMINAL_THRUST
                    for value, old in zip(command, previous)
                )
            previous = command
            previous_time = current_time
        energy_proxies.append(energy)
        control_variations.append(variation)

    standardized_path_mean = sum(standardized_paths) / len(standardized_paths)
    achieved_progress_mean = sum(achieved_progresses) / len(achieved_progresses)
    pair_distances_sorted = sorted(standardized_pair_distances)
    pair_p05 = (
        pair_distances_sorted[max(0, math.ceil(0.05 * len(pair_distances_sorted)) - 1)]
        if pair_distances_sorted else math.nan
    )
    standardized_rows = [
        row for row in rows if float(row["time"]) <= evaluation_end
    ]

    control_mode = rows[0].get("control_mode", "unknown")
    graph_mode = rows[0].get("graph_mode", "")
    if not graph_mode:
        graph_mode = "not_applicable" if control_mode == "baseline" else "legacy_unknown"
    summary = {
        "run": Path(run_dir).name,
        "disturbance": config.get("disturbance", "unknown"),
        "random_seed": config.get("random_seed", "unknown"),
        "encounter": config.get("encounter", "unknown"),
        "control_mode": control_mode,
        "graph_mode": graph_mode,
        "duration_s": max(times) - min(times),
        "vessels": len(by_vessel),
        "ever_reached_count": sum(row["ever_reached"] for row in vessel_rows),
        "arrival_success_count_6m": sum(
            row["entered_6m_goal_region"] for row in vessel_rows),
        "finally_at_goal_count": sum(row["finally_at_goal"] for row in vessel_rows),
        "residence_success_count": sum(
            row["residence_success_10s"] for row in vessel_rows),
        "mean_first_arrival_s": sum(
            row["first_arrival_s"] for row in vessel_rows
            if math.isfinite(row["first_arrival_s"])
        ) / max(1, sum(
            math.isfinite(row["first_arrival_s"]) for row in vessel_rows)),
        "mission_arrival_time_6m_s": mission_arrival_time,
        "standardized_evaluation_duration_s": evaluation_end - run_start,
        "standardized_minimum_pair_distance_m": standardized_minimum_pair_distance,
        "standardized_pair_distance_p05_m": pair_p05,
        "standardized_safety_margin_m": (
            standardized_minimum_pair_distance - VESSEL_LENGTH_M),
        "standardized_overlap_pair_samples_under_5p4m": standardized_overlap_samples,
        "standardized_overlap_pair_seconds_under_5p4m": (
            standardized_overlap_samples * 0.1),
        "standardized_near_miss_pair_seconds_under_8m": (
            standardized_near_miss_samples * 0.1),
        "standardized_caution_pair_seconds_under_10m": (
            standardized_caution_samples * 0.1),
        "standardized_risk_exposure_m_s": standardized_risk_exposure * 0.1,
        "standardized_mean_path_length_m": standardized_path_mean,
        "standardized_mean_excess_path_m": (
            standardized_path_mean - achieved_progress_mean),
        "standardized_path_efficiency": (
            achieved_progress_mean / max(standardized_path_mean, 1e-9)),
        "standardized_mean_energy_proxy": statistics.mean(energy_proxies),
        "standardized_mean_control_variation": statistics.mean(control_variations),
        "standardized_mean_active_edge_count": statistics.mean(
            float(row.get("active_edge_count", 0.0)) for row in standardized_rows),
        "standardized_yield_fraction": statistics.mean(
            int(row.get("yielding", 0)) for row in standardized_rows),
        "minimum_pair_distance_m": minimum_pair_distance,
        "closest_pair": "/".join(closest_pair),
        "overlap_pair_samples_under_5p4m": overlap_pair_samples,
        # Summed across all vessel pairs, so this is pair-seconds rather than
        # wall-clock duration. Simultaneous overlaps contribute independently.
        "overlap_pair_seconds_under_5p4m": overlap_pair_samples * 0.1,
        "mean_path_length_m": sum(row["path_length_m"] for row in vessel_rows)
        / len(vessel_rows),
        "complete_frames": complete_frames,
    }
    return summary, vessel_rows


def write_csv(path, rows):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dirs", nargs="+")
    parser.add_argument("--output-dir", default="results/comparison_latest")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    summaries = []
    details = []
    for run_dir in args.run_dirs:
        summary, vessels = summarize(run_dir)
        summaries.append(summary)
        for vessel in vessels:
            details.append({"run": summary["run"], **vessel})

    write_csv(output_dir / "run_summary.csv", summaries)
    write_csv(output_dir / "vessel_summary.csv", details)
    for row in summaries:
        print(
            f"{row['control_mode']:9s}/{row['graph_mode']:24s} "
            f"arrived6m={row['arrival_success_count_6m']}/"
            f"{row['vessels']} final5m={row['finally_at_goal_count']}/"
            f"{row['vessels']} ever={row['ever_reached_count']}/{row['vessels']} "
            f"dwell={row['residence_success_count']}/{row['vessels']} "
            f"min_pair={row['minimum_pair_distance_m']:.3f} m "
            f"overlap_samples={row['overlap_pair_samples_under_5p4m']} "
            f"duration={row['duration_s']:.1f} s "
            f"mean_path={row['mean_path_length_m']:.1f} m"
        )
    print(output_dir / "run_summary.csv")


if __name__ == "__main__":
    main()
