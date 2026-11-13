#!/usr/bin/env python3
"""Summarize matched harbor runs on the current shuttle map."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path


ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
METHODS = (
    "distance_star",
    "risk_star",
    "conditional_csd",
    "gnn_gru",
    "gnn_gru_sac",
)
SEA_STATES = (
    "calm",
    "medium_wind_wave",
    "strong_wind_wave",
    "extreme_wind_wave",
)
SEA_LABELS = {
    "calm": "Calm",
    "medium_wind_wave": "Medium wind/wave",
    "strong_wind_wave": "Strong wind/wave",
    "extreme_wind_wave": "Extreme wind/wave",
}
METHOD_LABELS = {
    "distance_star": "Distance-Star",
    "risk_star": "Risk-Star",
    "conditional_csd": "Conditional CSD",
    "gnn_gru": "GNN-GRU",
    "gnn_gru_sac": "GNN-GRU + SAC",
}
COOPERATIVE = ("wamv_1", "wamv_2", "wamv_3", "wamv_4")
ALL_VESSELS = COOPERATIVE + ("wamv_5", "wamv_6")
WALLS = (((-11.0, 13.0), (7.0, 3.0)), ((12.0, -13.0), (7.0, 3.0)))
HULL_HALF_LENGTH = 2.7
PAIR_PROXIMITY_PROXY = 5.4


def number(row, key, default=math.nan):
    try:
        value = float(row.get(key, default))
    except (TypeError, ValueError):
        return default
    return value


def flag(row, key):
    return number(row, key, 0.0) >= 0.5


def read_rows(path: Path):
    try:
        with path.open(newline="", encoding="utf-8") as handle:
            return list(csv.DictReader(handle))
    except (FileNotFoundError, OSError, csv.Error):
        return []


def finite(values):
    return [value for value in values if math.isfinite(value)]


def path_length(rows):
    ordered = sorted(rows, key=lambda row: number(row, "time", 0.0))
    total = 0.0
    previous = None
    for row in ordered:
        point = (number(row, "x"), number(row, "y"))
        if not all(math.isfinite(value) for value in point):
            continue
        if previous is not None:
            step = math.hypot(point[0] - previous[0], point[1] - previous[1])
            # Reject pose resets from a failed startup mixed into one file.
            if step < 10.0:
                total += step
        previous = point
    return total


def point_to_box_distance(x, y, center, half_size):
    dx = max(abs(x - center[0]) - half_size[0], 0.0)
    dy = max(abs(y - center[1]) - half_size[1], 0.0)
    return math.hypot(dx, dy)


@dataclass
class RunSummary:
    method: str
    label: str
    status: str
    run_dir: str
    samples: int = 0
    duration_s: float = math.nan
    vessels_logged: int = 0
    arrivals: int = 0
    residences: int = 0
    fleet_arrival_s: float = math.nan
    fleet_residence_s: float = math.nan
    min_pair_center_distance_m: float = math.nan
    pair_proximity_proxy: bool = False
    min_wall_center_distance_m: float = math.nan
    approx_wall_clearance_m: float = math.nan
    wall_overlap_proxy: bool = False
    mean_cooperative_path_m: float = math.nan
    patrol_min_span_m: float = math.nan
    residual_accepted: int = 0
    residual_eligible: int = 0


def summarize_csv(path: Path, method: str, minimum_duration: float = 170.0):
    rows = read_rows(path)
    summary = RunSummary(
        method=method,
        label=METHOD_LABELS[method],
        status="missing" if not rows else "incomplete",
        run_dir=str(path.parent),
        samples=len(rows),
    )
    if not rows:
        return summary

    by_vessel = {
        name: [row for row in rows if row.get("vessel") == name]
        for name in ALL_VESSELS
    }
    summary.vessels_logged = sum(bool(vessel_rows) for vessel_rows in by_vessel.values())
    times = finite(number(row, "time") for row in rows)
    if times:
        start = min(times)
        summary.duration_s = max(times) - start
    else:
        start = 0.0

    arrival_times = []
    residence_times = []
    paths = []
    for name in COOPERATIVE:
        vessel_rows = by_vessel[name]
        paths.append(path_length(vessel_rows))
        arrival = [number(row, "time") for row in vessel_rows if flag(row, "arrival_qualified")]
        residence = [number(row, "time") for row in vessel_rows if flag(row, "residence_qualified")]
        if arrival:
            summary.arrivals += 1
            arrival_times.append(min(arrival))
        if residence:
            summary.residences += 1
            residence_times.append(min(residence))
    if len(arrival_times) == len(COOPERATIVE):
        summary.fleet_arrival_s = max(arrival_times) - start
    if len(residence_times) == len(COOPERATIVE):
        summary.fleet_residence_s = max(residence_times) - start
    if paths:
        summary.mean_cooperative_path_m = sum(paths) / len(paths)

    pair_distances = finite(number(row, "nearest_vessel_distance") for row in rows)
    if pair_distances:
        summary.min_pair_center_distance_m = min(pair_distances)
        summary.pair_proximity_proxy = (
            summary.min_pair_center_distance_m < PAIR_PROXIMITY_PROXY
        )

    wall_distances = []
    for row in rows:
        x, y = number(row, "x"), number(row, "y")
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        wall_distances.extend(
            point_to_box_distance(x, y, center, half_size)
            for center, half_size in WALLS
        )
    if wall_distances:
        summary.min_wall_center_distance_m = min(wall_distances)
        summary.approx_wall_clearance_m = (
            summary.min_wall_center_distance_m - HULL_HALF_LENGTH
        )
        summary.wall_overlap_proxy = summary.approx_wall_clearance_m < 0.0

    patrol_spans = []
    for name in ("wamv_5", "wamv_6"):
        ys = finite(number(row, "y") for row in by_vessel[name])
        if ys:
            patrol_spans.append(max(ys) - min(ys))
    if patrol_spans:
        summary.patrol_min_span_m = min(patrol_spans)

    summary.residual_accepted = sum(flag(row, "residual_accepted") for row in rows)
    summary.residual_eligible = sum(flag(row, "residual_eligible") for row in rows)

    # A run is terminal once all vessels have arrived and the configured
    # residence observation window has elapsed.  Residence is an outcome,
    # not a prerequisite for considering the experiment data complete.
    complete_by_arrival_observation = (
        summary.arrivals == len(COOPERATIVE)
        and math.isfinite(summary.fleet_arrival_s)
        and summary.duration_s >= summary.fleet_arrival_s + 9.0
    )
    complete_by_timeout = summary.duration_s >= minimum_duration
    summary.status = (
        "complete"
        if summary.vessels_logged == 6
        and (complete_by_arrival_observation or complete_by_timeout)
        else "incomplete"
    )
    return summary


def candidate_summaries(
    method: str,
    seed: str,
    sea_state: str = "calm",
    minimum_duration: float = 170.0,
):
    pattern = f"harbor_{method}_{sea_state}_seed{seed}_*/six_vessel_driver.csv"
    return [
        summarize_csv(path, method, minimum_duration)
        for path in RESULTS.glob(pattern)
    ]


def best_summary(
    method: str,
    seed: str,
    sea_state: str = "calm",
    minimum_duration: float = 170.0,
):
    summaries = candidate_summaries(
        method, seed, sea_state, minimum_duration
    )
    if not summaries:
        return RunSummary(method, METHOD_LABELS[method], "missing", "")

    def score(item):
        return (
            item.status == "complete",
            item.residences,
            item.arrivals,
            item.vessels_logged,
            item.duration_s if math.isfinite(item.duration_s) else -1.0,
            Path(item.run_dir).stat().st_mtime,
        )

    return max(summaries, key=score)


def display(value, digits=2):
    if isinstance(value, bool):
        return "Yes" if value else "No"
    if isinstance(value, float):
        return f"{value:.{digits}f}" if math.isfinite(value) else "-"
    return str(value)


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


def write_outputs(summaries, seed: str, sea_state: str = "calm"):
    output_dir = RESULTS / f"{sea_state}_horizontal_seed{seed}"
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "comparison_summary.csv"
    json_path = output_dir / "comparison_summary.json"
    md_path = output_dir / "comparison_summary.md"

    fields = list(asdict(summaries[0]).keys())
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(asdict(item) for item in summaries)
    json_path.write_text(
        json.dumps(
            json_safe([asdict(item) for item in summaries]),
            indent=2,
            allow_nan=False,
        ),
        encoding="utf-8",
    )

    lines = [
        f"# {SEA_LABELS[sea_state]} horizontal comparison",
        "",
        f"Matched current shuttle map, sea state `{sea_state}`, random seed `{seed}`, "
        "four cooperative WAM-Vs, "
        "two reversing patrol WAM-Vs, and two rectangular static obstacles.",
        "",
        "| Method | Status | Arrival | Residence | Fleet arrival (s) | Min pair center (m) | Pair <5.4 m | Min wall center (m) | Approx wall clearance (m) | Mean path (m) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in summaries:
        lines.append(
            f"| {item.label} | {item.status} | {item.arrivals}/4 | {item.residences}/4 | "
            f"{display(item.fleet_arrival_s)} | {display(item.min_pair_center_distance_m)} | "
            f"{display(item.pair_proximity_proxy)} | "
            f"{display(item.min_wall_center_distance_m)} | "
            f"{display(item.approx_wall_clearance_m)} | "
            f"{display(item.mean_cooperative_path_m)} |"
        )
    lines.extend([
        "",
        "## Interpretation limits",
        "",
        "- `Pair <5.4 m` is a conservative sampled center-distance proxy, not a contact event.",
        "- Wall clearance subtracts the WAM-V half-length (2.7 m) from point-to-wall-box distance; it is an approximate geometric proxy.",
        "- Formal collision claims still require Gazebo contact logging or full oriented-hull intersection logging.",
        "- Only runs from the current reversing-shuttle map and the same seed are included; historical single-pass runs are intentionally excluded.",
    ])
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return csv_path, md_path, json_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", default="20261011")
    parser.add_argument("--sea-state", choices=SEA_STATES, default="calm")
    parser.add_argument("--minimum-duration", type=float)
    parser.add_argument("--is-finished", choices=METHODS)
    parser.add_argument("--print-table", action="store_true")
    args = parser.parse_args()
    minimum_duration = args.minimum_duration
    if minimum_duration is None:
        minimum_duration = 170.0 if args.sea_state == "calm" else 230.0

    if args.is_finished:
        result = best_summary(
            args.is_finished, args.seed, args.sea_state, minimum_duration
        )
        print(f"{args.is_finished}: {result.status} ({result.run_dir or 'no run'})")
        raise SystemExit(0 if result.status == "complete" else 1)

    summaries = [
        best_summary(method, args.seed, args.sea_state, minimum_duration)
        for method in METHODS
    ]
    csv_path, md_path, json_path = write_outputs(
        summaries, args.seed, args.sea_state
    )
    if args.print_table:
        print((md_path).read_text(encoding="utf-8"))
    print(f"CSV: {csv_path}")
    print(f"Markdown: {md_path}")
    print(f"JSON: {json_path}")


if __name__ == "__main__":
    main()
