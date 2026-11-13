#!/usr/bin/env python3
"""Export compact, auditable run tables from the original VRX backup."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


OLD_ROOT = "/home/hzh/Documents/Codex/2026-08-13/hao/vrx_harbor_sac_v6_aligned_ws_v1/"
CALM_COLUMNS = [
    "method", "campaign", "seed", "repeat", "status", "arrival_count",
    "fleet_time_s", "vessel_overlap_frames", "wall_overlap_frames",
    "stall20_count", "strict_safe", "strict_success_130s",
    "minimum_vessel_clearance_m", "minimum_wall_clearance_m",
    "run_dir_relative",
]


def rows(path: Path, delimiter: str = ",") -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle, delimiter=delimiter))


def relative_run_dir(value: str) -> str:
    if not value.startswith(OLD_ROOT):
        raise ValueError(f"Unexpected run path: {value}")
    return value[len(OLD_ROOT):]


def valid_time(value: object) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(value)


def v6_rows(root: Path) -> list[dict[str, object]]:
    campaigns = [
        ("locked_v6_calm_5seed_10repeat_20261102", "v6_main50"),
        ("locked_v6_calm_extra50_20261108", "v6_extra50"),
    ]
    result = []
    for folder, campaign in campaigns:
        source = rows(root / folder / "results.tsv", "\t")
        if len(source) != 50:
            raise ValueError(f"Expected 50 V6 rows in {folder}, found {len(source)}")
        for item in source:
            arrival = int(item["arrival"])
            vessel_overlap = int(item["ship_overlap"])
            wall_overlap = int(item["wall_overlap"])
            stalls = int(item["stall20_count"])
            fleet_time = float(item["fleet_time_s"])
            strict_safe = arrival == 4 and vessel_overlap == 0 and wall_overlap == 0 and stalls == 0
            result.append({
                "method": "locked_v6_gnn_gru_sac",
                "campaign": campaign,
                "seed": item["seed"],
                "repeat": item["repeat"],
                "status": item["status"],
                "arrival_count": arrival,
                "fleet_time_s": item["fleet_time_s"],
                "vessel_overlap_frames": vessel_overlap,
                "wall_overlap_frames": wall_overlap,
                "stall20_count": stalls,
                "strict_safe": int(strict_safe),
                "strict_success_130s": int(strict_safe and fleet_time <= 130.0),
                "minimum_vessel_clearance_m": item["min_ship_gap_m"],
                "minimum_wall_clearance_m": item["min_wall_gap_m"],
                "run_dir_relative": relative_run_dir(item["run_dir"]),
            })
    return result


def baseline_rows(root: Path) -> list[dict[str, object]]:
    campaign = "calm_baselines_5seed_10repeat_gui_20261103"
    status_path = root / "results" / campaign / "status.csv"
    completed = [item for item in rows(status_path) if item["status"] == "DONE"]
    if len(completed) != 400:
        raise ValueError(f"Expected 400 completed calm baseline runs, found {len(completed)}")
    seen: set[tuple[str, str, str]] = set()
    result = []
    for item in completed:
        key = (item["method"], item["seed"], item["repeat"])
        if key in seen:
            raise ValueError(f"Duplicate completed baseline run: {key}")
        seen.add(key)
        relative = relative_run_dir(item["run_dir"])
        metrics_path = root / relative / "body_audit" / "metrics.json"
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        arrival = int(metrics["arrival_count"])
        vessel_overlap = int(metrics["vessel_overlap_frames"])
        wall_overlap = int(metrics["wall_overlap_frames"])
        stalls = len(metrics["distant_stalls_20s"])
        fleet_time = metrics.get("fleet_completion_s")
        strict_safe = (
            bool(metrics["six_vessels_valid"])
            and arrival == 4
            and vessel_overlap == 0
            and wall_overlap == 0
            and stalls == 0
        )
        result.append({
            "method": item["method"],
            "campaign": campaign,
            "seed": item["seed"],
            "repeat": item["repeat"],
            "status": item["status"],
            "arrival_count": arrival,
            "fleet_time_s": "" if not valid_time(fleet_time) else fleet_time,
            "vessel_overlap_frames": vessel_overlap,
            "wall_overlap_frames": wall_overlap,
            "stall20_count": stalls,
            "strict_safe": int(strict_safe),
            "strict_success_130s": int(strict_safe and valid_time(fleet_time) and fleet_time <= 130.0),
            "minimum_vessel_clearance_m": metrics["minimum_vessel_clearance_m"],
            "minimum_wall_clearance_m": metrics["minimum_wall_clearance_m"],
            "run_dir_relative": relative,
        })
    return result


def write_csv(path: Path, data: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CALM_COLUMNS)
        writer.writeheader()
        writer.writerows(data)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, required=True, help="Original VRX project directory")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "data_export" / "calm_500_runs.csv")
    args = parser.parse_args()
    root = args.project_root.resolve()
    data = baseline_rows(root) + v6_rows(root)
    data.sort(key=lambda row: (str(row["method"]), str(row["campaign"]), int(row["seed"]), int(row["repeat"])))
    write_csv(args.output, data)
    print(f"Wrote {len(data)} runs to {args.output}")


if __name__ == "__main__":
    main()
