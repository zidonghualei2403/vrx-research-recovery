#!/usr/bin/env python3
import csv
import json
import sys
from pathlib import Path

batch = Path(sys.argv[1])
batch.mkdir(parents=True, exist_ok=True)
status_path = batch / "status.csv"
output = batch / "comparison_summary.csv"
fields = [
    "sea_state", "seed", "method", "arrivals", "residences",
    "fleet_completion_time_s", "vessel_overlap_frames", "wall_overlap_frames",
    "minimum_vessel_clearance_m", "minimum_wall_clearance_m", "passed", "run_dir",
]
rows_out = []
if status_path.exists():
    with status_path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    latest = {}
    for row in rows:
        latest[(row["sea_state"], row["seed"], row["method"])] = row
    for (sea, seed, method), row in sorted(latest.items()):
        if row["status"] != "DONE":
            continue
        run_dir = Path(row["run_dir"])
        metrics_path = run_dir / "body_audit" / "metrics.json"
        if not metrics_path.exists():
            continue
        metrics = json.loads(metrics_path.read_text())
        rows_out.append({
            "sea_state": sea,
            "seed": seed,
            "method": method,
            "arrivals": metrics.get("arrival_count"),
            "residences": metrics.get("residence_count"),
            "fleet_completion_time_s": metrics.get("fleet_completion_s"),
            "vessel_overlap_frames": metrics.get("vessel_overlap_frames"),
            "wall_overlap_frames": metrics.get("wall_overlap_frames"),
            "minimum_vessel_clearance_m": metrics.get("minimum_vessel_clearance_m"),
            "minimum_wall_clearance_m": metrics.get("minimum_wall_clearance_m"),
            "passed": metrics.get("passed"),
            "run_dir": str(run_dir),
        })
with output.open("w", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=fields)
    writer.writeheader()
    writer.writerows(rows_out)
print(output)
