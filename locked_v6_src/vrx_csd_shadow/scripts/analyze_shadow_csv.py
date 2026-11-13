#!/usr/bin/python3
import csv
import sys
from collections import Counter
from pathlib import Path


script_path = Path(__file__).resolve()
workspace = next(
    (parent for parent in script_path.parents if parent.name == "vrx_csd_shadow_ws_v1"),
    None,
)
if workspace is None:
    raise SystemExit("Cannot locate vrx_csd_shadow_ws_v1 from the script path.")
if len(sys.argv) > 1:
    run_dir = Path(sys.argv[1]).expanduser().resolve()
else:
    runs = sorted((workspace / "results").glob("shadow_*"), key=lambda p: p.stat().st_mtime)
    if not runs:
        raise SystemExit("No shadow result directory found.")
    run_dir = runs[-1]

edge_path = run_dir / "shadow_edges.csv"
state_path = run_dir / "shadow_states.csv"
if not edge_path.is_file() or not state_path.is_file():
    raise SystemExit(f"Incomplete shadow output: {run_dir}")

with edge_path.open(newline="") as handle:
    edges = list(csv.DictReader(handle))
with state_path.open(newline="") as handle:
    states = list(csv.DictReader(handle))

entities = sorted({row["name"] for row in states})
selected = [row for row in edges if row["selected"] == "1"]
hard = [row for row in edges if row["hard_safety"] == "1"]
pair_counts = Counter((row["ego"], row["neighbor"]) for row in selected)
clearances = [float(row["predicted_clearance"]) for row in edges]
risks = [float(row["calibrated_risk"]) for row in edges]

print(f"run_dir={run_dir}")
print(f"state_rows={len(states)} edge_rows={len(edges)}")
print(f"entities={entities}")
print(f"selected_rows={len(selected)} hard_safety_rows={len(hard)}")
if clearances:
    print(f"minimum_predicted_clearance={min(clearances):.3f}")
    print(f"maximum_calibrated_risk={max(risks):.3f}")
print("most_frequent_selected_edges:")
for (ego, neighbor), count in pair_counts.most_common(12):
    print(f"  {ego} -> {neighbor}: {count}")

if len(entities) < 2 or not edges:
    raise SystemExit("FAIL: fewer than two fresh entities reached the shadow node.")
print("PASS: shadow state and directed-edge logs are populated.")
