#!/usr/bin/env python3
import csv
import statistics
import sys
from collections import defaultdict

rows = list(csv.DictReader(open(sys.argv[1]), delimiter="\t"))
groups = defaultdict(list)
for row in rows:
    groups[row["seed"]].append(row)

print("C119 medium matched 5-seed summary")
print("seed       strict  safe  arrival  time mean (range)")
for seed in sorted(groups):
    group = groups[seed]
    strict = [row for row in group if row["status"] == "PASS"]
    safe = [row for row in group if row["status"] in ("PASS", "SAFE_SLOW")]
    arrival = [row for row in group if row["arrival"] == "4"]
    times = [float(row["fleet_time_s"]) for row in arrival if row["fleet_time_s"]]
    timing = "NA"
    if times:
        timing = f"{statistics.mean(times):.2f}s ({min(times):.2f}-{max(times):.2f})"
    print(f"{seed}  {len(strict):2d}/{len(group):2d}  {len(safe):2d}/{len(group):2d}  "
          f"{len(arrival):2d}/{len(group):2d}  {timing}")
