#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import math
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SEED = sys.argv[1] if len(sys.argv) > 1 else "20261014"
SEAS = ("calm", "medium_wind_wave", "strong_wind_wave", "extreme_wind_wave")
METHODS = ("distance_star", "risk_star", "conditional_csd", "gnn_gru")

spec = importlib.util.spec_from_file_location(
    "harbor_summary", ROOT / "summarize_calm_horizontal.py"
)
summary_module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = summary_module
spec.loader.exec_module(summary_module)


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


while True:
    completed = 0
    lines = []
    for sea in SEAS:
        sea_done = 0
        details = []
        minimum = {
            "calm": 230.0,
            "medium_wind_wave": 290.0,
            "strong_wind_wave": 290.0,
            "extreme_wind_wave": 350.0,
        }[sea]
        for method in METHODS:
            item = summary_module.best_summary(method, SEED, sea, minimum)
            done = item.status == "complete"
            completed += int(done)
            sea_done += int(done)
            if done:
                arrival = f"{item.arrivals}/4"
                pair = (
                    f"{item.min_pair_center_distance_m:.2f}m"
                    if math.isfinite(item.min_pair_center_distance_m)
                    else "-"
                )
                details.append(f"{method}:{arrival},d={pair}")
        lines.append(f"{sea}: {sea_done}/4 " + " ".join(details))

    width = 40
    fill = int(width * completed / 16)
    bar = "#" * fill + "-" * (width - fill)
    current = read_text(ROOT / ".matched_baselines_current") or "waiting"
    print("\033[2J\033[H", end="")
    print(f"Matched baselines [{bar}] {completed}/16  current={current}")
    print("\n".join(lines))

    pid_text = read_text(ROOT / ".matched_baselines.pid")
    running = False
    if pid_text.isdigit():
        try:
            Path(f"/proc/{pid_text}").stat()
            running = True
        except OSError:
            pass
    if (ROOT / ".matched_baselines_complete").exists() or completed == 16:
        print("All matched baseline runs are complete.")
        break
    if not running:
        print("The batch is not running. Saved runs remain; restart the start script to resume.")
        break
    time.sleep(5)
