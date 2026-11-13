#!/usr/bin/env python3
import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Rectangle
import numpy as np


STARTS = {
    "wamv_1": (-48.0, -22.0),
    "wamv_2": (-48.0, 22.0),
    "wamv_3": (48.0, -8.0),
    "wamv_4": (48.0, 8.0),
    "wamv_5": (-25.0, -20.0),
    "wamv_6": (25.0, 20.0),
}

GOALS = {
    "wamv_1": (48.0, 22.0),
    "wamv_2": (48.0, -22.0),
    "wamv_3": (-48.0, 8.0),
    "wamv_4": (-48.0, -8.0),
    "wamv_5": (-25.0, 20.0),
    "wamv_6": (25.0, -20.0),
}

ROUTES = {
    "wamv_1": [(-24.0, -11.0), (0.0, -4.0), (24.0, 8.0)],
    "wamv_2": [(-30.0, 20.0), (-14.0, 20.8), (6.0, 20.0),
               (16.0, 14.0), (22.0, 5.0), (32.0, 5.0), (38.0, 0.0)],
    "wamv_3": [(24.0, -5.0), (0.0, 2.0), (-24.0, 4.0)],
    "wamv_4": [(24.0, 8.0), (0.0, 4.0), (-24.0, -4.0)],
    "wamv_5": [GOALS["wamv_5"]],
    "wamv_6": [GOALS["wamv_6"]],
}

WALLS = [((-11.0, 13.0), (7.0, 3.0)), ((12.0, -13.0), (7.0, 3.0))]
COLORS = {
    "wamv_1": "#1478b8", "wamv_2": "#e69500", "wamv_3": "#159f78",
    "wamv_4": "#cb6ba8", "wamv_5": "#d95f02", "wamv_6": "#56b4e9",
}


def load_tracks(path):
    tracks = defaultdict(list)
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            tracks[row["vessel"]].append({
                "time": float(row["time"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "stage": int(float(row["harbor_route_stage"])),
                "yielding": int(float(row["yielding"])),
            })
    t0 = min(point["time"] for values in tracks.values() for point in values)
    for values in tracks.values():
        for point in values:
            point["time"] -= t0
    return tracks


def draw_walls(ax):
    for center, half_size in WALLS:
        ax.add_patch(Rectangle(
            (center[0] - half_size[0], center[1] - half_size[1]),
            2.0 * half_size[0], 2.0 * half_size[1],
            facecolor="#8c9199", edgecolor="#343a40", linewidth=1.3,
            alpha=0.75, zorder=1,
        ))


def setup_axis(ax, title):
    draw_walls(ax)
    ax.set_title(title, fontsize=12)
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.set_xlim(-56, 56)
    ax.set_ylim(-48, 48)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#d8dde3", linewidth=0.7, alpha=0.8)


def plot_all(ax, tracks):
    setup_axis(ax, "C64 harbor map: planned routes and recorded tracks")
    for name in sorted(tracks):
        color = COLORS[name]
        planned = np.asarray([STARTS[name], *ROUTES[name], GOALS[name]])
        actual = np.asarray([(p["x"], p["y"]) for p in tracks[name]])
        ax.plot(planned[:, 0], planned[:, 1], "--", color=color,
                linewidth=1.2, alpha=0.55)
        ax.plot(actual[:, 0], actual[:, 1], color=color, linewidth=2.3,
                label=name, zorder=3)
        ax.scatter(*STARTS[name], s=38, facecolors="white", edgecolors=color,
                   linewidths=1.5, zorder=4)
        ax.scatter(*GOALS[name], marker="x", s=48, color=color,
                   linewidths=1.8, zorder=4)
    ax.legend(ncol=2, fontsize=8, loc="lower left")


def plot_wamv2(ax, tracks):
    setup_axis(ax, "wamv_2 detail: time-colored actual track")
    name = "wamv_2"
    values = tracks[name]
    planned = np.asarray([STARTS[name], *ROUTES[name], GOALS[name]])
    ax.plot(planned[:, 0], planned[:, 1], "--o", color="#5f6368",
            linewidth=1.4, markersize=3.5, label="C64 planned route")
    for idx, (x, y) in enumerate(planned):
        label = "S" if idx == 0 else ("G" if idx == len(planned) - 1 else str(idx))
        ax.annotate(label, (x, y), xytext=(4, 4), textcoords="offset points",
                    fontsize=8, color="#30343b")

    points = np.asarray([(p["x"], p["y"]) for p in values])
    times = np.asarray([p["time"] for p in values])
    segments = np.stack([points[:-1], points[1:]], axis=1)
    collection = LineCollection(segments, cmap="viridis", linewidth=3.0)
    collection.set_array(times[:-1])
    ax.add_collection(collection)
    plt.colorbar(collection, ax=ax, fraction=0.046, pad=0.04,
                 label="elapsed time / s")

    stages = np.asarray([p["stage"] for p in values])
    changes = np.flatnonzero(np.r_[True, stages[1:] != stages[:-1]])
    ax.scatter(points[changes, 0], points[changes, 1], s=32,
               facecolors="white", edgecolors="#111", zorder=5,
               label="route-stage change")
    for idx in changes:
        ax.annotate(f"stage {stages[idx]}\n{times[idx]:.1f}s",
                    points[idx], xytext=(5, -18), textcoords="offset points",
                    fontsize=7)
    ax.legend(fontsize=8, loc="lower left")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    tracks = load_tracks(args.csv)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6.3), constrained_layout=True)
    plot_all(axes[0], tracks)
    plot_wamv2(axes[1], tracks)
    fig.suptitle(
        "Harbor navigation diagnostic (solid: recorded, dashed: planned)",
        fontsize=14,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=220, bbox_inches="tight")
    print(args.output)


if __name__ == "__main__":
    main()
