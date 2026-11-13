#!/usr/bin/env python3

import argparse
import csv
import math
import time
from collections import defaultdict

import numpy as np
from gz.msgs10 import boolean_pb2, pose_pb2
from gz.transport13 import Node


VESSELS = tuple(f"wamv_{index}" for index in range(1, 7))


def load_tracks(csv_path):
    tracks = defaultdict(list)
    with open(csv_path, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            name = row["vessel"]
            if name not in VESSELS:
                continue
            tracks[name].append((
                float(row["time"]),
                float(row["x"]),
                float(row["y"]),
                float(row["heading"]),
                int(row["arrival_qualified"]),
            ))

    missing = [name for name in VESSELS if not tracks[name]]
    if missing:
        raise RuntimeError(f"CSV missing vessel tracks: {', '.join(missing)}")

    origin = min(samples[0][0] for samples in tracks.values())
    prepared = {}
    for name, samples in tracks.items():
        samples.sort(key=lambda item: item[0])
        prepared[name] = {
            "time": np.asarray([item[0] - origin for item in samples]),
            "x": np.asarray([item[1] for item in samples]),
            "y": np.asarray([item[2] for item in samples]),
            "heading": np.unwrap(np.asarray([item[3] for item in samples])),
            "arrival": np.asarray([item[4] for item in samples]),
        }
    return prepared


def sample_track(track, elapsed):
    return (
        float(np.interp(elapsed, track["time"], track["x"])),
        float(np.interp(elapsed, track["time"], track["y"])),
        float(np.interp(elapsed, track["time"], track["heading"])),
    )


def set_pose(node, service, name, x, y, yaw, z):
    request = pose_pb2.Pose()
    request.name = name
    request.position.x = x
    request.position.y = y
    request.position.z = z
    request.orientation.z = math.sin(0.5 * yaw)
    request.orientation.w = math.cos(0.5 * yaw)
    executed, response = node.request(
        service, request, pose_pb2.Pose, boolean_pb2.Boolean, 1000)
    return bool(executed and response.data)


def main():
    parser = argparse.ArgumentParser(
        description="Replay the immutable historical C05 trajectory in Gazebo.")
    parser.add_argument("--csv", required=True)
    parser.add_argument("--world", default="csd_harbor")
    parser.add_argument("--rate", type=float, default=8.0)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--hold-seconds", type=float, default=30.0)
    # ROS 2 launch appends its own arguments (for example ``--ros-args``)
    # even though this helper talks to Gazebo Transport directly.
    args, _ros_arguments = parser.parse_known_args()

    tracks = load_tracks(args.csv)
    end_time = max(track["time"][-1] for track in tracks.values())
    service = f"/world/{args.world}/set_pose"
    node = Node()

    print("=" * 72, flush=True)
    print("C05 HISTORICAL TRAJECTORY REPLAY (not a new controller run)", flush=True)
    print("Recorded fleet completion: 107.811 s", flush=True)
    print("Recorded run duration: 117.953 s", flush=True)
    print(f"Source CSV: {args.csv}", flush=True)
    print("=" * 72, flush=True)

    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        first = tracks[VESSELS[0]]
        if set_pose(
                node, service, VESSELS[0], first["x"][0], first["y"][0],
                first["heading"][0], args.z):
            break
        time.sleep(0.5)
    else:
        raise RuntimeError(f"Gazebo pose service unavailable: {service}")

    period = 1.0 / max(args.rate, 1.0)
    start = time.monotonic()
    last_status_second = -1
    while True:
        elapsed = min((time.monotonic() - start) * args.speed, end_time)
        failures = []
        for name in VESSELS:
            x, y, yaw = sample_track(tracks[name], elapsed)
            if not set_pose(node, service, name, x, y, yaw, args.z):
                failures.append(name)
        if failures:
            raise RuntimeError("Failed to update: " + ", ".join(failures))

        status_second = int(elapsed)
        if status_second != last_status_second:
            arrived = sum(
                int(np.any(track["arrival"][track["time"] <= elapsed] > 0))
                for name, track in tracks.items() if name in VESSELS[:4]
            )
            print(
                f"Historical replay: {elapsed:7.3f}/{end_time:7.3f} s, "
                f"cooperative arrivals={arrived}/4",
                flush=True,
            )
            last_status_second = status_second

        if elapsed >= end_time:
            break
        sleep_for = period - ((time.monotonic() - start) % period)
        time.sleep(max(0.001, sleep_for))

    print("Historical C05 replay complete: fleet completion = 107.811 s", flush=True)
    print(f"Holding final GUI frame for {args.hold_seconds:.0f} seconds.", flush=True)
    time.sleep(max(0.0, args.hold_seconds))


if __name__ == "__main__":
    main()
