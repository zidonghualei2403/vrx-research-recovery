#!/usr/bin/env python3
"""Reject VRX runs contaminated by stale poses or discontinuous trajectories."""

import argparse
import csv
import json
import math
from pathlib import Path


CENTRAL_STARTS = {
    "wamv_1": (45.77, -0.22),
    "wamv_2": (23.08, 39.72),
    "wamv_3": (-22.70, 39.94),
    "wamv_4": (-45.77, 0.22),
    "wamv_5": (-23.08, -39.72),
    "wamv_6": (22.70, -39.94),
}

HARBOR_STARTS = {
    "wamv_1": (-48.71, -22.43),
    "wamv_2": (-48.71, 22.43),
    "wamv_3": (48.78, -8.17),
    "wamv_4": (48.78, 8.17),
    "wamv_5": (-24.97, -20.91),
    "wamv_6": (24.97, 20.91),
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--start-tolerance", type=float, default=4.0)
    parser.add_argument("--max-step", type=float, default=2.0)
    parser.add_argument("--minimum-samples", type=int, default=60)
    parser.add_argument("--json-out")
    parser.add_argument("--encounter", choices=("central", "harbor"),
                        default="central")
    args = parser.parse_args()
    expected_starts = HARBOR_STARTS if args.encounter == "harbor" else CENTRAL_STARTS

    run_dir = Path(args.run_dir)
    csv_path = run_dir / "six_vessel_driver.csv"
    reasons = []
    diagnostics = {
        "run_dir": str(run_dir),
        "csv": str(csv_path),
        "valid": False,
        "start_tolerance_m": args.start_tolerance,
        "maximum_allowed_step_m": args.max_step,
        "minimum_samples_per_vessel": args.minimum_samples,
        "vessels": {},
        "reasons": reasons,
    }

    if not csv_path.exists():
        reasons.append("missing_six_vessel_driver_csv")
    else:
        with csv_path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        by_vessel = {}
        for row in rows:
            by_vessel.setdefault(row.get("vessel", ""), []).append(row)

        missing = sorted(set(expected_starts) - set(by_vessel))
        if missing:
            reasons.append("missing_vessels:" + ",".join(missing))

        for vessel, expected in expected_starts.items():
            samples = by_vessel.get(vessel, [])
            vessel_diag = {"samples": len(samples)}
            diagnostics["vessels"][vessel] = vessel_diag
            if len(samples) < args.minimum_samples:
                reasons.append(f"too_few_samples:{vessel}:{len(samples)}")
                continue

            points = [(float(row["x"]), float(row["y"])) for row in samples]
            start_error = math.hypot(
                points[0][0] - expected[0], points[0][1] - expected[1])
            steps = [
                math.hypot(bx - ax, by - ay)
                for (ax, ay), (bx, by) in zip(points, points[1:])
            ]
            max_step = max(steps, default=0.0)
            jump_count = sum(step > args.max_step for step in steps)
            vessel_diag.update({
                "initial_x": points[0][0],
                "initial_y": points[0][1],
                "expected_x": expected[0],
                "expected_y": expected[1],
                "start_error_m": start_error,
                "maximum_step_m": max_step,
                "steps_over_limit": jump_count,
            })
            if start_error > args.start_tolerance:
                reasons.append(
                    f"stale_or_wrong_start:{vessel}:{start_error:.3f}m")
            if max_step > args.max_step:
                reasons.append(
                    f"trajectory_jump:{vessel}:{max_step:.3f}m")

    diagnostics["valid"] = not reasons
    output = json.dumps(diagnostics, ensure_ascii=False, indent=2)
    if args.json_out:
        out_path = Path(args.json_out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(output + "\n", encoding="utf-8")
    print(output)
    raise SystemExit(0 if diagnostics["valid"] else 1)


if __name__ == "__main__":
    main()
