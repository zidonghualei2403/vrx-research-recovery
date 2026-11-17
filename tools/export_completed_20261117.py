#!/usr/bin/env python3
"""Archive only completed, local VRX cohorts with relative paths and checksums."""
from __future__ import annotations

import csv
import gzip
import hashlib
import json
import math
import shutil
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "data_export/vrx_completed_20261117"
HARBOR = Path("/home/hzh/Documents/Codex/2026-08-13/hao/vrx_harbor_sac_v6_aligned_ws_v1")
CENTRAL = Path("/home/hzh/Documents/Codex/2026-08-13/hao/vrx_central_sac_v7_unified_ws_v1")
SIX_BATCH = CENTRAL / "results/central_v7_matched_5seed_3sea_5method_20261117"
STRONG = HARBOR / "runtime_recovery_20261115/strong_baselines_20repeat_h53world_gui"
H53 = HARBOR / "h53_strong_5seed_20repeat_gui_20261116"
M54 = HARBOR / "medium_c119_failed14_m54_deepwall_20261116"
SEAS = ("calm", "medium_wind_wave", "strong_wind_wave")
SEEDS = ("20261101", "20261102", "20261110", "20261117", "20261123")
METHODS = ("distance_star", "risk_star", "conditional_csd",
           "learned_gnn_gru_graph_shield", "gnn_gru_residual_sac")


def read_rows(path: Path, delimiter: str = ",") -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream, delimiter=delimiter))


def write_rows(path: Path, rows: list[dict], delimiter: str = ",") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        raise ValueError(f"Empty table: {path}")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), delimiter=delimiter)
        writer.writeheader()
        writer.writerows(rows)


def safe_source(value: str, root: Path) -> Path:
    path = Path(value.strip()).resolve(strict=True)
    if not path.is_relative_to(root.resolve(strict=True)):
        raise ValueError(f"Run outside local workspace: {path}")
    if not (path / "six_vessel_driver.csv").is_file():
        raise ValueError(f"Missing complete trajectory: {path}")
    return path


def copy_file(source: Path, dest: Path) -> str:
    if not source.is_file() or source.is_symlink():
        raise ValueError(f"Missing or linked source: {source}")
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, dest)
    return str(dest.relative_to(OUT))


def compress_file(source: Path, dest: Path) -> tuple[str, str, int]:
    if not source.is_file() or source.is_symlink():
        raise ValueError(f"Missing or linked source: {source}")
    dest.parent.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256()
    size = 0
    with source.open("rb") as src, dest.open("wb") as raw:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0,
                           compresslevel=6) as target:
            for block in iter(lambda: src.read(1024 * 1024), b""):
                digest.update(block)
                size += len(block)
                target.write(block)
    return str(dest.relative_to(OUT)), digest.hexdigest(), size


def archive_run(cohort: str, method: str, sea: str, seed: str,
                repeat: str, source: Path, audit: Path | None = None,
                six: bool = False) -> dict:
    dest = OUT / cohort / sea / method / f"seed{seed}" / f"repeat{int(repeat):02d}"
    trajectory, digest, size = compress_file(
        source / "six_vessel_driver.csv", dest / "six_vessel_driver.csv.gz")
    config = copy_file(source / "experiment_config.csv", dest / "experiment_config.csv")
    world = copy_file(source / ("csd_six_vessel.sdf" if six else "csd_harbor.sdf"),
                      dest / ("csd_six_vessel.sdf" if six else "csd_harbor.sdf"))
    audit_rel = ""
    if audit is not None and audit.is_file():
        audit_rel = copy_file(audit, dest / audit.name)
    if six:
        validation = json.loads((source / "start_validation.json").read_text())
        if not validation.get("valid"):
            raise ValueError(f"Six-vessel validation failed: {source}")
        for name in ("shadow_edges.csv", "shadow_states.csv"):
            compress_file(source / name, dest / (name + ".gz"))
    return dict(cohort=cohort, sea_state=sea, method=method, seed=seed,
                repeat=repeat, source_run_name=source.name,
                trajectory=trajectory, trajectory_sha256=digest,
                trajectory_bytes=size, config=config, world=world,
                audit=audit_rel)


def normalized(rows: list[dict], keys: tuple[str, ...], source_root: Path) -> list[dict]:
    output = []
    for row in rows:
        clean = dict(row)
        for key in keys:
            if clean.get(key):
                clean[key] = safe_source(clean[key], source_root).name
        output.append(clean)
    return output


def oriented_hull_audit(path: Path) -> tuple[float, int, int]:
    """Sampled 2.7 m × 1.35 m half-hulls using four separating axes."""
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) % 6:
        raise ValueError(f"Incomplete six-vessel frame: {path}")
    expected_names = tuple(f"wamv_{i}" for i in range(1, 7))
    for start in range(0, len(rows), 6):
        block = rows[start:start + 6]
        if tuple(r["vessel"] for r in block) != expected_names or len({r["time"] for r in block}) != 1:
            raise ValueError(f"Unsynchronized vessel frame at row {start}: {path}")
    position = np.asarray([[float(r["x"]), float(r["y"])] for r in rows]).reshape(-1, 6, 2)
    heading = np.asarray([float(r["heading"]) for r in rows]).reshape(-1, 6)
    along = np.stack((np.cos(heading), np.sin(heading)), axis=-1)
    across = np.stack((-np.sin(heading), np.cos(heading)), axis=-1)
    min_gap = math.inf
    pair_samples = 0
    overlap_frames = np.zeros(len(position), dtype=bool)
    for i in range(6):
        for j in range(i + 1, 6):
            delta = position[:, j] - position[:, i]
            axes = (along[:, i], across[:, i], along[:, j], across[:, j])
            separations = []
            for axis in axes:
                radius_i = (2.7 * np.abs(np.sum(along[:, i] * axis, axis=1))
                            + 1.35 * np.abs(np.sum(across[:, i] * axis, axis=1)))
                radius_j = (2.7 * np.abs(np.sum(along[:, j] * axis, axis=1))
                            + 1.35 * np.abs(np.sum(across[:, j] * axis, axis=1)))
                separations.append(np.abs(np.sum(delta * axis, axis=1)) - radius_i - radius_j)
            gap = np.max(np.stack(separations, axis=1), axis=1)
            min_gap = min(min_gap, float(np.min(gap)))
            overlap = gap < 0.0
            pair_samples += int(np.sum(overlap))
            overlap_frames |= overlap
    return min_gap, int(np.sum(overlap_frames)), pair_samples


def main() -> None:
    for folder, marker in ((SIX_BATCH, "FINISHED"), (STRONG, "BATCH_FINISHED"),
                           (H53, "FINISHED"), (M54, "FINISHED")):
        if not (folder / marker).is_file():
            raise ValueError(f"Cohort not finished: {folder}")
    if (SIX_BATCH / "BLOCKED").exists():
        raise ValueError("Six-vessel batch has a BLOCKED marker")

    six_rows = read_rows(SIX_BATCH / "results.tsv", "\t")
    expected = {(sea, seed, method) for sea in SEAS for seed in SEEDS
                for method in METHODS}
    actual = {(r["sea_state"], r["seed"], r["method"]) for r in six_rows}
    if len(six_rows) != 75 or actual != expected or any(r["status"] != "VALID" for r in six_rows):
        raise ValueError("Six-vessel results are not exactly 75 unique valid pairs")
    strong_rows = read_rows(STRONG / "run_metrics.csv")
    if len(strong_rows) != 400:
        raise ValueError("Strong baseline table is not 400 rows")
    extra_strong = [r for r in strong_rows if r["method"] in ("conditional_csd", "gnn_gru")]
    if Counter(r["method"] for r in extra_strong) != {"conditional_csd": 100, "gnn_gru": 100}:
        raise ValueError("Expected 100 Conditional CSD and 100 GNN-GRU strong runs")
    h53_rows = read_rows(H53 / "results.tsv", "\t")
    if len(h53_rows) != 100:
        raise ValueError("H53 table is not 100 rows")
    extra_h53 = [r for r in h53_rows if int(r["repeat"]) > 10]
    if len(extra_h53) != 50:
        raise ValueError("Expected 50 newly completed H53 runs")
    m54_rows = read_rows(M54 / "results.tsv", "\t")
    if len(m54_rows) != 14 or len({(r["seed"], r["repeat"]) for r in m54_rows}) != 14:
        raise ValueError("Expected 14 distinct M54 selected-failure reruns")

    manifest = []
    six_metrics = []
    for row in six_rows:
        run = safe_source(row["run_dir"], CENTRAL)
        summary = read_rows(run / "v7_audit/run_summary.csv")[0]
        if summary["graph_mode"] != row["method"] or summary["random_seed"] != row["seed"]:
            raise ValueError(f"Method/seed mismatch: {run}")
        if int(summary["vessels"]) != 6 or int(summary["complete_frames"]) < 60:
            raise ValueError(f"Incomplete six-vessel trajectory: {run}")
        hull_gap, hull_frames, hull_pairs = oriented_hull_audit(run / "six_vessel_driver.csv")
        manifest.append(archive_run("six_central_v7_75", row["method"],
                                    row["sea_state"], row["seed"], "1", run,
                                    run / "v7_audit/run_summary.csv", six=True))
        six_metrics.append(dict(sea_state=row["sea_state"], seed=row["seed"],
                                method=row["method"], repeat=1, status="VALID",
                                arrival_6of6=int(summary["arrival_success_count_6m"]) == 6,
                                arrival_count=summary["arrival_success_count_6m"],
                                mission_arrival_time_s=summary["mission_arrival_time_6m_s"],
                                observation_duration_s=summary["duration_s"],
                                residence_count=summary["residence_success_count"],
                                min_pair_center_distance_m=summary["minimum_pair_distance_m"],
                                center_overlap_samples_under_5p4m=summary["overlap_pair_samples_under_5p4m"],
                                min_oriented_hull_sat_gap_m=f"{hull_gap:.6f}",
                                oriented_hull_overlap_frames=hull_frames,
                                oriented_hull_overlap_pair_samples=hull_pairs,
                                mean_path_m=summary["mean_path_length_m"],
                                median_control_interval_s=row["median_interval_s"],
                                source_run_name=run.name))
    for row in extra_strong:
        run = safe_source(row["run_dir"], HARBOR)
        manifest.append(archive_run("strong_baselines_extra200", row["method"],
                                    "strong_wind_wave", row["seed"], row["repeat"], run))
    for row in extra_h53:
        run = safe_source(row["run_dir"], HARBOR)
        manifest.append(archive_run("strong_h53_extra50", "gnn_gru_sac",
                                    "strong_wind_wave", row["seed"], row["repeat"], run,
                                    run / "body_audit_strong/metrics.json"))
    for row in m54_rows:
        run = safe_source(row["run_dir"], HARBOR)
        manifest.append(archive_run("medium_m54_selected14", "gnn_gru_sac_m54",
                                    "medium_wind_wave", row["seed"], row["repeat"], run,
                                    run / "m54_audit/metrics.json"))
    if len(manifest) != 339 or len({(r["cohort"], r["sea_state"], r["method"], r["seed"], r["repeat"]) for r in manifest}) != 339:
        raise ValueError("Archive manifest must have 339 unique runs")
    if len({r["trajectory"] for r in manifest}) != len(manifest):
        raise ValueError("Archive trajectory paths are not unique")

    write_rows(OUT / "manifest.csv", manifest)
    write_rows(OUT / "tables/six_central_v7_75.csv", six_metrics)
    write_rows(OUT / "tables/strong_baselines_400.csv",
               normalized(strong_rows, ("run_dir",), HARBOR))
    write_rows(OUT / "tables/strong_h53_100.tsv",
               normalized(h53_rows, ("run_dir",), HARBOR), "\t")
    write_rows(OUT / "tables/medium_m54_selected14.tsv",
               normalized(m54_rows, ("run_dir", "original_run"), HARBOR), "\t")
    for source, name in ((STRONG / "summary_by_sea_method.csv", "strong_baselines_summary.csv"),
                         (H53 / "attempts.tsv", "strong_h53_attempts.tsv"),
                         (M54 / "attempts.tsv", "medium_m54_attempts.tsv")):
        if source.is_file():
            copy_file(source, OUT / "tables" / name)
    code = CENTRAL / "src/vrx_csd_shadow"
    for source, name in (
        (code / "scripts/six_vessel_crossing_driver.py", "six_vessel_crossing_driver.py"),
        (code / "scripts/csd_shadow_graph.py", "csd_shadow_graph.py"),
        (code / "models/central_sac_v6_selected_50k_actor.json", "central_sac_v6_selected_50k_actor.json"),
        (code / "models/harbor_gnn_gru_best.json", "harbor_gnn_gru_best.json"),
        (CENTRAL / "run_six_wamv_crossing_shadow.sh", "run_six_wamv_crossing_shadow.sh"),
        (CENTRAL / "run_offline_six_central_matched_5seed_3sea_5method.sh", "run_offline_six_central_matched_5seed_3sea_5method.sh"),
        (CENTRAL / "validate_six_vessel_run.py", "validate_six_vessel_run.py"),
        (CENTRAL / "analyze_six_vessel_comparison.py", "analyze_six_vessel_comparison.py"),
    ):
        copy_file(source, OUT / "code" / name)
    checksums = []
    for path in sorted((OUT / "code").iterdir()):
        checksums.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(OUT)}")
    (OUT / "code/SHA256SUMS").write_text("\n".join(checksums) + "\n")
    aggregates = []
    for sea in SEAS:
        for method in METHODS:
            rows = [r for r in six_metrics if r["sea_state"] == sea and r["method"] == method]
            complete = [r for r in rows if r["arrival_6of6"]]
            times = [float(r["mission_arrival_time_s"]) for r in complete]
            aggregates.append(dict(sea_state=sea, method=method, runs=len(rows),
                                   all_six_arrived=len(complete),
                                   all_six_arrival_rate=len(complete)/len(rows),
                                   mean_mission_arrival_s=(sum(times)/len(times) if times else ""),
                                   sampled_center_overlap_free=sum(int(r["center_overlap_samples_under_5p4m"]) == 0 for r in rows),
                                   sampled_oriented_hull_overlap_free=sum(int(r["oriented_hull_overlap_frames"]) == 0 for r in rows),
                                   minimum_center_distance_m=min(float(r["min_pair_center_distance_m"]) for r in rows)))
    write_rows(OUT / "tables/six_central_v7_method_sea_aggregate.csv", aggregates)
    readme = """# Completed VRX data snapshot (2026-11-17)

This snapshot adds 339 completed runs to the earlier 750-run snapshot:

| Cohort | Runs | Meaning |
| --- | ---: | --- |
| `six_central_v7_75` | 75 | Five methods × three sea states × five common seeds, one run per key |
| `strong_baselines_extra200` | 200 | Conditional CSD and GNN-GRU strong-sea runs, 100 each |
| `strong_h53_extra50` | 50 | H53 strong-sea repetitions 11–20; repetitions 1–10 are in the earlier snapshot |
| `medium_m54_selected14` | 14 | Separate deep-wall reruns of 14 original C119 non-arrivals |

The formal six-vessel seed list is 20261101, 20261102, 20261110, 20261117, 20261123. The earlier 20261011–20261013 six-vessel pilot runs are excluded from this formal comparison. All five methods use the V7 common nominal controller and actuator path; only graph selection and the SAC residual toggle vary. The world, start and goal positions, 300 s limit, and 6 m first-arrival definition are shared. The SAC actor has 108 observation inputs and 12 action outputs. `code/` holds the tested V7 driver, graph node, weights, runner, and auditing code with SHA-256 checksums.

`manifest.csv` gives each archived trajectory, configuration and world file with a SHA-256 hash of the uncompressed trajectory. Six-vessel runs also include compressed graph edge and state logs and a per-run audit. The table `six_central_v7_75.csv` reports the final fleet arrival time as the last vessel's first entry into its 6 m goal region. `observation_duration_s` includes the post-arrival observation window; these time columns must not be interchanged. `center_overlap_samples_under_5p4m` is a center-distance screen. The separate oriented-hull audit uses 2.7 m × 1.35 m half-extents and four separating axes at each synchronized six-vessel sample; its signed SAT gap is not the exact Euclidean surface distance.

The strong baseline table has 400 rows across four methods. This snapshot stores only the 200 newly completed Conditional CSD and GNN-GRU trajectories because the Distance-Star and Risk-Star trajectories are already in `vrx_completed_20261116`. The H53 table has all 100 rows, while this snapshot stores only repetitions 11–20. The M54 selected-failure reruns are a separate intervention and must not replace observations in the original C119 100-run rate. Original local run paths are reduced to run names in public tables. ROS logs, user credentials, and the mobile SSD contents are not uploaded.
"""
    (OUT / "README.md").write_text(readme)
    print(f"EXPORT_COMPLETE runs={len(manifest)} output={OUT}")


if __name__ == "__main__":
    main()
