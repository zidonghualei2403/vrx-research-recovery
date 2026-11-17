#!/usr/bin/env python3
"""Archive the 500 completed calm-harbor runs from the local internal disk."""
from __future__ import annotations

import csv
import gzip
import hashlib
import json
import shutil
from collections import Counter
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
INDEX = REPO / "data_export/calm_500_runs.csv"
SOURCE = Path("/home/hzh/Documents/Codex/2026-08-13/hao/vrx_harbor_sac_v6_aligned_ws_v1")
OUT = REPO / "data_export/calm_500_raw_20261117"
METHODS = {"conditional_csd", "distance_star", "gnn_gru", "locked_v6_gnn_gru_sac", "risk_star"}
CAMPAIGNS = {"calm_baselines_5seed_10repeat_gui_20261103", "v6_main50", "v6_extra50"}
SEEDS = {"20261101", "20261102", "20261110", "20261117", "20261123"}
COMPRESSED = ("six_vessel_driver.csv", "shadow_edges.csv", "shadow_states.csv")
COPIED = ("experiment_config.csv", "csd_harbor.sdf", "body_audit/metrics.json",
          "body_audit/overlap_events.csv")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    source_root = SOURCE.resolve(strict=True)
    with INDEX.open(newline="", encoding="utf-8") as stream:
        runs = list(csv.DictReader(stream))
    if len(runs) != 500 or Counter(row["method"] for row in runs) != {m: 100 for m in METHODS}:
        raise ValueError("Expected exactly 100 runs for each of five calm-sea methods")
    if len({row["run_dir_relative"] for row in runs}) != 500:
        raise ValueError("Duplicate run directory in calm index")

    planned = []
    for row in runs:
        method, campaign, seed = (row[key] for key in ("method", "campaign", "seed"))
        repeat = int(row["repeat"])
        if method not in METHODS or campaign not in CAMPAIGNS or seed not in SEEDS or not 1 <= repeat <= 20:
            raise ValueError(f"Unexpected method/campaign/seed/repeat: {row}")
        relative = Path(row["run_dir_relative"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"Unsafe source path: {relative}")
        source = (SOURCE / relative).resolve(strict=True)
        if not source.is_relative_to(source_root) or not source.is_dir():
            raise ValueError(f"Source outside internal workspace: {source}")
        destination = OUT / method / campaign / f"seed{seed}" / f"repeat{repeat:02d}"
        for name in COMPRESSED + COPIED:
            path = source / name
            if not path.is_file() or path.is_symlink():
                raise ValueError(f"Missing or linked source file: {path}")
        planned.append((row, source, destination))
    if len({dest for _, _, dest in planned}) != 500:
        raise ValueError("Duplicate archive destination")

    OUT.mkdir(parents=True, exist_ok=True)
    manifest = []
    for number, (row, source, destination) in enumerate(planned, 1):
        destination.mkdir(parents=True, exist_ok=True)
        for name in COMPRESSED + COPIED:
            input_path = source / name
            output_path = destination / (name + ".gz" if name in COMPRESSED else name)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            if name in COMPRESSED:
                with input_path.open("rb") as input_stream, output_path.open("wb") as raw_stream:
                    with gzip.GzipFile(filename="", mode="wb", fileobj=raw_stream,
                                       mtime=0, compresslevel=6) as output_stream:
                        shutil.copyfileobj(input_stream, output_stream, 1024 * 1024)
            else:
                shutil.copyfile(input_path, output_path)
            manifest.append({
                "method": row["method"], "campaign": row["campaign"],
                "seed": row["seed"], "repeat": row["repeat"],
                "run_name": source.name, "source_file": name,
                "archive_file": str(output_path.relative_to(OUT)),
                "source_bytes": input_path.stat().st_size,
                "source_sha256": sha256_file(input_path),
                "archive_bytes": output_path.stat().st_size,
                "archive_sha256": sha256_file(output_path),
            })
        if number % 50 == 0:
            print(f"Archived {number}/500 runs", flush=True)

    manifest_path = OUT / "manifest.csv"
    with manifest_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(manifest[0]))
        writer.writeheader()
        writer.writerows(manifest)
    (OUT / "README.md").write_text(
        "# 平静海况港池实验：500 组逐次原始数据\n\n"
        "本目录对应上一级 `calm_500_runs.csv` 的全部 500 行：五种方法各 100 组，"
        "使用五个固定种子。`locked_v6_gnn_gru_sac` 的 100 组分属 `v6_main50` "
        "和 `v6_extra50` 两个批次；目录名保留批次以避免重复编号冲突。\n\n"
        "每组包含船只轨迹 `six_vessel_driver.csv.gz`、图边和状态记录、"
        "实验配置、Gazebo 场景，以及船体审计结果和重叠事件。"
        "`manifest.csv` 逐文件列出原始和归档文件的字节数与 SHA-256。"
        "可用 `gzip -dc 文件.csv.gz` 读取压缩 CSV。\n\n"
        "数据从本机内部硬盘的已完成运行目录只读归档；未读取或修改移动硬盘。"
        "此归档不包含 ROS 控制台日志、系统二进制环境或用户凭据。\n",
        encoding="utf-8",
    )
    print(json.dumps({"runs": len(planned), "files": len(manifest),
                      "methods": dict(Counter(r["method"] for r in runs))}, ensure_ascii=False))


if __name__ == "__main__":
    main()
