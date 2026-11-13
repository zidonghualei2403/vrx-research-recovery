#!/usr/bin/env python3
from __future__ import annotations

import csv
import os
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
seed = sys.argv[1] if len(sys.argv) > 1 else "20261018"
batch = ROOT / "results" / f"paper_matched_body_aware_seed{seed}"
status_path = batch / "status.csv"
heartbeat_path = batch / "heartbeat.txt"
total = 20
pid_path = ROOT / f".paper_vrx_batch_{seed}.pid"


def process_is_alive() -> bool:
    try:
        pid = int(pid_path.read_text().strip())
        os.kill(pid, 0)
        return True
    except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError):
        return False

while True:
    done = failed = retries = 0
    if status_path.is_file():
        with status_path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        completed_cells = {(row["sea_state"], row["method"]) for row in rows if row["status"] == "DONE"}
        failed_cells = {(row["sea_state"], row["method"]) for row in rows if row["status"] == "FAILED"}
        done = len(completed_cells)
        failed = len(failed_cells - completed_cells)
        retries = sum(row["status"] == "RETRY" for row in rows)
    current = "waiting"
    current_path = batch / "current.txt"
    if current_path.is_file():
        current = current_path.read_text(errors="ignore").strip().replace(",", " / ")
    width = 40
    filled = int(width * done / total)
    bar = "#" * filled + "-" * (width - filled)
    heartbeat = ""
    if heartbeat_path.is_file():
        try:
            heartbeat_epoch = int(heartbeat_path.read_text().split(",", 1)[0])
            age = max(0, int(time.time()) - heartbeat_epoch)
            heartbeat = f" 心跳={age}s"
            if age > 120:
                heartbeat += "(已停滞)"
        except (OSError, ValueError):
            heartbeat = " 心跳=无效"
    print(
        f"\rVRX统一复跑[{bar}] {done}/{total} {100*done/total:5.1f}% "
        f"失败={failed} 重试={retries} 当前={current:36s}{heartbeat}",
        end="",
        flush=True,
    )
    if (batch / "BATCH_FINISHED").is_file():
        print(f"\n批处理结束。结果：{batch}")
        break
    if not process_is_alive():
        print("\n后台当前未运行；已有结果保留。重新执行启动脚本即可续跑。")
        break
    time.sleep(5)
