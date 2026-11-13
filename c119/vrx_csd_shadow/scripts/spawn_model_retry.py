#!/usr/bin/env python3

import subprocess
import sys
import time


def main():
    arguments = sys.argv[1:]
    attempts = 4
    for attempt in range(1, attempts + 1):
        result = subprocess.run(
            ["ros2", "run", "ros_gz_sim", "create", *arguments],
            check=False,
        )
        if result.returncode == 0:
            return
        print(
            f"Model spawn failed ({attempt}/{attempts}); retrying in 3 s.",
            file=sys.stderr,
            flush=True,
        )
        time.sleep(3.0)
    raise SystemExit("Model spawn failed after four attempts.")


if __name__ == "__main__":
    main()
