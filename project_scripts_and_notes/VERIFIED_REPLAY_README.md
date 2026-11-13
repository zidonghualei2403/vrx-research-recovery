# V6 verified replay

This entry point reuses the exact calm-water SDF saved by the earlier V6 run that
was recorded as PASS. It does not regenerate the world or change the random seed.

```bash
cd /home/hzh/Documents/Codex/2026-08-13/hao/vrx_harbor_sac_v6_aligned_ws_v1
HEADLESS=false MAX_EXPERIMENT_DURATION=240 bash run_verified_v6_replay.sh
```

The new result is written under `results/verified_v6_replay_<timestamp>/`.
Check `six_vessel_driver.csv` and confirm the four cooperative vessels reach and
hold their goals. The exact source world is recorded in `experiment_config.csv`.
