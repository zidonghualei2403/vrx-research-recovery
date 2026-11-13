# VRX Harbor Shuttle-Aligned GNN-GRU V1

This workspace is the isolated VRX validation target for the GNN-GRU trained
on the bidirectional shuttle scenario. It must not be described as validated
until `run_shuttle_alignment_check.sh` reports `SHUTTLE_VALIDATION=PASS`.

## Model provenance

- training workspace: `csd_lfcg_shuttle_gnn_sac_train_v1`
- checkpoint: `outputs_shuttle_harbor_gnn_gru_v1/models/gnn_gru_best.pt`
- checkpoint SHA-256: `8379ef2d0da05e97bac434adb57b57e1adea48e91c13384754a90aae673948d6`
- exported VRX JSON SHA-256: `862294f60ad23bb2b4ea76d181e664bc585357b8507c4dcce31d077e4422ec92`

The older validated model hash `944f6f...` is deliberately not used here.

## Build

```bash
cd /home/hzh/Documents/Codex/2026-08-13/hao/vrx_harbor_shuttle_aligned_ws_v1
source /opt/ros/jazzy/setup.bash
source /home/hzh/桌面/vrx_ws/install/setup.bash
colcon build --packages-select vrx_csd_shadow
```

## Automatic acceptance test

```bash
cd /home/hzh/Documents/Codex/2026-08-13/hao/vrx_harbor_shuttle_aligned_ws_v1
bash run_shuttle_alignment_check.sh calm 20261011
```

The focused validation allows 240 s by default. Reciprocal patrol traffic
creates two real waiting windows, so the old 120 s limit could terminate a
safe, still-converging vessel before the second gap opened.

If VRX creates all models but the GPS/IMU streams do not start, the acceptance
script rejects the header-only CSV and performs one clean launch retry. Such a
startup miss is infrastructure failure, not a navigation result.

Monitor a running validation from another terminal:

```bash
cd /home/hzh/Documents/Codex/2026-08-13/hao/vrx_harbor_shuttle_aligned_ws_v1
python3 watch_shuttle_alignment_progress.py
```

The test requires all six vessels to be logged, both shuttle vessels to cover
their routes without thrust chatter, and all four cooperative vessels to
arrive and remain at their goals. Residual SAC training is allowed only after
this test passes.
