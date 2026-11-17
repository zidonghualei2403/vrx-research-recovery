# VRX 科研代码与数据索引

本仓库从旧 Ubuntu 备份中整理出可共享的源码、模型、运行脚本、环境包版本记录和实验表。原始备份未修改。旧代码的完整跨机器复现仍需逐项核对；新机器上的独立六艇 V7 批次另见下方数据快照。请先阅读[复现与绘图指南](docs/REPRODUCE.md)。

仓库公开；同学可直接通过链接查看或 `git clone` 复制代码与数据。

## 同学从哪里开始

1. 需要复制代码：查看 `src/vrx_csd_shadow/`（当前源码）、`locked_v6_src/vrx_csd_shadow/`（平静海域锁定 V6）和 `c119/vrx_csd_shadow/`（中等海况 C119）。
2. 需要看五种方法：根目录 `run_harbor_gnn_gru_vrx.sh` 把 `distance_star`、`risk_star`、`conditional_csd`、`gnn_gru`、`gnn_gru_sac` 映射到同一驱动的模式。锁定 V6 的四个基线和平静海域新方法都使用 `locked_v6_src` 构建的 `install_v6`。
3. 需要摘取数据：`data_export/calm_500_runs.csv` 包含平静海域四种基线各 100 次、锁定 V6 100 次的逐次指标；对应的轨迹、图状态、场景和审计文件见 [平静海况 500 组原始数据](data_export/calm_500_raw_20261117/README.md)。中等海况和已完成的强海况实验见 [750 组轨迹与指标快照](data_export/vrx_completed_20261116/README.md)。`data_summary/` 是较早的验证汇总，不能代替这些逐次数据。
   后续补齐的强海况、14 组中等海况单独复验，以及六艇 V7 的五方法配对实验见 [2026-11-17 数据快照](data_export/vrx_completed_20261117/README.md)。
4. 需要自行绘图：平静海况按 `data_export/calm_500_runs.csv` 的 `method` 分组计算指标；新快照的逐次轨迹见其 `manifest.csv` 和压缩的 `six_vessel_driver.csv.gz`。本仓库不提供新生成的图。

## 重要边界

- 上游 VRX 版本：`osrf/vrx` 提交 `7609d1bd90ce7edb29d040a082f949e8b089c864`。
- `run_locked_v6.sh` 通过 SHA-256 校验锁定 V6 的驱动与两个模型。`locked_v6_src` 中的驱动及模型哈希与旧系统 `install_v6` 一致。
- `environment/` 是从旧 `vrx_rl` 环境中读取的包版本清单，不是已验证的安装锁文件。系统层的 ROS 2 Jazzy、Gazebo Harmonic 和驱动需要重新安装。
- 新快照仅包含选定批次的轨迹、场景配置、审计指标和汇总表；完整原始实验运行目录、ROS 日志、Conda 二进制环境、Codex 聊天和凭据均未上传。
