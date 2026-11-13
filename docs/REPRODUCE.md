# 复现、摘取数据与绘图

## 1. 方法和源码

根目录 `run_harbor_gnn_gru_vrx.sh` 接收 `海况 种子 方法`。方法名与驱动模式如下：

| 方法参数 | 论文称呼 | 驱动 `graph_mode` |
|---|---|---|
| `distance_star` | Distance-Star | `distance_star` |
| `risk_star` | Risk-Star | `risk_star` |
| `conditional_csd` | Conditional CSD | `conditional_csd` |
| `gnn_gru` | GNN-GRU | `learned_gnn_gru_graph_shield` |
| `gnn_gru_sac` | GNN-GRU + residual SAC / locked V6 | `gnn_gru_residual_sac` |

`src/vrx_csd_shadow/` 保存主工作区当前源码。平静海域的**锁定 V6**必须用 `locked_v6_src/vrx_csd_shadow/` 构建：它的 `six_vessel_crossing_driver.py` 是冻结版本，SHA-256 为 `e4dfd79a85e94d899f35786487888f16732472037a0cb4e604843842555c25c9`，与旧系统 `install_v6` 的驱动相同。两个模型的 SHA-256 分别为 SAC `82d19bbf6b63dd1695eec438ab3e368c4958faf4ac6fd2cd85bca18263357148`、GNN-GRU `862294f60ad23bb2b4ea76d181e664bc585357b8507c4dcce31d077e4422ec92`。不要把当前 `src` 的驱动误当作冻结 V6。

中等海况 C119 的源码快照在 `c119/vrx_csd_shadow/`。它是不同的冻结控制器；跨海况比较时要分别标注版本。

## 2. 在 Ubuntu 24.04 上准备运行环境

先按 [ROS 2 Jazzy 官方说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 和 [VRX 系统要求](https://github.com/osrf/vrx/wiki/system_requirements) 安装 ROS 2 Jazzy、Gazebo Harmonic 与 ROS/Gazebo 桥接依赖。旧 `/opt/ros/jazzy` 不需要复制。以下是源码的**构建步骤示例**；新机器上尚未完成实际运行验证。

```bash
mkdir -p ~/vrx_ws
git clone https://github.com/osrf/vrx.git ~/vrx_ws/src
git -C ~/vrx_ws/src checkout 7609d1bd90ce7edb29d040a082f949e8b089c864
source /opt/ros/jazzy/setup.bash
cd ~/vrx_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source ~/vrx_ws/install/setup.bash
```

在本仓库根目录构建锁定 V6：

```bash
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
colcon --log-base log_v6 build --base-paths locked_v6_src \
  --packages-select vrx_csd_shadow --build-base build_v6 --install-base install_v6
export VRX_WS_INSTALL="$HOME/vrx_ws/install"
```

`run_harbor_gnn_gru_vrx.sh` 默认从 `$HOME/vrx_ws/install` 加载上游 VRX 工作区；若工作区位于其他位置，设置 `VRX_WS_INSTALL`。`environment/` 的 Conda 和 Python 包清单用于重建与核对 `vrx_rl`，其中 CUDA/PyTorch 版本需按新机器 GPU 和驱动重新选择，不能直接复制旧环境二进制目录。

单次平静海域锁定 V6 运行命令示例：

```bash
bash run_locked_v6.sh calm 20261101
```

四种基线的平静海域批处理脚本是 `run_calm_baselines_resumable_gui.sh`，默认每种种子 10 次；本仓库导出的基线数据来自后续扩展到**每种种子 20 次**的完成记录。锁定 V6 的首批 50 次脚本是 `run_locked_v6_calm_5seed_10repeat_gui.sh`，另有独立的第二批 50 次。若要重新跑到同样样本数，必须分清批次并设置不同输出目录，不能把 50 次结果当作 100 次。这些脚本会启动 ROS/Gazebo、产生结果文件并清理旧进程，请先用单次运行确认环境，再运行批处理。不要在原始 F: 备份上运行。

## 3. 逐次数据和口径

`data_export/calm_500_runs.csv` 的 500 行由原备份中两份锁定 V6 `results.tsv`（50＋50 行），以及四种基线的 `status.csv` 中 400 条 `DONE` 记录和对应 `body_audit/metrics.json` 生成。`RETRY`、`FAILED` 尝试没有计入完成运行的分母。每种方法都是 **5 个种子 × 每种子 20 次 = 100 次**。V6 的两批各 10 次重复由 `campaign` 字段区分。

- `arrival_count`：四艘受控船中完成到达的数量。
- `vessel_overlap_frames`、`wall_overlap_frames`：船体与船体、船体与墙重叠的采样帧数。这里用船体几何判断，不以中心距离代替碰撞。
- `stall20_count`：远离目标的连续 20 秒停滞事件数。
- `strict_safe=1`：4/4 到达、无船体重叠、无撞墙、无 20 秒停滞，并通过基线审计中的六船有效性检查。锁定 V6 原 TSV 没有单独的 `six_vessels_valid` 列，按其审计脚本已写出的上述字段判断。
- `strict_success_130s=1`：满足严格安全，且全队完成时间不超过 130 秒。
- `run_dir_relative`：相对于旧 VRX 项目根目录的原始运行目录。需要轨迹、逐帧数据时，去 F: 的该目录查找 `six_vessel_driver.csv`。

当前表算得的严格安全率：Distance-Star **34/100**、Risk-Star **33/100**、Conditional CSD **49/100**、GNN-GRU **45/100**、锁定 V6 **77/100**。这些是平静海域这批 500 次运行的描述统计，不代表其他海况。

若拿到完整 F: 项目备份，可重新生成逐次表：

```bash
python3 tools/export_from_backup.py --project-root /path/to/vrx_harbor_sac_v6_aligned_ws_v1
```

`data_summary/` 的验证汇总来自较早实验，样本数和安全口径与 `calm_500_runs.csv` 不同；请在图注中写明使用哪一份。

## 4. 同学自行绘图

打开 `data_export/calm_500_runs.csv`，按 `method` 分组。每组恰好 100 行，`strict_safe` 的平均值乘以 100 是严格安全率，`strict_success_130s` 的平均值乘以 100 是 130 秒内严格成功率。图注写明“calm，五种方法各 100 次”，并写出上述判定口径。需要画误差条或做统计检验时，按 `seed` 分层处理，不要把同种子重复运行误作完全独立的种子。

单次轨迹图需要 `run_dir_relative` 对应原始目录中的 `six_vessel_driver.csv`。完整逐帧轨迹没有上传到 GitHub；需要时从 F: 备份按该相对路径复制。仓库保留旧项目原有的 `plot_harbor_route_diagnostic.py` 供参考，但本次没有制作或上传新图。

## 5. 文件复制和引用

同学可用 `git clone` 复制本仓库。研究中引用表格或图片时，记录 Git 提交哈希、数据文件名、海况、方法、种子、重复次数和指标定义。模型或脚本有改动时另建提交，不能继续称为“锁定 V6”。
