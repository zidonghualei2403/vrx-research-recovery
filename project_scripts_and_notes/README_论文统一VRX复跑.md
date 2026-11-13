# 论文统一 VRX 复跑

## 目的

在同一随机种子、同一海况、同一往返船协议和同一物理船体评价口径下，复跑：

- Distance-Star
- Risk-Star
- Conditional CSD
- GNN-GRU
- GNN-GRU + residual SAC

共 `4 海况 × 5 方法 = 20` 个单元。往返船使用规定航线并关闭直接风/流作用，四艘受控船仍承受对应海况；所有方法共享完全相同的场景设置。

## 启动

```bash
cd /home/hzh/Documents/Codex/2026-08-13/hao/vrx_harbor_sac_v6_aligned_ws_v1
bash start_paper_vrx_batch.sh 20261018
```

该命令后台运行。发生停电、重启或主动中断后，再执行同一命令即可；已经完成的单元会自动跳过。

## 查看进度

```bash
cd /home/hzh/Documents/Codex/2026-08-13/hao/vrx_harbor_sac_v6_aligned_ws_v1
python3 watch_paper_vrx_batch.py 20261018
```

`Ctrl+C` 只关闭进度显示，不会停止后台实验。

## 停止

```bash
cd /home/hzh/Documents/Codex/2026-08-13/hao/vrx_harbor_sac_v6_aligned_ws_v1
PID=$(cat .paper_vrx_batch_20261018.pid 2>/dev/null)
test -n "$PID" && kill -INT "$PID"
bash cleanup_vrx_batch_processes.sh
```

已完成单元和 CSV 会保留。

## 输出

批处理目录：

```text
results/paper_matched_body_aware_seed20261018/
```

关键文件：

- `status.csv`：每个海况和方法的完成/重试状态；
- `body_aware_audit.csv`：统一物理船体口径的最终表；
- `*.log`：每个单元的 ROS/Gazebo 日志；
- `BATCH_FINISHED`：整个批处理已经结束的标志。

## 统计注意事项

1. 种子 `20261018` 只是一轮协议验证，不足以给出期刊统计结论；
2. 协议验证通过后，应至少增加 3 个匹配种子，较稳妥的是 10–20 个；
3. 论文不得再用中心距离代替碰撞判定；负间隙表示有方向矩形船体发生重叠；
4. 共形下界是边准入置信度，不等于物理零碰撞保证；
5. 首次到达、连续驻留和最终占位要分开报告。
