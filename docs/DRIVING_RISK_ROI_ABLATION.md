# 驾驶风险有限识别消融路线 v0.1

## 1. 研究目标

首轮不以绝对帧率作为方法成立的前置条件，而验证：

> 全画面深度、空间占据和未知账本持续运行时，限制彩图语义处理范围和刷新频率，能否降低可控语义工作量，并保持现有空间账本与时序代理指标。

该目标只覆盖离线机制验证。当前没有同步车速、转角、规划轨迹和错误空闲真值，因此不得解释为真实自动驾驶安全验证，也不得宣称适用于 60 km/h 实时驾驶。

## 2. 四层目标与门禁

### G0 数据与因果隔离

- 所有候选使用相同确定性目录回放、帧区间、代码和编译配置。
- 每组只改变语义处理范围、触发方式或调度方式。
- 原始 `.bag` 保持只读，派生数据不代替原始材料。

### G1 空间账本代理门禁

- `cluster_coverage_percent_p50 >= 95`。
- `unknown_percent_p50 <= 15`。
- 未关注区域继续由现有深度、背景、远场或未知模式登记，不得因未运行彩图语义而直接写成可靠空闲。

当前缺少轨迹占据和错误空闲真值，所以 G1 只是工程代理门禁，不是安全结论。

### G2 时序代理门禁

- `contour_lost_event_count = 0`。
- `merge_event_count = 0`。
- `split_event_count = 0`。
- 结果年龄超过 `attention-max-result-age-frames` 或缓存版本不一致时拒绝应用。

### G3 工作量目标

- 统一记录 `semantic_processed_pixels / percent`，缓存复用帧记为 0。
- 相对全画面逐帧基线，实验候选的平均语义工作量至少下降 10 个百分点。
- 同时报告风险走廊面积、实际语义处理帧数和全帧等价工作量。

### G4 性能观察项

- 保留 `p50/p95/p99/max`、超过 100 ms 帧数和各阶段耗时。
- 当前仓库 100 ms hard gate 继续原样报告，但不作为本轮机制代理结论的唯一否决条件。
- 队列、缓存年龄和陈旧结果必须有界；不能以无限积压换取表面低耗时。

## 3. 首轮四组消融

| 候选 | 语义范围 | 刷新方式 | 用途 |
| --- | --- | --- | --- |
| `candidate_0050` | 全画面 | 每帧同步 | 100% 语义工作量参考 |
| `candidate_0051` | 固定驾驶走廊 | 每帧同步 | 隔离空间裁剪收益 |
| `candidate_0052` | 驾驶走廊内的差异 ROI | 变化触发、同步 | 隔离动态局部刷新收益 |
| `candidate_0053` | 同 0052 | 15 帧 cooldown、异步、年龄门禁 | 测量分级频率与调度收益 |

驾驶走廊由纵向分片矩形近似梯形，默认参数：

```text
top=35%
bottom=100%
far_width=30%
near_width=90%
center_offset=0%
slices=4
```

这只是图像空间注入式走廊。`candidate_0052/0053` 的“动态”来自画面差异触发，不代表已经接入真实车辆状态。

## 4. 实现边界

`--driving-risk-roi` 只约束彩图轮廓语义刷新：

```text
全帧 color/depth/IR 输入
  -> 全画面深度、边界、空间簇和像素账本
  -> 构建驾驶走廊
  -> 固定走廊刷新，或差异 ROI 与走廊求交
  -> 只对受限 ROI 执行彩图轮廓语义提取
  -> 合并缓存语义结果
  -> 未关注区域继续保留空间模式或未知状态
```

当纯 motion 差异完全位于走廊外时，本帧跳过彩图语义刷新并记录 `driving_risk_roi_clipped_all_dirty=1`；这不影响全画面空间链。启动、周期、未知突增或远场丢失等非纯 motion 刷新仍至少处理完整走廊。

## 5. 指标

新增逐帧指标：

```text
driving_risk_roi_enabled
driving_risk_roi_pixels
driving_risk_roi_percent
driving_risk_roi_clipped_all_dirty
semantic_processed_pixels
semantic_processed_percent
```

评分汇总新增：

```text
semantic_processing_frame_count
semantic_processed_percent_p50/p95/mean
semantic_full_frame_equivalent
semantic_workload_reduction_percent
```

`scripts/summarize_driving_roi_ablation.py` 以 `candidate_0050` 为基线生成机制代理结论。它不覆盖仓库正式 `pass`，也不生成自动驾驶安全结论。

## 6. 首轮固定回放

使用今天原始车载 `.bag` 派生的三个连续窗口：

```text
analysis_runs/generated_cases/vehicle_motion_contiguous_005s
analysis_runs/generated_cases/vehicle_motion_contiguous_035s
analysis_runs/generated_cases/vehicle_motion_contiguous_050s
```

每个窗口 120 帧，忽略前 15 帧启动阶段。首轮为 `4 candidates x 3 cases = 12 runs`。

## 7. 后续真实风险 ROI

首轮机制通过后再进入下一阶段：

1. 保存同步车速、转角、制动状态和规划轨迹。
2. 由轨迹走廊、停车距离和遮挡边界生成真实风险 ROI。
3. 标注轨迹占据、错误空闲、未知障碍和 ROI 进入时刻。
4. 在独立重复的 `0/5/10/20 km/h` 数据上验收。
5. `40/60 km/h` 仅作为被动离线压力测试，直到闭环安全证据成立。

## 8. 首轮执行结果

`replay_driving_risk_roi_ablation_001` 已完成 12 条运行。评分忽略每段前 15 帧，三个窗口合计 315 个评分时序帧。

| 候选 | 仓库 hard pass | 平均语义工作量 | 全帧等价次数 | 平均 p95 | 代理结论 |
| --- | ---: | ---: | ---: | ---: | --- |
| `0050` | 3/3 | 100.000% | 315.000 | 91.219 ms | 全画面参考成立 |
| `0051` | 2/3 | 39.000% | 122.850 | 121.706 ms | 空间裁剪成立；一次环境尖峰 |
| `0052` | 3/3 | 2.291% | 7.218 | 63.492 ms | 差异局部刷新代理通过 |
| `0053` | 3/3 | 0.385% | 1.211 | 61.714 ms | 分级频率工作量最低 |

四组均为 `coverage=100%`、`unknown=0%`，且 contour-lost/merge/split 合计均为 0。`0052` 在 315 帧中有 288 帧处理小 ROI；`0053` 只有 21 帧提交语义处理，因此 0053 的收益主要来自刷新频率和缓存，而不是更小的单次走廊。

`0051 x 050s` 首次运行出现所有阶段同时放大的系统级尖峰，`p95=230.708ms`、`max=2944.941ms`；独立原配置复跑为 `p95=60.734ms`、`max=72.510ms`，语义工作量仍为 39%。失败样本保留，当前归类为运行环境抖动风险，不删除也不据复跑覆盖原结果。

本轮只证明：在这三个固定车载窗口和现有代理指标下，语义工作量可被明确约束，且没有观察到像素账本、未知比例及已有事件计数退化。由于 `FarBackground` 可以维持账本覆盖，以上结果不能替代错误空闲、轨迹占据召回和逐对象真值。`candidate_0052/0053` 均保持 probe-only，不进入 `configs/best/`。

最小结果：

```text
leaderboards/replay_driving_risk_roi_ablation_001/leaderboard.csv
leaderboards/replay_driving_risk_roi_ablation_001/pareto_front.csv
leaderboards/replay_driving_risk_roi_ablation_001/ablation_summary.csv
leaderboards/replay_driving_risk_roi_ablation_001/ablation_decision.json
```

## 9. 验证命令

```powershell
msbuild .\D455.vcxproj /p:Configuration=Release /p:Platform=x64 /m

python -m py_compile `
  scripts\convert_exports_to_analysis_run.py `
  scripts\score_run.py `
  scripts\select_winners.py `
  scripts\summarize_driving_roi_ablation.py
```

实际运行输出位于本地 `analysis_runs/`，不提交原始回放和完整运行数据。最小 leaderboard 与结论摘要可提交到 `leaderboards/`。
