# D455 Static Stability Probe

`StaticStabilityProbe` 是独立于主分割程序的双目静态稳定性测试子项目。它不使用 tracker、轮廓缓存、时间滤波或最终归簇逻辑，每一帧都通过同一个无状态函数独立生成：

- 彩图灰度与 Canny 边缘；
- 左右 IR 边缘；
- 原始深度有效掩码；
- 基于左右 IR 的 OpenCV SGBM 视差和有效掩码；
- 原始帧哈希与派生结果哈希。

首帧和前帧只用于计算差异，不参与当前帧结果生成。

## 构建

```powershell
msbuild .\StaticStabilityProbe\StaticStabilityProbe.vcxproj /p:Configuration=Release /p:Platform=x64 /m
```

输出程序为：

```text
x64/Release/StaticStabilityProbe.exe
```

## 实验顺序

### E0：同一原始帧重复处理

```powershell
.\x64\Release\StaticStabilityProbe.exe `
  --replay-dir=datasets\static_stability_auto_001 `
  --repeat-frame=0 `
  --repeat-count=100 `
  --out-dir=analysis_runs\static_stability_repeat_001
```

验收要求是 `unique_derived_hash_count=1`、`derived_exact_match_percent=100` 和 `software_repeatability_status=pass`。不满足时应先修软件非确定性，不能进入传感器稳定性判断。

### E1：自动控制静态采集

固定相机和场景，预热期间不要移动物体：

```powershell
.\x64\Release\StaticStabilityProbe.exe `
  --capture-dir=datasets\static_stability_auto_001 `
  --out-dir=analysis_runs\static_stability_auto_001 `
  --frames=600 `
  --warmup-frames=300
```

### E2：锁定控制静态采集

保持和 E1 相同的相机位置、光照和场景：

```powershell
.\x64\Release\StaticStabilityProbe.exe `
  --capture-dir=datasets\static_stability_locked_001 `
  --out-dir=analysis_runs\static_stability_locked_001 `
  --frames=600 `
  --warmup-frames=300 `
  --settle-frames=30 `
  --lock-controls
```

`--lock-controls` 会在预热后读取当时的曝光、增益和白平衡，关闭自动控制后继续采集；结束或异常退出时恢复原始设置。需要恢复相机默认自动控制时可显式运行：

```powershell
.\x64\Release\StaticStabilityProbe.exe --reset-auto-controls
```

### 目录回放

```powershell
.\x64\Release\StaticStabilityProbe.exe `
  --replay-dir=datasets\static_stability_auto_001 `
  --frames=570 `
  --ignore-first-frames=30 `
  --out-dir=analysis_runs\static_stability_auto_replay_001
```

### 对比自动控制与锁定控制

```powershell
python .\StaticStabilityProbe\compare_stability.py `
  --baseline=analysis_runs\static_stability_auto_001\stability_summary.json `
  --candidate=analysis_runs\static_stability_locked_001\stability_summary.json `
  --out-dir=analysis_runs\static_stability_auto_vs_locked_001
```

比较器生成 `stability_comparison.csv/json`。`improvement_percent_p95 > 0` 表示锁定控制后的 p95 波动更低；第一轮仍只输出描述性差异，不自动宣布相机合格。

## 输出

实时采集目录保存：

```text
datasets/<case_id>/
  calibration.json
  sensor_options_before_lock.json
  sensor_options_capture.json
  sensor_options_after_capture.json
  sensor_options_restored.json       # 使用固定控制时存在
  frame_metadata.csv
  case_manifest.json
  frames/
    000000_color.png
    000000_depth16.png
    000000_ir_left.png
    000000_ir_right.png
```

分析目录保存：

```text
analysis_runs/<run_id>/
  frame_metrics.csv
  stability_summary.json
```

逐帧指标同时包含相对参考帧和相对前帧的：

- 彩图、左右 IR 的平均绝对差和超阈值像素比例；
- 彩图边缘变化比例；
- 深度平均/p95绝对差、有效像素闪烁率；
- 视差平均/p95绝对差、有效视差闪烁率；
- 原始和派生结果哈希。

`frame_metadata.csv` 额外记录 color/depth/左右IR 各自的硬件帧号、时间戳，以及设备实际提供的逐帧曝光、增益、白平衡和激光功率字段，用于区分传感器变化、自动控制变化和不同步取帧。

实时采集只接受四路帧号都相对上一接受帧前进、左右 IR 帧号相同，且四路时间戳跨度不超过 `--max-stream-timestamp-delta-ms=50` 的 frameset。被拒绝数量写入 manifest 的 `skipped_unsynchronized_framesets`，避免把重复旧 IR 帧误判为传感器稳定。

第一轮阈值只用于描述分布，不直接判定传感器合格。正式阈值应在 E1/E2 固定场景结果出来后，根据 p50/p95/max 和空间分区特征确定。

## 当前验证

- Release x64 构建通过，无警告。
- 真实保存帧重复处理100次：原始哈希1种、派生哈希1种、完全一致率100%。
- 现有 `near_single_object` 回放的连续90帧无状态分析通过；该数据不是本轮正式静态采集，只验证读取和统计链路。
- 真实 D455 自动控制和锁定控制短采集通过；锁定时 AE/AWB 从1变为0，结束后恢复为1。
- `compare_stability.py` 通过自比较烟测，能够生成同口径 p50/p95/max 差值和改善比例。

## 边界

- SGBM 指标只描述左右 IR 视差输出稳定性，不等同于最终物体距离精度。
- 原始 depth/IR 保持各自传感器坐标，不为了显示而对齐到 RGB。
- 本子项目不判断最终轮廓、簇身份或彩图归属稳定性；这些应在原始双目稳定边界明确后作为下一层测试加入。
- `datasets/`、`analysis_runs/` 和构建产物不进入 Git。
