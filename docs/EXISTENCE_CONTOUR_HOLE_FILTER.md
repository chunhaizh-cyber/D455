# 外围轮廓内部保留与真实穿孔过滤

## 目标

稳定外围轮廓一旦成立，轮廓内部像素默认属于同一个观察存在。系统不再要求每个内部像素都有本帧深度，也不使用插值或历史值生成伪造的稠密深度。

只有能够确认看见外部背景的真实穿孔，才从外围轮廓中扣除：

```text
无深度 / 黑色材质 / 视差失败 -> 保留在存在内部
有效深度且落在存在表面之后 -> 仅成为穿孔候选
与外圈背景深度和颜色一致 -> 增加穿孔证据
同一 track_id 连续多帧确认 -> 从 ownership mask 扣除
```

## 实现流程

`ExistenceContourHoleFilter` 位于 tracker 之后、cluster map 和第三/第四色生成之前。

1. 以稳定材料的外轮廓生成填充掩码，内部无深度像素仍保留。
2. 腐蚀外轮廓，排除轮廓边缘的深度混合像素。
3. 膨胀外轮廓形成外部背景环，从有效且明显更远的像素计算背景深度中值和平均颜色。
4. 在内部只选择有效深度、比存在表面至少远指定阈值、并与外部背景深度相符的像素。
5. 对连通区域执行面积和背景颜色门禁。没有正背景证据时保守拒绝，不删除内部像素。
6. 以稳定 `track_id` 保存局部证据图。候选像素增加证据，未重复观测到的像素缓慢衰减；达到确认帧数后才扣除。
7. 第三格、cluster map 和 final segmentation 使用扣除已确认穿孔后的 ownership mask；第四色使用同一掩码的互补区域，final segmentation 允许后续彩图区域接管被释放的背景缺口。

当前记忆随同一轨迹的 ROI 尺寸做最近邻重采样，并受最大年龄限制。它保存的是“穿孔证据”，不是深度值。cluster metadata 另写 `depth_evidence_pixel_count`，使完整轮廓的归属像素数不再被误报为 100% 有效深度。

## 开关与参数

功能默认关闭：

```text
--existence-contour-hole-filter
--no-existence-contour-hole-filter
--existence-hole-min-depth-gap-mm=250
--existence-hole-background-depth-match-mm=350
--existence-hole-background-color-distance=70
--existence-hole-min-area-px=24
--existence-hole-max-area-percent=35
--existence-hole-inner-margin-px=3
--existence-hole-exterior-ring-px=9
--existence-hole-confirm-frames=2
--existence-hole-memory-max-age-frames=8
```

`configs/candidates/round_001/candidate_0049.json` 基于 `candidate_0030`，只增加该机制及其显式参数。它是 probe，不是 best。

## 可观测指标

`profile.csv` 和标准分析包包含：

```text
existence_hole_filter_enabled
existence_hole_tracked_count
existence_hole_retained_no_depth_pixels
existence_hole_candidate_count
existence_hole_candidate_pixels
existence_hole_confirmed_count
existence_hole_confirmed_pixels
existence_hole_remembered_pixels
existence_hole_rejected_no_background_count
existence_hole_rejected_color_count
existence_hole_processing_ms
```

关键判据：

- 仅缺深度探针：`retained_no_depth_pixels > 0` 且 `confirmed_hole_pixels = 0`。
- 真实背景穿孔探针：稳定后 `candidate_hole_pixels > 0`、`confirmed_hole_pixels > 0`。
- 真实 replay：coverage、unknown、contour-lost、merge/split 和 p95 继续服从项目硬门槛。

## 机制探针

以下脚本生成两组 120 帧四路合成 replay。它们只验证因果机制，不替代真实画面验收，生成目录不提交云端：

```powershell
python scripts\generate_existence_contour_hole_probe.py --out-case analysis_runs\generated_cases\existence_background_hole --mode background-hole --frames 120 --force
python scripts\generate_existence_contour_hole_probe.py --out-case analysis_runs\generated_cases\existence_missing_depth --mode missing-depth --frames 120 --force

$probeArgs = @(
  '--cluster-map',
  '--quality-segmentation',
  '--existence-contour-hole-filter',
  '--replay-dir=analysis_runs\generated_cases\existence_background_hole',
  '--max-frames=120',
  '--no-display',
  '--profile-csv=analysis_runs\existence_background_hole\profile.csv',
  '--cluster-map-export=analysis_runs\existence_background_hole\cluster_map'
)
& .\x64\Release\D455.exe $probeArgs
```

真实回放至少对比 `candidate_0030` 与 `candidate_0049`：

```powershell
python scripts\run_batch.py --candidates configs\candidates\round_001 --cases eval\cases.yaml --candidate-id candidate_0030 --candidate-id candidate_0049 --case-id depth_hole_black_object --case-id near_single_object --case-id hand_occlusion_reappear_holdout --max-frames-override 120 --analysis-export-every-n 30 --ignore-first-n-frames 30 --validate-replay --require-replay-ir --require-reviewed-manifest --execute
```

## 2026-07-14 验证结果

合成机制门禁：

| case | 确认背景穿孔 max | 保留无深度 max | 结论 |
|---|---:|---:|---|
| `existence_background_hole` | 8783 | 0 | 真实背景缺口可在多帧后扣除 |
| `existence_missing_depth` | 0 | 8787 | 无深度内部不会被当作穿孔 |

`existence_missing_depth` 的完整轮廓有 78561 个归属像素，其中本帧有效深度证据为 69774 个，`depth_valid_percent=88.815%`；这证明归属完整性没有被误写为稠密深度完整性。

final segmentation 的中心像素也符合相同口径：`background-hole` 的外环为深度材料 ID 1、中心由后续彩图区域接管为 ID 2；`missing-depth` 的外环和中心都保持为深度材料 ID 1。

真实固定 replay 对比：

| case | candidate | pass | score | p95 ms | max ms | >100ms | 确认穿孔 p95 | 保留无深度 p50 |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `depth_hole_black_object` | 0030 | yes | 96.855 | 47.608 | 84.617 | 0 | - | - |
| `depth_hole_black_object` | 0049 | yes | 93.948 | 66.997 | 95.503 | 0 | 313.0 | 23832.5 |
| `hand_occlusion_reappear_holdout` | 0030 | yes | 96.169 | 52.183 | 80.378 | 0 | - | - |
| `hand_occlusion_reappear_holdout` | 0049 | yes | 94.381 | 64.111 | 91.186 | 0 | 0 | 29866.5 |
| `near_single_object` | 0030 | yes | 96.729 | 48.450 | 74.136 | 0 | - | - |
| `near_single_object` | 0049 | yes | 93.366 | 70.881 | 97.991 | 0 | 943.7 | 21801.0 |

六次运行均为 coverage=100%、unknown=0，且 contour-lost、merge、split 均为 0。0049 的过滤阶段 p95 为 12.747-21.966ms；最终重跑没有 100ms 以上帧，但三个 case 的分数和 p95 均未超过 0030，因此当前安全语义成立、性能不足以晋级。`candidate_0030` 继续作为 motion bucket 基线，0049 只保留为外围轮廓/真实穿孔机制探针。

## 当前边界

- 该过滤器只能处理已经形成的稳定外围轮廓，不能把上游分裂成多个材料的人体或平面自动聚合为一个外轮廓。
- 背景纹理、背景深度变化或外圈有效深度不足时会保守地保留穿孔，优先避免误删真实存在。
- 当前只改变像素归属掩码；不会为内部无深度像素生成测量深度，也不会把缓存证据升级为当前帧精确三维事实。
- 合成探针通过只证明两条机制分支正确；生产晋级仍需真实带开孔目标的 replay 和视觉复核。
