# D455 2x2 外设观察材料演示

这个工程是第一阶段稳定型路线：

- RealSense D455 采集 RGB + Depth + 左红外灰度图。
- RealSense depth 后处理：Depth->Disparity、Spatial、Temporal、Disparity->Depth、Hole Filling。
- OpenCV 左红外灰度边缘 + 深度边缘 + 深度确认的红外轮廓边界生成切分边界，深度切片、形态学、连通域、背景大块过滤、精细轮廓线提取；RGB 彩色流只用于第一窗口原图和最终稳定轮廓彩图剪切。
- PCL 点云聚类：把候选轮廓内的有效深度点反投影为点云，使用欧式聚类给完整候选材料标记 3D cluster，用于阻止后续二维近邻误合并；稳定轮廓显示仍使用完整 OpenCV 轮廓，避免稀疏点重建造成条纹漏空。
- 稳定基准点：用原始深度和滤波深度的一致性、局部深度稳定性、边缘排除筛出可信稀疏点。
- 稳定轮廓跟踪：当前帧先生成候选材料，再用稳定基准点筛选和校准轮廓中心/深度，最后用历史轨迹确认稳定轮廓。
- 单个 `D455 observation dashboard 2x2` 主窗口显示四格同步画面：
  - 左上：原始彩色帧。
  - 右上：以左红外灰度图为背景，显示稳定基准点、稳定点支撑的候选轮廓、历史确认后的稳定轮廓和当前稳定评分。
  - 左下：只显示稳定轮廓内部的原始彩图；P5 测试分支默认会在稳定轮廓邻域内严格使用 RGB 轮廓补齐断口，补齐候选必须和原稳定轮廓差异很小、且不碰到其他稳定轮廓隔离带才采用，其他区域保持黑色未知背景。
  - 右下：单独显示左下未知背景对应的原始彩图，稳定/补齐轮廓内部保持黑色。

当前输出只表示“外设观察存在材料 / 稳定候选轮廓”，不是世界真值、已确认观察存在、扫描事实或跟踪事实。

## 维护规则

每次调整算法、显示、性能、验证或排查方向时，都必须同步更新本 README，说明本次改进方向、影响的观察/显示链路、默认行为变化、主要开关参数和验证方式。README 应作为当前方案边界和后续调参入口，不能只在代码或对话中保留改进意图。

每轮结束时如果项目文件有变动，必须自动完成云端同步：先运行相关校验，再只暂存本轮有意变更的项目文件，提交并推送当前分支到 `origin`。不要自动同步真实录制视频、实际运行大数据、构建产物、密钥或无关用户改动；如果校验失败，则停止推送并说明阻塞原因。

## 云端分析协议

仓库现在固定使用 `analysis_runs/<run_id>/` 作为云端分析包目录，协议说明见 `docs/ANALYSIS_PROTOCOL.md`，评判指标见 `docs/EVALUATION_FEATURES.md`，特征采集性能策略见 `docs/FEATURE_OPTIMIZATION.md`，运行索引见 `docs/RUN_INDEX.md`。每次同步一次运行目录即可按 `run_id / branch / commit / config / metrics / sample_frames` 复查问题；不要把分析数据散落到 `recordings/` 里再靠聊天记录解释。最小六件套为：

```text
analysis_runs/<run_id>/run_manifest.json
analysis_runs/<run_id>/config_snapshot.json
analysis_runs/<run_id>/frame_metrics.csv
analysis_runs/<run_id>/cluster_metrics.jsonl
analysis_runs/<run_id>/events.csv
analysis_runs/<run_id>/notes.md
```

新建运行目录时可以复制 `analysis_runs/_template`，或用脚本自动写入当前 branch/commit：

```powershell
.\tools\New-AnalysisRun.ps1 -RunId 20260703_153000_case01 -CaseName indoor_far_objects -Purpose "P5 full-pixel clustering check" -Scene "近处桌面物体，远处柜子和墙面"
```

小文件直接进 Git；`*.mp4`、`*.avi`、`*.bag`、`*.raw`、`*.bin`、`*.depth`、`*.npy`、`analysis_runs/**/sample_frames/*.png` 和 `analysis_runs/**/videos/*` 已在 `.gitattributes` 中声明为 Git LFS。同步完成后在 `docs/RUN_INDEX.md` 追加一行，并告知 `branch / commit / run_id / 重点问题`。

工程效果评判 v0.1 的总目标是把“全像面像素归簇、近场高精度空间信息、远场保留 2D 轮廓并返回粗距、超出深度精度范围不丢存在轮廓”拆成可度量字段。第一版优先落地 `cluster_coverage_percent`、`unknown_percent`、mode 像素分布、每 cluster 的 `mode / bbox / contour / depth / stereo / confidence` 和 `events.csv`；分析输出占位文件为 `run_score.json`、`run_summary.md`、`failure_report.md`。

特征采集不能拖垮实时主循环。默认路线是 `feature_profile=normal`：每帧保留核心 `frame_metrics`，高成本轮廓/双目/PCL 诊断按间隔或事件触发，sample frames 只在采样或异常时保存。后续实现命令行开关时按 `--feature-profile=light|normal|debug|full`、`--feature-heavy-interval=15`、`--feature-dump-on-event` 的口径落地。

自动实验闭环见 `docs/AUTOMATED_EXPERIMENT_LOOP.md`。闭环原则是“程序自己跑，评判集自己打分，Codex 只根据证据提出下一轮参数、运行方式或代码改动”。第一阶段只开放配置候选：`configs/baseline/` 保存基线，`configs/candidates/` 保存候选，`eval/` 保存 case、搜索空间、权重和阈值，`scripts/score_run.py` / `scripts/select_winners.py` / `scripts/make_codex_handoff.py` 生成 `run_score.json`、排行榜和 `.codex_handoff/round_xxx.md`。在 replay 输入真正实现前，`scripts/run_batch.py` 只用于生成 dry-run 命令计划，不代表已完成评测。当前过渡闭环先用 `scripts/convert_exports_to_analysis_run.py` 把 D455 已能导出的 `cluster_map_metadata.json`、`final_segmentation_metadata.json` 和 `profile.csv` 转成标准 `frame_metrics.csv`、`cluster_metrics.jsonl`、`events.csv`，再交给 `score_run.py`。

自动优化 Round 001 已先落地为配置候选和干跑计划：候选位于 `configs/candidates/round_001/`，计划位于 `.codex_handoff/round_001_plan/command_plan.csv`，交接记录为 `.codex_handoff/round_001.md`。这一轮只确认候选生成和命令规划可复现，不宣称画面分割质量提升；真正的下一步是补齐 `datasets/` 确定性回放输入或实现可复跑 replay，然后由 `run_score.json` 和排行榜决定是否保留候选。当前候选生成脚本会从 `eval/search_space.yaml` 的 `arg` 字段读取真实 D455 命令行参数，基线配置不再携带尚未实现的 `--feature-profile=*` 开关。

当前转换器兼容“最终帧/单帧桥”，但 smoke 评分应优先使用多帧分析导出：D455 新增 `--analysis-export-every-n=N`，会额外写出 `cluster_map_frame_000030_metadata.json`、`final_segmentation_frame_000030_metadata.json` 这类按帧编号的 metadata；`scripts/convert_exports_to_analysis_run.py` 会把这些多帧 metadata 和 `profile.csv` 中对应帧合并成多行 `frame_metrics.csv` 与多帧 `cluster_metrics.jsonl`。这仍然不是确定性 replay，不适合选真正赢家，但已经能暴露最终帧桥漏掉的时间序列问题。转换模板目录时，converter 会更新模板或缺字段的 `run_manifest.json` / `config_snapshot.json`；需要强制更新时可加 `--update-manifest` 或 `--overwrite-config-snapshot`。`scripts/select_winners.py` 和 `scripts/make_codex_handoff.py` 现在只从 `pass=true` 的 run 中选择 winner/pareto/current best；如果没有合格 run，会写出 `no_pass_candidate`，避免失败配置被自动优化误选。

全画面 cluster map 的最终未覆盖区域不再作为 `Unknown` 计入 hard fail，而是作为低置信 `FarBackground` fallback 输出，source 为 `unassigned_far_background_remainder`。这表示“像素已有粗背景归属，但没有精确 3D 或明确视觉轮廓”，不是远距离精确测量。

小型 replay gate 的第一个通过候选是 `configs/candidates/round_001/candidate_0013.json`：在保留全画面 cluster map 的同时关闭重型 `color_segmentation` 和 `stereo_contour_distance`，三段固定 replay 均达到 `coverage=100%`、`unknown=0%`、p95 约 74-77ms 并通过 hard gate。它是速度基线，不是最终远距离粗距方案；后续应把双目粗距改成采样/低频路径，而不是每帧重开完整双目轮廓。

`configs/candidates/round_001/candidate_0014.json` 是第一版低频粗距候选：新增 `--color-contour-frame-interval=N`，默认 `1` 保持每帧重算；当设置为 `120` 时，D455 会在首个刷新帧提取彩图轮廓并执行双目轮廓估距，后续帧复用缓存的 `ColorContourRegion`，从而在静态 replay 的评分帧保留 `ApproxStereoContour` 粗距信息，同时避免每帧付出完整 `gray_prepare_ms` 成本。这个方案只作为固定 replay / 低运动场景的速度桥接；相机移动或物体移动时缓存轮廓可能滞后，后续仍需要 ROI 跟踪或运动触发刷新。

`configs/best/best_replay_static_far_distance.json` 把 `candidate_0014` 固化为阶段性 best bucket：只用于 deterministic/static replay gate，不替换 `best_overall`。`replay_small_matrix_static_001` 跑了 baseline/default、candidate_0001、candidate_0002、candidate_0013、candidate_0014 × 三个固定 case；6/15 通过 hard gate，baseline/0001/0002 全部因 `total_frame_ms_p95 > 100` 失败。`candidate_0014` 平均分略高于 `candidate_0013`，并拿到完整远场保留分，但 p95 已接近 100ms；`candidate_0013` 仍是更快的速度基线。最小 leaderboard 摘要提交在 `leaderboards/replay_small_matrix_static_001/`，完整 `analysis_runs/` 仍只作为本地证据。

`configs/candidates/round_001/candidate_0015.json` 是运动敏感缓存候选：在 `--color-contour-frame-interval=120` 基础上增加 `--color-contour-refresh-on-motion`、`--color-contour-refresh-on-unknown-spike` 和 `--color-contour-refresh-on-far-loss`。视觉运动触发使用下采样灰度差分，当前阈值为保守的 `--color-contour-refresh-motion-delta-percent=25`；unknown spike 和 far stereo loss 是下一帧恢复触发。静态 smoke 中 0015 仍有 2/3 case 因 p95 略超 100ms 未通过，因此它只代表已实现的待测策略，不进入 best。`eval/cases.yaml` 已新增 `slow_pan_far_object` 与 `hand_occlusion_reappear` 合同，用来专门暴露缓存轮廓滞后、遮挡后重现和粗距延迟更新问题；在这些数据集录入并通过验证前，`candidate_0015` 不代表已优于 `candidate_0014`。

缓存刷新现在会进入可度量输出：D455 在 `profile.csv` 追加 `color_contour_refreshed`、`color_contour_cache_reused`、`color_contour_refresh_motion`、`color_contour_refresh_unknown_spike`、`color_contour_refresh_far_loss`、`color_contour_motion_delta_percent` 和 stereo reuse 相关字段；converter 会把它们写入 `frame_metrics.csv`，并在刷新帧写 `events.csv` 的 `color_contour_refresh` 事件。`run_score.json` 也汇总 refresh/cache/reuse 计数，后续 motion gate 不再只靠 p95 和 far_score 间接推断触发策略是否有效。

`candidate_0016` / `candidate_0017` 是 `candidate_0015` 的保守变体：0016 使用 `--color-contour-frame-interval=180`、motion 阈值 35%、关闭 unknown-spike refresh，仅保留 far-loss 恢复；0017 保持 120 帧间隔，但把 motion 阈值提高到 40%、unknown 阈值提高到 12%。它们用于 motion-sensitive replay gate 的下一轮对照，不应在 `slow_pan_far_object` 和 `hand_occlusion_reappear` 评分前提升为 best。

`replay_static_refresh_variants_001` 已跑完 0016/0017 在 `near_single_object`、`far_cabinet`、`depth_hole_black_object` 上的静态 smoke，6/6 pass，且 `candidate_0017` 分数和 p95 最好；但该结果中 scored frames 的 `color_contour_refresh_count=0`、`color_contour_cache_reuse_count=3`，只说明静态缓存复用成本可控，不说明 motion/unknown/far-loss 触发在运动场景有效。最小摘要在 `leaderboards/replay_static_refresh_variants_001/`。

`configs/best/best_replay_static_refresh.json` 将 `candidate_0017` 固化为当前静态 refresh bucket 赢家；它不替换 `best_replay_static_far_distance.json`，后者仍保留为来自 `candidate_0014` 的低频粗距基础桶。`best_replay_static_refresh` 的适用范围限定为 deterministic/static replay 和低运动场景，不能代表 motion best。`scripts/select_winners.py` 的 leaderboard 现在直接输出 `color_refresh_count`、`color_refresh_motion_count`、`color_refresh_unknown_spike_count`、`color_refresh_far_loss_count`、`color_cache_reuse_count` 和 `color_stereo_reuse_p50`，后续 `replay_motion_gate_001` 不需要再逐个翻 `run_score.json` 才能判断刷新触发是否真的发生。

`slow_pan_far_object` 的第一段真实慢速平移 replay 显示，0015/0017 能触发 `color_contour_refresh_motion`，但刷新帧的 `gray_prepare_ms` 达到约 380-416ms，主要开销来自对全部彩图轮廓执行双目粗距匹配。为此新增 `--stereo-contour-max-regions-per-frame=N`，默认 32 保持旧行为；`candidate_0018` 先设为 8，但按面积优先仍会命中最大、最贵的区域。随后新增 `--stereo-contour-max-roi-area-percent=N`，默认 100 保持旧行为；`candidate_0019` 设为 12%，并把每帧实际 stereo match 尝试限制为 16 次，目标是跳过巨大平面/背景模板，让 motion refresh 不再产生 400ms 级别卡顿。风险是大区域可能暂时只有 2D 归属、没有双目粗距。

ROI/数量筛选只能小幅降低刷新帧耗时，不能根治同步 `matchTemplate` 卡顿。`--stereo-contour-reuse-cached-on-refresh` 改为在已有缓存后刷新彩图轮廓、但把上一批 stereo 粗距按 bbox IoU/中心距离转移到新轮廓；`candidate_0020` 使用该策略，目标是让 motion refresh 更新轮廓归属而不在同一帧重跑双目匹配。这个策略的距离会有短时滞后，后续更完整的方案应把 stereo 粗距做成分帧或异步后台刷新。

如果复用 stereo 后刷新帧仍超时，瓶颈就转移到彩图分割本身，尤其是 `pyrMeanShiftFiltering`。`candidate_0021` 在 0020 基础上把 `--color-segmentation-mean-shift-spatial=0` 和 `--color-segmentation-mean-shift-color=0`，用于验证轻量彩图轮廓刷新是否足够通过慢速平移 replay；风险是彩色连通区域会更碎，需要靠 leaderboard 的 split/merge 和后续可视化复核约束。

`candidate_0022` 继续压低轻量彩图刷新成本：颜色 bins 从 6 降到 4，最多保留 16 个彩图区域，最小区域提高到 1200 px。它是 motion gate 的性能探针，不代表最终细粒度分割策略；如果它通过但画面颗粒度明显下降，下一步应改为 ROI 局部刷新或分帧刷新，而不是继续全局降质量。

`candidate_0023` 是更激进的全局刷新下界探针：bins=3、最多 8 个彩图区域、min area=2000。它只用于判断全局彩图刷新是否存在可过 100ms p95 的参数空间；即使通过，也不能直接当最终质量方案。

`candidate_0024` 进一步把全局刷新压到最多 4 个大彩图区域、min area=3000，用来确认硬门槛的理论下界。它若通过，只能说明“极粗全局刷新能跑快”，不能说明分割质量达标。

真实慢速平移数据 `datasets/slow_pan_far_object` 已完成本地采集，包含 180 帧 color/depth16/left IR/right IR，并已按 case contract finalized/reviewed；检查视频为本地 `recordings/slow_pan_far_object_color_20260706_1549.mp4`，数据集和视频不进入 Git。`replay_motion_slow_pan_001` 跑了 0013、0014、0015、0017、0018-0025。评分器现在用完整 `profile.csv` 计算性能 p95，用导出帧 `frame_metrics.csv` 计算分割质量，并在 leaderboard 保留导出采样 p95、最大帧耗时、超 100ms 帧数作为顿挫风险指标。按完整 profile p95，`candidate_0024` 暂时领先，但仍有 1 帧超过 100ms；`candidate_0025` 证明异步刷新链路可用，但 p95 93.44ms、超 100ms 3 帧，当前不应替代 0024 或 promotion 为 motion best。

第一版多线程实现为 `--async-color-contour-refresh`：启动帧仍同步建立首个 cache；之后 motion/interval/unknown/far-loss 触发只在后台 worker 空闲时提交彩图轮廓刷新任务，主线程继续使用旧 cache；worker 完成后按 bbox IoU/中心距离转移 cached stereo 粗距并切换新 cache。profile/leaderboard 新增 `color_contour_async_submitted/applied/dropped/pending`、`color_contour_cache_age_frames` 和 `color_contour_async_worker_ms`，用于判断多线程是否只是把卡顿变成缓存滞后。`candidate_0025` 在 `candidate_0017` 质量参数上启用异步刷新和 cached stereo 转移；本轮结论是异步刷新应继续作为观察/后续优化路径，但后台全局分割会与主线程争 CPU，下一步应优先做 ROI 局部刷新、后台限速或拆分任务粒度。

评分器同步收紧了远场保留口径：`far_retention` 不再只看是否没有 `far_stereo_failed` 事件，而是先按 `approx_stereo_contour_pixels + image_only_contour_pixels + depth_hole_candidate_pixels` 的像素占比给基础分，再用 `stereo_matched_cluster_count` 给双目粗距加分。converter 也只在“存在彩图轮廓但完全没有有效 stereo 距离”时记录 `far_stereo_failed`，避免把部分轮廓未匹配误判成整帧远场粗距失败。这样自动优化不会因为关闭彩图/双目而虚假拿到远场满分。

确定性目录 replay 是自动优化进入真实评测的入口。目录格式为 `datasets/<case_id>/frames/000000_color.png`、`000000_depth16.png`、`000000_ir_left.png`、`000000_ir_right.png` 和 `case_manifest.json`；`depth16.png` 按 16-bit 毫米深度读取，右红外缺失时只影响双目轮廓粗距。D455 支持 `--replay-dir=datasets\<case_id>`，不需要连接相机；`scripts/run_batch.py --execute` 会按 `eval/cases.yaml` 的 `replay:` 字段依次执行 D455、converter 和 `score_run.py`。为避免启动帧污染 p95 和 unknown 指标，converter 和 run_batch 支持 `--ignore-first-n-frames=30`：

```powershell
python scripts\run_batch.py --candidates configs\candidates\round_001 --cases eval\cases.yaml --analysis-export-every-n=30 --ignore-first-n-frames=30 --execute
```

固定 case 的采集入口是 `--capture-replay-dir=datasets\<case_id>`，它会从真实 D455 保存 color、对齐后的毫米 depth16、左/右 IR 和 `case_manifest.json`。第一批不要采太长，建议每个 case 60 到 120 帧，先建立 `near_single_object`、`far_cabinet`、`depth_hole_black_object` 三个可复跑输入，再跑小型 Round 001：

```powershell
.\tools\Capture-ReplayCases.ps1 -Frames 120 -Warmup 30
```

采集脚本会按 case 暂停提示摆放场景，调用 `D455.exe --capture-replay-dir=... --quality-segmentation --stereo-contour-distance` 保存左右 IR，在每个 case 完成后运行 `scripts\validate_replay_dataset.py`，并默认调用 `scripts\finalize_replay_manifests.py` 从 `eval/cases.yaml` 自动写入 `reviewed: true`、`review_method: auto_case_contract_v1`、`expected` 和 tags。`datasets/` 是本地真实采集输入，默认不进入 Git；需要同步数据集时应单独确认数据大小、隐私和 LFS 策略。

这里的 reviewed 表示“按固定 case_id 合同接受为评测输入”，不是人工像素级真值标注。自动化闭环只要求 case 目录、帧格式和 `eval/cases.yaml` 中的 expected 合同一致；如果要做人眼复核，可另写 `review_method: manual_review_v1`。

小型 replay round 先只跑 3 个固定 case 和少量候选，`run_batch.py` 支持用 `--case-id`、`--candidate-id`、`--max-frames-override=120` 限定范围；`--skip-missing-replay` 只用于采集未完成时验证命令链，不代表有效评分：

```powershell
python scripts\validate_replay_dataset.py --case-dir datasets\near_single_object --case-dir datasets\far_cabinet --case-dir datasets\depth_hole_black_object --min-frames=120 --require-ir-left --require-ir-right --require-reviewed-manifest
python scripts\run_batch.py --candidates configs\baseline\default.json --cases eval\cases.yaml --case-id near_single_object --case-id far_cabinet --case-id depth_hole_black_object --max-frames-override=120 --analysis-export-every-n=30 --ignore-first-n-frames=30 --validate-replay --require-replay-ir --require-reviewed-manifest --execute
python scripts\run_batch.py --candidates configs\candidates\round_001 --candidate-id candidate_0001 --candidate-id candidate_0002 --cases eval\cases.yaml --case-id near_single_object --case-id far_cabinet --case-id depth_hole_black_object --max-frames-override=120 --analysis-export-every-n=30 --ignore-first-n-frames=30 --validate-replay --require-replay-ir --require-reviewed-manifest --execute
```

## 下一阶段工程化路线

当前项目已经进入工程化、可验证、可复现实验阶段。后续优先级不再是继续把所有能力塞进 `D455.cpp`，而是先建立可拆分、可回放、可量化的实验骨架：

| 阶段 | 目标 | 当前动作 | 判定方式 |
| --- | --- | --- | --- |
| P4.5 工程拆分 | 将采集、深度后处理、边界分析、材料提取、PCL、tracker、诊断显示分层 | 先保持单文件行为不变，新增独立 profile CSV 作为拆分前基线；后续再逐步搬到 `src/` 模块 | 拆分前后 2x2 画面、验收 CSV 字段和核心指标基本一致 |
| P5 全画面归簇 | 每个像素都有 `cluster_id`，近场给精确 3D，远场/无效深度先保留 2D 轮廓，剩余区域归 background/unknown | 已新增可选 `--cluster-map` 原型，导出最后一帧 `*_ids.png`、`*_overlay.png`、`*_metadata.json`；仍不改变默认 2x2 稳定显示 | `assignment_coverage_percent` 检查是否全像素有归属，`cluster_coverage_percent` 检查非 Unknown 有效归簇率，`unknown_percent` 检查未知区域占比 |
| P6 指标门槛 | 把“感觉更稳”变成 pass/fail | 用 acceptance CSV + profile CSV 汇总 p50/p90/p95、IoU、抖动和 flicker | 指标达到阶段阈值，且失败帧可定位 |
| P7 配置系统 | 减少命令行参数雪崩 | 后续增加 `configs/default.json` / `configs/fast.json`，命令行只做覆盖 | 一组效果参数能随仓库保存并复跑 |
| P8 构建复现 | 降低换机器成本 | 后续补 `vcpkg.json`、`CMakeLists.txt`、`CMakePresets.json`，保留现有 `.vcxproj` | 新机器可按 preset 构建 |
| P9 最小测试 | 防止纯算法行为被改坏 | 优先覆盖 config clamp、tracker 匹配、IoU、边界抖动和 cue selection | 无相机也能跑核心单元测试 |
| P10-P12 解释化 | 让 PCL/cue/tracker、双目粗距和输出语义可追踪 | 后续启用 IR2，新增远场 contour disparity；当前远场距离仍只沿用已有深度区间候选，不伪装成精确 3D | 每类失败能对应至少一个 CSV 字段或 overlay 证据 |

建议的长期文件结构如下，迁移时必须先保留行为基线，再拆模块：

```text
src/
  main.cpp
  camera/
    RealSenseCapture.h/.cpp
    DepthPostProcessor.h/.cpp
  segmentation/
    BoundaryAnalysis.h/.cpp
    MaterialExtractor.h/.cpp
    PclClusterRefiner.h/.cpp
    CueSelection.h/.cpp
  tracking/
    SegmentationTracker.h/.cpp
    StableContourMetrics.h/.cpp
  diagnostics/
    AcceptanceMetricsWriter.h/.cpp
    ProfileCsvWriter.h/.cpp
    IndoorPlaneDiagnostics.h/.cpp
    Visualizer.h/.cpp
  config/
    SegmentationConfig.h/.cpp
    CliOptions.h/.cpp
```

## 本机依赖

工程按当前机器的 vcpkg 路径配置：

- `D:\vcpkg\installed\x64-windows\include`
- `D:\vcpkg\installed\x64-windows\include\opencv4`
- `D:\vcpkg\installed\x64-windows\lib`
- `D:\vcpkg\installed\x64-windows\debug\lib`

依赖库：

- `realsense2`
- `opencv_core`
- `opencv_imgproc`
- `opencv_highgui`
- `pcl_common`
- `pcl_search`
- `pcl_kdtree`
- `pcl_segmentation`
- `flann_cpp`

## 构建

建议使用 x64：

```powershell
msbuild .\D455.vcxproj /p:Configuration=Debug /p:Platform=x64 /m
```

## 运行

构建后事件会调用 vcpkg `applocal.ps1`，把 `D455.exe` 依赖的 DLL 自动复制到输出目录。

```powershell
.\x64\Debug\D455.exe
```

可选参数：

```powershell
.\x64\Debug\D455.exe --min-depth-mm=250 --max-depth-mm=3500 --min-area-px=900
```

分割颗粒度调参：

```powershell
.\x64\Debug\D455.exe --depth-slice-mm=250 --max-area-percent=35 --max-roi-area-percent=60 --min-area-px=700
```

稳定轮廓视图是默认模式。主窗口用 2x2 同步显示：右上显示当前稳定点/候选轮廓/稳定轮廓信息，左下只保留稳定轮廓内部彩图，右下单独显示左下黑区对应的彩图，方便直接观察轮廓是否抖动、漂移、漏分或误合并。

默认有效距离仍由 `--max-depth-mm=3500` 控制，有效距离内的候选、稳定轮廓和彩图拼接流程不变。超过有效距离但仍在 `--far-max-depth-mm=12000` 内的深度点，会作为低置信远距候选只叠加到右上分割视图：程序用远距深度掩码叠加红外/灰度边界提取粗轮廓，用中位深度排序，显示 `far intervals near->far: F1<F2...`，并在轮廓旁标出类似 `F1 nearest 3.5-5.0m`、`F2 farthest 5.0-6.5m` 的区间距离。`F1/F2` 表示当前帧远距候选的相对近远顺序，不表示已经确认存在；验收 CSV 会追加 `far_candidate_count`、最近/最远远距候选的区间和中位深度字段：

```powershell
.\x64\Release\D455.exe --far-distance-intervals
.\x64\Release\D455.exe --far-max-depth-mm=12000 --far-interval-mm=1500 --far-min-area-px=600
.\x64\Release\D455.exe --no-far-distance-intervals
```

远距候选和近处水平平面候选默认只叠加到右上诊断视图，仍只属于显示/观察材料层，不进入稳定 tracker，也不影响左下稳定拼接图。近处水平平面用于把桌面这类大平面作为一个可视候选补出来；默认每 10 帧刷新一次并缓存显示，避免拖慢主链；如果误把地面或柜面纳入，可以收紧中心位置、最大深度或最小面积。需要临时检查远距/桌面候选在彩图拼接里的覆盖效果时，再显式启用 `--extra-candidates-in-mosaic`。2x2 输出默认在处理后裁剪掉对齐深度中缺少有效双目重叠的边缘，首次估计后锁定裁剪框，并向内额外裁 2px；每个面板固定缩到原图尺寸的 75%，所以窗口尺寸不会随深度边缘抖动而变化：

```powershell
.\x64\Release\D455.exe --near-plane-display
.\x64\Release\D455.exe --near-plane-max-depth-mm=1800 --near-plane-min-center-y-percent=45
.\x64\Release\D455.exe --near-plane-frame-interval=5 --near-plane-sample-step-px=12
.\x64\Release\D455.exe --extra-candidates-in-mosaic
.\x64\Release\D455.exe --processed-view-scale-percent=75 --overlap-trim
.\x64\Release\D455.exe --overlap-trim-extra-crop-px=2
.\x64\Release\D455.exe --processed-view-scale-percent=100 --no-overlap-trim
```

视频 `recordings\d455_record_20260703_181435.avi` 的抽帧检查显示，左下稳定拼接非黑比例在约 23% 到 63% 间跳变，主要由贴边大背景块、大面积稳定材料和短时碎块进入拼接引起。曾增加 `mosaic foreground gate` 作为左下拼接过滤实验，但 `recordings\d455_record_20260703_192619.avi` 显示该方向会让启动阶段左下过黑。当前显示路线改为五画面：第三格恢复原稳定轮廓拼接路径，不再叠加 `mosaic foreground gate` / track 质量门控；第五格改为“非精度区域彩图归属”诊断图。第五格只把彩图轮廓中两类非近场精度像素画到黑底：一类是 `depth > maxDepthMm` 且仍在 `farMaxDepthMm` 内的远距深度像素，可用于区间远近；另一类是 `depth == 0` 的无深度像素，只按彩图轮廓判断属于哪个 cluster，不把它当成远处或超出有效距离。室内画面通常不应理解为全部超出 D455 距离范围，更多是有效 depth 像素不完整、黑色材质/遮挡/反射/角度导致当前深度支撑不足。需要继续排查旧门控参数时仍可保留这些开关作为实验参数，但默认观察重点应放在第三格原路径和第五格“远距/无深度彩图归属”之间的互补关系：

```powershell
.\x64\Release\D455.exe --no-mosaic-foreground-gate
.\x64\Release\D455.exe --mosaic-max-material-area-percent=45 --mosaic-border-reject-area-percent=8
.\x64\Release\D455.exe --mosaic-min-track-observations=20 --mosaic-min-track-score=30 --mosaic-max-track-stale-frames=5
.\x64\Release\D455.exe --mosaic-near-min-track-observations=1 --mosaic-near-min-track-score=0
```

实时模式默认直接用稳定轮廓掩码拼接彩图，以降低相机移动时的显示滞后。需要诊断 RGB 边缘能否补齐稳定轮廓时，可以显式启用严格 RGB 轮廓补齐：只在稳定轮廓的小邻域里用彩图生成候选补齐轮廓；如果候选和原稳定轮廓的 IoU、面积变化、中心偏移都在阈值内，且候选不会进入其他稳定轮廓的隔离带，左下/右下两格才采用彩图补齐后的轮廓，否则继续使用原稳定轮廓。未知背景不会被强行分配给任何前景轮廓，也不会为了补齐而把两个轮廓融合：

```powershell
.\x64\Release\D455.exe --color-contour-completion
```

主窗口现在使用五画面布局，录制视频也保存同一布局。顺序为：1 原始彩图，2 稳定/候选诊断，3 原路径稳定轮廓彩图拼接，4 稳定轮廓外区域彩图，5 非精度区域彩图归属图。第五格不是完整分割图，也不是单纯远距图；它用于观察彩图轮廓里没有进入 D455 近场高精度深度路径的区域。显示时亮色表示有深度值但超出 `maxDepthMm` 的远距区间像素，浅色表示 `depth == 0` 的无深度像素；浅色区域只说明 2D 彩图轮廓归属，不说明远近。默认阈值是 `--non-precise-color-ownership-min-percent=10`，即一个彩图轮廓中“远距深度像素 + 无深度像素”的占比达到 10% 才显示到第五格。显示窗口模式会在主窗口上创建 `Color owner %` 滑条，可运行时手动调节这个阈值；`--no-display` 模式仍使用命令行参数。旧参数 `--d455-precision-min-far-depth-support-percent` 和 `--d455-precision-min-color-depth-support-percent` 保留为兼容别名。超过四个画面时窗口自动使用 3 列布局，避免第五格把窗口拉得过高。

主窗口模式默认不录制，优先保持实时窗口低延迟。需要保存当前五画面同步画面时，可以在启动时显式传入 `--record-video` 或 `--record-video=...`，也可以在运行后让命令窗口或 OpenCV 窗口获得焦点并按 `r` 开始录制、按 `s` 停止录制、按 `q` 或 `Esc` 退出。运行时按 `r` 打开的默认路径为 `D:\D455\recordings\d455_record_时间戳.avi`；如果启动时给了固定 `--record-video=...`，后续重复开始录制会自动给新文件追加时间戳，避免覆盖上一段：

```powershell
.\x64\Release\D455.exe --record-video
.\x64\Release\D455.exe --record-video=recordings\d455_stable_contours.avi --record-fps=30
.\x64\Release\D455.exe --record-command-control
.\x64\Release\D455.exe --no-record-command-control
```

默认只有显示窗口模式启用运行时录制命令；`--no-display` 批处理默认不启用，以免为了等待命令而额外生成 2x2 画面。确实需要无显示模式下从命令窗口按键开始/停止录制时，显式传入 `--record-command-control`。

性能分析时可以关闭窗口显示和验收录制，只保留 CSV 指标；验收 CSV 会追加各阶段耗时字段，方便区分算法、渲染、显示和录制开销。需要只定位速度瓶颈时，优先用独立的 `--profile-csv`，它不要求启用 P0 验收，也不会自动录像；输出字段包含 `total_ms`、`processing_ms`、`capture_wait_align_ms`、`depth_post_ms`、`boundary_ms`、`extract_ms`、`pcl_ms`、`tracker_ms`、`render_ms`、`display_ms`、`record_ms` 以及候选/稳定轮廓计数。注意 `capture_wait_align_ms` 包含等待相机帧和 RealSense 对齐时间，不等同于纯算法处理耗时；判断 30fps 处理瓶颈时重点看 `processing_ms` 和各算法分段：

```powershell
.\x64\Release\D455.exe --acceptance-baseline --acceptance-no-record --no-display --max-frames=600
.\x64\Release\D455.exe --profile-csv=recordings\profile_fast.csv --no-display --max-frames=600
.\x64\Release\D455.exe --profile-csv --no-display --max-frames=600
```

全画面归簇是可选 P5 原型，默认关闭，不改变现有稳定候选显示。启用后程序会在最后一帧导出三类文件：`*_ids.png` 是 16-bit cluster id 图，每个像素都有非零归属；`*_overlay.png` 是叠加到原始彩图上的调试图；`*_metadata.json` 记录每个 cluster 的 `mode`、来源、像素数、2D bbox、中心、深度统计和置信度。当前分层口径如下：

| mode | 含义 | 空间精度口径 |
| --- | --- | --- |
| `PreciseDepth3D` | 当前稳定 tracker 输出的近场深度稳定轮廓 | 可使用 D455 深度统计，属于较高精度 3D 观察材料 |
| `ApproxStereoContour` | 超出有效距离但仍有深度区间或双目轮廓支持的远距候选 | 只表示粗距或近远顺序，不是精确 3D |
| `ImageOnlyContour` | 不依赖有效深度的 IR/RGB 视觉轮廓 | 只确认 2D 轮廓归属，距离未知 |
| `BackgroundPlane` | 室内平面/近处水平面诊断得到的背景结构 | 背景结构归属，不进入前景稳定 tracker |
| `FarBackground` | 远处背景结构 | 远场背景归属，不作为前景目标 |
| `DepthHoleCandidate` | 无效深度空洞中仍有视觉证据的候选 | 保留存在可能，不提供精确距离 |
| `Unknown` | 剩余像素 | 为保证全画面覆盖的未知归属 |

```powershell
.\x64\Release\D455.exe --cluster-map --no-display --max-frames=120
.\x64\Release\D455.exe --cluster-map-export=recordings\cluster_map_p5 --no-display --max-frames=120
.\x64\Release\D455.exe --cluster-map --cluster-map-visual-min-area-px=500 --cluster-map-max-visual-regions=32
python scripts\convert_exports_to_analysis_run.py --run-dir=analysis_runs\smoke --cluster-map=analysis_runs\smoke\cluster_map_metadata.json --final-segmentation=analysis_runs\smoke\final_segmentation_metadata.json --profile-csv=analysis_runs\smoke\profile.csv
```

注意：P5 原型只解决“每个像素有归属”和“深度精度分层标注”。右红外 IR2 + 双目轮廓偏差粗距属于后续 P6，不应把当前 `ImageOnlyContour` 当成已有距离估计。

质量优先分割在原深度稳定流程之后追加彩图轮廓辅助层，默认开启，但不删除原有 depth slice、稳定点、cue selection 和 tracker。流程是：先按原路径生成深度候选，再用 RGB 彩图做不依赖有效深度的轮廓分割；彩图层同时使用 Canny 外轮廓和 Lab 颜色量化连通块，先做均值漂移平滑，再按颜色分箱取连通区域，避免只剩少量细边线。若彩图轮廓和深度候选有足够重叠，就用彩图轮廓掩码替换深度候选的 2D 轮廓，同时保留该掩码内的 D455 深度统计。彩图辅助层会拒绝 bbox 过大或贴边且横跨画面的区域，避免把人体、窗帘、墙面粘成一个跨画面大掩码；靠边但不横跨画面的近处主体默认允许进入候选。如果启用右红外，程序会用左右红外轮廓/边缘模板的 x 位移估算粗略距离和不确定度，并按 `距离 * bbox像素 / 焦距像素` 输出粗略宽高。该粗距和粗尺寸只用于远近排序和尺寸估计参考，不是精确深度。需要导出最终一帧分割图时指定 `--final-segmentation-export`，会生成 `*_ids.png`、`*_overlay.png` 和 `*_metadata.json`，metadata 中包含 `assigned_pixel_count`、`stereo_distance_valid_count`、彩图区域 `estimated_size_mm` 和深度材料 `size_3d_mm`：

```powershell
.\x64\Release\D455.exe --final-segmentation-export=recordings\final_segmentation_sample --no-display --max-frames=120
.\x64\Release\D455.exe --quality-segmentation --color-refine-depth-masks --stereo-contour-distance
.\x64\Release\D455.exe --color-segmentation-min-area-px=500 --color-refine-min-overlap-percent=15
.\x64\Release\D455.exe --color-segmentation-max-roi-area-percent=55 --color-segmentation-border-reject-area-percent=28
.\x64\Release\D455.exe --color-segmentation-color-bins=6 --color-segmentation-mean-shift-spatial=9 --color-segmentation-mean-shift-color=18
.\x64\Release\D455.exe --no-color-refine-depth-masks
.\x64\Release\D455.exe --no-stereo-contour-distance
```

当前质量优先分割的边界口径：

| 层 | 输入 | 输出 | 说明 |
| --- | --- | --- | --- |
| 原深度链 | D455 depth + 红外/深度边界 + 稳定点 | 深度候选材料 | 原流程保留，仍是近场精确 3D 的主依据 |
| 彩图轮廓层 | RGB Canny + morphology + contour | 全画面 2D 轮廓候选 | 不依赖有效深度，避免远处/无效深度区域直接被入口过滤 |
| 掩码优化层 | 深度候选与彩图轮廓重叠 | 用彩图轮廓修正后的深度候选 | 只有重叠率和面积变化在阈值内才替换，避免彩图误分割污染深度结果 |
| 双目粗距层 | 左右红外边缘模板位移 | `estimated_distance_mm` / `distance_uncertainty_mm` | 只给粗略远近和尺寸估计，不进入精确 3D 判断 |
| 最终分割图 | 修正后深度候选 + 未覆盖彩图轮廓 | id map + overlay + metadata | 用于检查一帧最终分割归属 |

实时运行默认启用 `--realtime-30`，目标是把主处理链压到 33ms 以内，避免相机移动时旧帧积压。实时档默认跳过 RealSense 深度后处理、使用轻量右上格、用 450mm 深度切片和略稀疏的稳定点采样，并关闭每帧 PCL 聚类、灰度/红外切分和灰度边局部深度确认切分；稳定输出仍需要稳定深度点和历史 tracker 确认。需要回到质量优先全链，或在现场噪声较高时恢复深度后处理：

```powershell
.\x64\Release\D455.exe --no-realtime-30
.\x64\Release\D455.exe --no-realtime-30 --pcl-clustering --color-contour-completion
.\x64\Release\D455.exe --realtime-30 --no-realtime-skip-depth-post
.\x64\Release\D455.exe --realtime-30 --no-realtime-depth-only-boundary --no-realtime-disable-depth-confirm-split
.\x64\Release\D455.exe --realtime-30 --no-realtime-disable-depth-confirm-split
.\x64\Release\D455.exe --realtime-30 --no-realtime-fast-depth-post --no-realtime-disable-pcl
```

需要保留视频但降低写盘压力时，可以抽帧和缩放录制：

```powershell
.\x64\Release\D455.exe --acceptance-baseline --record-every-n=2 --record-scale-percent=50 --max-frames=600
```

P0 基线验收模式默认打开录制，并为视频生成同名 CSV 指标文件；传入 `--acceptance-no-record` 或 `--no-record-video` 时只生成 CSV。普通主窗口运行不会因为设置 `--record-fps` 等录制参数而自动开启录制，仍需显式 `--record-video`。默认记录每帧处理耗时、候选数、稳定轮廓数、稳定区/黑区比例、跨帧 IoU、边界抖动近似值和分段耗时：

```powershell
.\x64\Release\D455.exe --acceptance-baseline --max-frames=600
.\x64\Release\D455.exe --acceptance-baseline --acceptance-label=acceptance_p0 --max-frames=600
.\x64\Release\D455.exe --acceptance-baseline --record-video=recordings\acceptance_p0.avi --acceptance-csv=recordings\acceptance_p0.csv --max-frames=600
```

阶段验收表暂定如下，后续有固定回放集后再把阈值按场景细化。当前先用真实 D455 录制和 CSV 汇总判断是否退化：

| 阶段 | 关注指标 | 建议门槛 | 典型失败信号 |
| --- | --- | --- | --- |
| P0 稳定性/速度 | `processing_ms` p95、`avg_contour_iou`、`boundary_jitter_px`、`stable_count` 抖动 | p95 尽量 <= 33ms；质量优先模式可放宽到 <= 50ms；连续稳定轮廓 IoU 均值尽量 >= 0.85；边界抖动 p90 尽量 <= 3px | 相机移动时视觉残留、稳定轮廓闪烁、窗口尺寸变化 |
| P1 边界质量 | `split_boundary_px`、`gray_depth_confirmed_edge_px`、`depth_step_edge_px`、`depth_hole_edge_px` | 边界像素不应异常飙升；深度确认边应随真实物体边界变化 | 大框误检、斜面被切成碎片、空洞边误切 |
| P2 cue selection | `cue_rejected_texture_count`、`cue_accepted_by_depth`、`cue_accepted_by_anchor`、`stable_count` | 纹理拒绝上升时，真实稳定轮廓数量不应明显下降 | 纹理边误检、真实物体被 cue 拦掉 |
| P3 平面诊断 | `indoor_plane_cached`、`indoor_plane_px`、`indoor_support_px`、稳定前景遮罩 | 平面诊断应稳定命中，且不吞掉稳定前景 | 墙/桌/天面诊断漂移，平面 mask 压住前景 |

姿态读取是可选旁路能力，默认不改变 RGBD 分割和稳定观察单元。打开 `--pose-read` 后，程序会在设备支持时同时启用 accel/gyro motion stream：roll/pitch 由加速度重力方向估计，`yaw_rel` 由陀螺仪短时积分得到，只能作为本次启动后的相对航向参考，会随时间漂移。默认会把姿态摘要叠加到原始彩图窗口和录制视频；验收 CSV 只在启用姿态时追加 `pose_*` 字段：

```powershell
.\x64\Release\D455.exe --pose-read
.\x64\Release\D455.exe --acceptance-baseline --pose-read --pose-log --pose-log-every-n=30 --max-frames=300
.\x64\Release\D455.exe --pose-read --no-pose-overlay --pose-smooth-percent=10
```

启用姿态读取时，左上原始彩图格默认用粗黄色箭头从画面中心标出 IMU 加速度估计的重力方向投影，并标注主导轴和加速度模长。`--no-gravity-line` 可以只保留文字姿态，不画重力箭头：

```powershell
.\x64\Release\D455.exe --pose-read --pose-overlay --gravity-line
.\x64\Release\D455.exe --motion-diagnostics --pose-read --pose-overlay --gravity-line --record-video
.\x64\Release\D455.exe --pose-read --no-gravity-line
```

### IMU Z 轴是否可当作重力轴的确认方案

不要直接假设 D455 IMU 的 Z 轴就是重力方向轴。静止时 accelerometer 给出的是当前相机姿态下的重力/支撑加速度方向在 IMU 坐标系中的投影；如果相机发生俯仰或横滚，重力方向会在 X/Y/Z 三轴之间重新分配。因此确认口径分两层：

- 只确认当前固定安装姿态：用 `--imu-gravity-check` 静止采样，判断 Z 分量是否稳定占主导。
- 确认 IMU 轴映射：对相机做六面静止测试，分别让不同外壳面朝上/朝下，检查 `best_gravity_axis` 是否按物理翻转在 `+X/-X/+Y/-Y/+Z/-Z` 间切换。

当前固定姿态的可复跑命令：

```powershell
.\x64\Release\D455.exe --imu-gravity-check
.\x64\Release\D455.exe --imu-gravity-check --imu-gravity-frames=300 --imu-gravity-csv=recordings\imu_z_static.csv
```

默认判定条件：

- `accel_norm_mps2` 接近 9.80665，误差不超过 `--imu-gravity-norm-tolerance-percent=15`。
- `z_axis_angle_deg <= --imu-gravity-z-max-angle-deg=10`。
- `z_axis_dominance_percent >= --imu-gravity-z-min-dominance-percent=90`。
- `accel_std_*` 最大值不超过 `--imu-gravity-max-accel-std-milli-mps2=250`，表示采样期间相机足够静止。
- 如果 gyro 可用，`gyro_rms_radps <= --imu-gravity-max-gyro-milliradps=50`。

程序会输出 `best_gravity_axis` 和 `z_axis_can_be_gravity`，并默认保存到 `recordings\imu_gravity_check_时间戳.csv`。只有 `z_axis_can_be_gravity=1` 时，才可以在当前固定安装姿态下临时把 IMU Z 轴当作重力方向轴；如果相机运行中会移动或倾斜，应使用 `pose_accel_x/y/z` 的完整重力向量实时估计 roll/pitch，而不是把 Z 轴硬编码为重力方向。

### 实时晃动检测与显示

打开 `--motion-diagnostics` 后，主循环会增加一个旁路运动诊断，不改变稳定轮廓筛选和输出归属。诊断同时使用两类信号：

- 视觉信号：对相邻彩图灰度帧做全局相位相关，估计整幅画面的位移，输出 `motion_visual_shift_*`。
- IMU 信号：如果设备支持 accel/gyro，自动读取 motion stream，输出角速度模长和加速度变化量；如果显式传入 `--no-pose-read`，则只使用视觉信号。

左上原始彩图格会显示 `motion stable / moving / shaking`、综合分数、视觉位移和 IMU 摘要。验收 CSV 会追加 `motion_*` 列，方便复查相机移动时的晃动强度：

```powershell
.\x64\Release\D455.exe --motion-diagnostics
.\x64\Release\D455.exe --motion-diagnostics --motion-log --motion-log-every-n=30
.\x64\Release\D455.exe --motion-diagnostics --acceptance-baseline --acceptance-no-record --max-frames=300
.\x64\Release\D455.exe --motion-diagnostics --no-pose-read
```

默认阈值以原始彩图像素为单位：`--motion-visual-move-millipx=2000` 表示相邻帧全局位移平滑值约 2px 进入 `moving`，`--motion-visual-shake-millipx=8000` 表示约 8px 进入 `shaking`。现场如果手持相机较敏感，可以提高阈值；如果要更早提示轻微晃动，可以降低阈值：

```powershell
.\x64\Release\D455.exe --motion-diagnostics --motion-visual-move-millipx=1000 --motion-visual-shake-millipx=5000
.\x64\Release\D455.exe --motion-diagnostics --motion-gyro-move-milliradps=60 --motion-gyro-shake-milliradps=180
```

视觉位移表示“画面整体变化”，强运动物体经过镜头时也可能触发；IMU 信号表示相机本体运动。两者同时升高时，基本可以判定为相机移动或晃动造成的画面不稳定。

P1 IR-D 边界类型验收会额外打开 `D455 IR-D boundary diagnostics`，第五栏显示边界来源：红色为深度突变，洋红为深度空洞/无效深度交界，蓝色为红外/灰度边，青色为深度边附近的灰度边，绿色为局部深度跨度确认的灰度边，黄色为最终切分边界。CSV 会同步记录各类边界像素数。深度空洞边默认只诊断和计数，不参与最终切分：

```powershell
.\x64\Release\D455.exe --acceptance-baseline --acceptance-label=acceptance_p1 --boundary-diagnostics --max-frames=600
```

P2 cue selection 默认启用。它在稳定锚点候选进入历史 tracker 前做轻量择机：深度边、深度确认灰度边、强稳定锚点任一可信则保留；只有明显灰度纹理边占主导且深度/灰度确认和锚点都弱的候选会被挡住，已有稳定 track 会由历史保持机制短时接管。右上格底部显示 cue 统计，CSV 会同步记录 cue 输入、保留、纹理拒绝和各类接受原因：

```powershell
.\x64\Release\D455.exe --acceptance-baseline --acceptance-label=acceptance_p2 --boundary-diagnostics --max-frames=600
```

P3 室内大平面诊断默认不影响分割结果。打开 `D455 indoor plane diagnostics` 后，独立诊断窗口按低频降采样法线估计显示大平面背景结构：蓝色为墙面类，洋红为天面类，橙色为支撑面类，绿色边线为当前稳定轮廓遮罩。平面 mask 会扣掉稳定前景，不会覆盖左下/右下两格的稳定轮廓结果。触顶的大平面会按上方带保守拆出天面类，避免整块墙/天面粘成一个诊断类。CSV 会同步记录大平面像素数、墙面/天面/支撑面像素数、组件数和是否复用缓存：

```powershell
.\x64\Release\D455.exe --acceptance-baseline --acceptance-label=acceptance_p3 --boundary-diagnostics --indoor-plane-diagnostics --max-frames=600
```

默认输出以细颗粒稳定观察单元为主：候选需要有稳定深度点支撑，并通过历史 tracker 后才进入稳定输出。大平面合并是可选增强，发生在稳定深度点筛选之后、cue/tracker 之前：先把稳定深度点膨胀成稳定点簇，并用真实切分边界阻断跨物体连接；再把同一稳定点簇内由固定深度切片造成的相邻碎片合成一个平面候选。只有需要把斜着看的墙面、桌面等大平面显示成连续轮廓时再打开：

```powershell
.\x64\Release\D455.exe --no-stable-plane-merge
.\x64\Release\D455.exe --stable-plane-merge --stable-plane-merge-anchor-gap-px=14 --stable-plane-merge-mask-gap-px=21 --stable-plane-merge-min-anchors=24
```

彩图轮廓默认只做严格补齐，不改变观察单元的物理归属。录制视频显示 `color contour completion adopted=0` 时，说明严格彩图补齐过于保守；如果只为显示连续大轮廓，可以显式打开彩图主轮廓门槛。普通候选仍按严格 IoU/面积/中心偏移检查；高稳定点支撑候选允许更大的彩图轮廓，只要它保留原深度候选足够重叠、不覆盖其他稳定候选，并受最大面积限制：

```powershell
.\x64\Release\D455.exe --color-contour-primary --color-contour-primary-min-anchors=24 --color-contour-primary-min-overlap-percent=45 --color-contour-primary-max-area-delta-percent=220
```

稳定点筛选主要由采样步长、邻域大小、局部深度稳定阈值、原始/滤波深度一致性阈值控制：

```powershell
.\x64\Debug\D455.exe --rgb-depth-anchor-step-px=4 --rgb-depth-anchor-neighborhood-px=2 --rgb-depth-anchor-max-neighbor-range-mm=35 --rgb-depth-anchor-max-raw-filtered-gap-mm=60
```

如果轮廓边缘附近的深度噪声影响稳定点，可以增大边缘排除范围：

```powershell
.\x64\Debug\D455.exe --rgb-depth-anchor-edge-dilate-px=4
```

稳定轮廓至少需要一定数量的稳定点支撑，数值越大越保守：

```powershell
.\x64\Debug\D455.exe --stable-contour-min-anchors=24
```

PCL 欧式聚类默认启用。`--pcl-cluster-tolerance-mm` 是点云聚类最关键参数：太大会把相邻物体标成同一 3D cluster，太小会让同一物体的片段难以合并。默认使用 `--pcl-sample-step-px=3`、`--pcl-max-input-points=7000` 和 `--pcl-frame-interval=2` 控制耗时：

```powershell
.\x64\Debug\D455.exe --pcl-cluster-tolerance-mm=35 --pcl-min-cluster-points=80 --pcl-sample-step-px=3 --pcl-max-input-points=7000 --pcl-frame-interval=2
```

如果要更快，优先增大采样步长和 PCL 间隔；如果要更细，降低采样步长和间隔：

```powershell
.\x64\Debug\D455.exe --pcl-sample-step-px=4 --pcl-frame-interval=3
.\x64\Release\D455.exe --pcl-sample-step-px=2 --pcl-frame-interval=1
```

需要临时退回 OpenCV 候选轮廓时：

```powershell
.\x64\Debug\D455.exe --no-pcl-clustering
```

历史稳定层默认启用。新候选连续命中 `--track-confirm-frames` 后才显示；已稳定材料允许丢失 `--track-miss-frames` 帧后再删除：

```powershell
.\x64\Debug\D455.exe --track-confirm-frames=3 --track-miss-frames=5
```

历史匹配同时看 ROI 重叠、中心距离、平均深度差和面积比例。画面稳定但输出仍跳时，可以放宽匹配并提高平滑；如果错误粘连，应收紧这些阈值：

```powershell
.\x64\Debug\D455.exe --track-center-gap-px=60 --track-depth-gap-mm=350 --track-iou-percent=15 --track-smooth-percent=75
```

需要对比原始单帧分割时：

```powershell
.\x64\Debug\D455.exe --no-history-tracking
```

默认最多保留面积最大的 24 个材料片段参与稳定轮廓筛选。需要保留更多片段时：

```powershell
.\x64\Debug\D455.exe --max-materials=32
```

默认已经启用“深度边缘 + 深度支持的左红外灰度边缘 + 深度确认的红外轮廓边界”参与切分。深度边缘仍提供基础切分；不稳定边界优先由左红外灰度轮廓确定位置，局部深度只负责确认该轮廓附近确实存在足够深度变化。RGB 彩色流不再参与主分割，只在最后按稳定轮廓剪切彩图。需要临时关闭灰度边界参与切分时：

```powershell
.\x64\Debug\D455.exe --no-color-split
.\x64\Debug\D455.exe --no-gray-split
```

如果只想关闭“灰度轮廓由深度确认”的新增路径，保留深度边缘和深度支持灰度边缘：

```powershell
.\x64\Debug\D455.exe --no-contour-depth-confirm-split
```

深度空洞边默认不参与切分。如果需要专门验证无效深度交界能否改善遮挡边，可以显式启用：

```powershell
.\x64\Debug\D455.exe --depth-hole-split
```

需要临时关闭 P2 cue selection、直接把稳定锚点候选交给历史 tracker 时：

```powershell
.\x64\Debug\D455.exe --no-cue-selection
```

需要和旧版“只按深度切片连通域、不扣边界线”的结果对比时：

```powershell
.\x64\Debug\D455.exe --no-boundary-split
```

边缘线和切分边界调参：

```powershell
.\x64\Debug\D455.exe --depth-canny-low=8 --depth-canny-high=28 --infrared-canny-low=40 --infrared-canny-high=120 --contour-depth-confirm-radius-px=4 --contour-depth-confirm-min-range-mm=25 --contour-depth-confirm-min-valid-px=8 --depth-hole-edge-px=3 --cue-min-reliable-edge-px=8 --cue-min-confirmed-gray-edge-px=6 --cue-max-texture-only-percent=75 --split-boundary-px=3 --min-edge-px=12 --contour-line-px=1
```

如果右上格仍然出现整幅画面的大框，优先降低：

- `--max-area-percent`
- `--max-roi-area-percent`
- `--max-border-area-percent`

贴边的人体、手臂、近距离物体默认按前景保护：只要组件最近深度或平均深度不超过 `--foreground-keep-depth-mm`，就不会因为贴边或略大于普通面积阈值被直接删除。若近处主体仍被裁掉，可放宽：

```powershell
.\x64\Debug\D455.exe --foreground-keep-depth-mm=2200 --foreground-max-area-percent=80 --foreground-max-roi-area-percent=95
```

为了避免单个大连通域把左下格几乎铺满，最终材料还有独立面积门控。默认单个材料最大轮廓面积 35%、ROI 最大 65%；如果左下格只剩少数几个大轮廓，优先降低它们：

```powershell
.\x64\Release\D455.exe --max-material-area-percent=25 --max-material-roi-area-percent=50
```

如果候选被切得太碎，优先增大：

- `--depth-slice-mm`
- `--min-area-px`

自动运行指定帧数后退出：

```powershell
.\x64\Debug\D455.exe --max-frames=30
```

用 RGB 图自行估算逐像素深度，并用对齐后的 D455 深度作为参考跑 20 帧准确性评估：

```powershell
.\x64\Release\D455.exe --rgb-depth-accuracy-test --rgb-depth-frames=20 --rgb-depth-warmup=30 --rgb-depth-eval-step-px=1 --rgb-depth-onnx=models\midas\model-small.onnx
```

两条线运行：

```powershell
# 线 1：红外/双目深度只提供稀疏可信基准点，不生成密集深度
.\x64\Release\D455.exe --rgb-depth-anchor-only --rgb-depth-frames=20 --rgb-depth-warmup=30 --rgb-depth-anchor-step-px=4 --rgb-depth-anchor-neighborhood-px=2 --rgb-depth-anchor-max-neighbor-range-mm=35 --rgb-depth-anchor-max-raw-filtered-gap-mm=60 --rgb-depth-anchor-edge-dilate-px=3 --rgb-depth-anchor-holdout-percent=30

# 线 2：RGB 单目深度用线 1 的训练基准点矫正，并用留出基准点验证
.\x64\Release\D455.exe --rgb-depth-accuracy-test --rgb-depth-anchor-correction --rgb-depth-frames=20 --rgb-depth-warmup=30 --rgb-depth-eval-step-px=1 --rgb-depth-anchor-step-px=4 --rgb-depth-anchor-neighborhood-px=2 --rgb-depth-anchor-max-neighbor-range-mm=35 --rgb-depth-anchor-max-raw-filtered-gap-mm=60 --rgb-depth-anchor-edge-dilate-px=3 --rgb-depth-anchor-holdout-percent=30 --rgb-depth-onnx=models\midas\model-small.onnx

# 稳定点为中心的轮廓稳定性评估：稳定点支撑 -> 轮廓中心/深度校准 -> 历史跟踪 -> track 稳定性汇总
.\x64\Release\D455.exe --stable-contour-test --rgb-depth-frames=20 --rgb-depth-warmup=30 --rgb-depth-anchor-step-px=4 --rgb-depth-anchor-neighborhood-px=2 --rgb-depth-anchor-max-neighbor-range-mm=35 --rgb-depth-anchor-max-raw-filtered-gap-mm=60 --rgb-depth-anchor-edge-dilate-px=3 --stable-contour-min-anchors=16 --stable-contour-top=12 --stable-contour-save=stable_contours_last.png

# 连续视频显示：实时刷新稳定点、候选轮廓、稳定 track 和当前稳定分数，q/Esc 退出
.\x64\Release\D455.exe --stable-contour-video --rgb-depth-warmup=30 --rgb-depth-anchor-step-px=4 --rgb-depth-anchor-neighborhood-px=2 --rgb-depth-anchor-max-neighbor-range-mm=35 --rgb-depth-anchor-max-raw-filtered-gap-mm=60 --rgb-depth-anchor-edge-dilate-px=3 --stable-contour-min-anchors=16 --stable-contour-top=12 --stable-contour-save=stable_contours_last.png
```

说明：

- `--rgb-depth-onnx` 指向 MiDaS v2.1 small ONNX 模型；不传模型时会退回内置 RGB 启发式基线，误差通常明显更大。
- MiDaS 输出是相对逆深度，不直接给毫米级绝对深度；程序用第一帧 D455 参考深度做一次尺度标定，然后用固定标定评估后续帧。
- 输出指标包括逐帧和 20 帧汇总的 MAE、RMSE、绝对误差分位数、毫米阈值命中率和相对误差命中率。
- `--rgb-depth-anchor-correction` 会改为每帧只用可信基准点做尺度矫正；红外/D455 深度在这个模式中只作为稀疏基准点和留出验证点，不作为密集深度输出。
- `--stable-contour-test` 不运行 RGB 单目深度模型；它用可信稳定点筛选并校准现有候选轮廓，再输出每个稳定 track 的出现帧数、连续 IoU、中心漂移、面积变化、深度变化和稳定性评分。
- `--stable-contour-save` 会保存最后一帧可视化；需要按固定帧数显示时加 `--stable-contour-show`，需要连续视频显示时用 `--stable-contour-video`。

无窗口探测 RealSense 设备：

```powershell
.\x64\Debug\D455.exe --probe-only
```

按 `q` 或 `Esc` 退出。
