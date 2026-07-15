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

## 外围轮廓内部保留与真实穿孔过滤

2026-07-14 新增默认关闭的 `--existence-contour-hole-filter` 探针。稳定轨迹给出外围轮廓后，系统先把轮廓内部全部记为该存在的像素归属；深度为 `0/65535`、黑色材质或双目视差失败都不是穿孔证据，因此这些内部像素继续保留。只有内部区域具有有效深度、明显位于存在表面之后、与轮廓外圈背景深度和颜色相容，并按 `track_id` 连续多帧确认时，才从该存在的 ownership mask 中扣除并交给后续视觉/背景路径接管。

该功能不补写内部逐像素深度，也不把历史值伪装成本帧测量；簇级距离仍只来自已有有效深度支撑，cluster metadata 用 `depth_evidence_pixel_count` 将深度证据数量与完整轮廓归属像素数分开。主要参数为 `--existence-hole-min-depth-gap-mm`、`--existence-hole-background-depth-match-mm`、`--existence-hole-background-color-distance`、`--existence-hole-min-area-px`、`--existence-hole-max-area-percent`、`--existence-hole-confirm-frames` 和 `--existence-hole-memory-max-age-frames`。profile、converter、score 和 leaderboard 已贯通保留无深度像素、候选/确认穿孔、记忆像素、保守拒绝和处理耗时指标。`candidate_0049` 仅用于证明此机制，不修复上游已经分裂的外围轮廓，也不替换现有 best；完整边界和验证命令见 `docs/EXISTENCE_CONTOUR_HOLE_FILTER.md`。

机制探针中，真实背景缺口连续确认并扣除 8783px；只把内部深度设为无效时则保留 8787px，候选和确认穿孔均为 0。三组真实 120 帧 replay 的 `candidate_0030/0049` 共 6/6 通过硬门槛，coverage=100%、unknown=0、contour-lost/merge/split=0；0049 最终合并后的确认穿孔 p95 为 0-944px，同时保留无深度像素 p50 为 21801-29867px。过滤器自身 p95 为 12.7-22.0ms，最终重跑没有超过 100ms 的帧，但 0049 在三个 case 上仍比 0030 慢约 12-22ms、总分也更低，因此保持 probe-only，0030 继续作为 motion 性能基线。

## 视觉注意力与差异驱动处理方向

后续性能主线不再只优化全画面完整算法耗时，而是建立“最快全画面变化扫描 + 场景缓存复用 + 中心/单存在关注 + 多关注区域线程池并发 + 分片全局刷新”。全画面像素账本继续保留，未关注区域不得自动视为背景；缓存深度必须标记历史来源。详细数据结构、中文方法、线程边界、双目裁剪边距、性能模型和验收门槛见 `资料/视觉注意力缓存差异扫描与并发ROI处理方案_v0.1.md`。

2026-07-10 已实现第一版默认关闭的 attention 调度闭环。`--attention-difference-scan` 会每帧生成参数化低分辨率灰度签名，以相位相关结果做有限二维位移门禁，计算变化比例并复用未触发重算的彩图轮廓缓存；局部变化沿用 motion 连通域生成多个 ROI。多个 ROI 由固定线程池只读处理，工作线程各自返回结果，调用线程按 ROI 输入顺序确定性合并；现有异步刷新结果还增加缓存版本和最大年龄校验，过期结果不得写回。缓存缺失、对齐不可靠、变化面积超限和既有 interval/unknown/far-loss 事件仍退回原全帧路径，不会把本帧未重算区域改写为背景。

主要参数是 `--attention-scan-width-px`、`--attention-diff-threshold`、`--attention-min-dirty-percent`、`--attention-max-dirty-percent`、`--attention-roi-workers`、`--attention-max-cache-shift-px` 和 `--attention-max-result-age-frames`；`attention-roi-workers=0` 自动使用 `min(逻辑核心数-1, 4)`。profile、converter、score 和 leaderboard 已贯通 `attention_scan_ms / dirty_percent / dirty_roi_count / cache_reused / full_refresh / worker_task_count / worker_queue_ms / worker_ms / longest_ms / merge_ms / apply_ms / stale_result_count`。实现和固定回放结果见 `docs/ATTENTION_DIFFERENCE_SCAN.md`。

`candidate_0048` 是机制探针，不是新的全局 best。最终 120 帧固定回放中，四个 case 均保持 coverage=100%、unknown=0，且 contour-lost/merge/split 均为 0；真实遮挡 holdout 的 p95 为 57.070ms，局部运动为 56.000ms。局部运动的前一次同配置运行曾得到 52.170ms，说明当前线程池时序仍有运行间波动，不能宣称稳定提速。slow-pan p95 为 70.194ms、max 为 108.834ms，仍差于 `candidate_0030` 的 63.218ms/90.872ms，因此 `candidate_0030` 继续保留 slow-pan bucket best。当前只把彩图轮廓提取缩小并并发，深度锚点复核、完整像素来源账本、IMU/三维缓存重投影、下游深度聚类和最终归属尚未局部化。

## 双目静态稳定性子项目

`StaticStabilityProbe/` 是独立于主分割程序的第一层稳定性测试项目，用于回答“同一原始帧重复处理是否确定”和“静态场景连续原始帧的 color/depth/左右IR/视差差异有多大”。它不复用 `D455.cpp` 的 tracker、轮廓缓存、时序滤波和最终归簇状态，每帧都独立执行相同处理；操作协议和指标定义见 `StaticStabilityProbe/README.md`，工具链基线见 `资料/D455双目静态稳定性第一阶段测试报告_v0.1.md`，正式 A-B-A-B 交叉测试见 `资料/D455双目静态稳定性第二阶段交叉测试报告_v0.1.md`。

子项目支持三条路径：实时采集会保存真实内外参、传感器选项、逐流帧号/时间戳和未对齐四路原始帧；目录 replay 会生成逐帧差异 CSV；同帧 repeat 会用派生结果哈希检查软件确定性。`--lock-controls` 在预热后锁定当前曝光、增益和白平衡，并在结束或异常退出时恢复原设置；`compare_stability.py` 用于对比自动控制和锁定控制的 p50/p95/max 差异。固定 5 灰度阈值只保留为原始亮度口径，正式判断同时使用 IR 每帧均值归一化强度差、归一化梯度差、均值相对漂移和整段变异系数，避免不同亮度工作点造成假结论。

2026-07-10 已完成 Auto1→Locked1→Auto2→Locked2 各600帧交叉测试。两次 Auto 的左右IR均值约为53/60，两次 Locked 约为162/161，说明亮度工作点跃迁随控制模式重复出现；归一化后，两次 Locked 的 IR 均值 CV 约0.12%，低于 Auto 的0.47%/0.64%，锁定控制也降低了本场景的彩图和深度波动。后续同步专项测试确认：关闭 Stereo AE 后沿用读回曝光33000会使 depth/左右IR从30Hz降为15Hz，RGB仍为30Hz；RGB-only 锁定不会降帧。显式使用 `--stereo-exposure=30000` 的两次短测均恢复四路30Hz和100%同步接受，并保持较低相邻帧与深度波动，但固定参考帧的IR结构漂移尚未稳定复现，因此30000目前是“30Hz手动曝光实验配置”，不是最终全局最优配置。

第三阶段使用曝光30000重新采集600帧，完成逐像素空间噪声、多帧收敛和孔洞连续长度测试，结果见 `资料/D455深度空间噪声与多帧收敛测试报告_v0.1.md`。有效深度现在明确排除 `0` 和毫米转换饱和值 `65535`。当前场景中，持续有效内部像素的简单平均将p95重复性误差从单帧约40.25mm降到3帧约24.42mm、5帧约20.30mm、32帧约13.09mm；但边缘跳变候选即使32帧处理后p95仍超过160mm，禁止直接平均。三帧历史保持可覆盖约75.17%的已恢复孔洞事件，只能维持归属连续性，不能升级为当前帧精确深度。

主分割链新增默认关闭的 `--binary-contour-stability` 诊断，用于测试静止场景中最终稳定 ownership 轮廓的连续帧形状和位置差异。程序按稳定 `observation_id` 分离轮廓，每个轮廓先裁到最小包围框，再放到最小 8 倍数宽高的矩形画布中央；每个 `8x8` 块无损压成一个 little-endian `uint64_t`，同时保存原图尺寸、原始包围框位置和 contour ID。在线 CSV 和离线脚本只在同一 ID 内计算相邻帧/首参考帧的 Hamming 一致率、前景 IoU、变化像素数、bbox 中心位移和位置相似度，并记录当前轮廓的画面面积占比、平均/范围深度及距画面中心的径向比例。形状判断使用居中前景 IoU，位置相似度使用 `100 * (1 - bbox中心位移 / 原图对角线)`；严格 99% 门禁只统计 `previous_gap_frames == 1` 的连续帧对，两项必须分别 `>99%`，不能互相补偿。格式、参数和复核命令见 `docs/BINARY_CONTOUR_STABILITY.md`。

首轮使用 `static_stability_locked_001` 前120帧、忽略前30帧完成正式回放：90个采样帧生成1059个压缩轮廓，C++与Python逐位复核通过，载荷相对居中8-bit二值图固定压缩为8:1。同一回放重复执行后1059/1059个文件SHA-256完全相同，证明固定输入下处理确定。严格 99% 目标复核共有1044个同ID连续帧对：轮廓IoU `>99%` 为13.1226%，位置相似度 `>99%` 为85.9195%，两项同时通过为13.1226%，因此当前目标明确失败，主要瓶颈是形状而不是位置。画面越靠边时位置通过率从中心桶95.13%降至最外桶75.57%；面积和距离关系受同一track重复采样及小碎片双峰分布影响，只能作为本场景描述，不能外推为因果规律。完整目标、分桶、相关系数和限制见 `资料/D455静止存在轮廓位置99目标与影响关系测试报告_v0.2.md`。

2026-07-15 新增离线 `scripts/analyze_hierarchical_filled_contour_stability.py`，用于验证“闭合外轮廓内部全部填充 + 正方形分级压缩”口径。脚本读取现有 `.b8x8`，先把所有未连接外部的内部黑洞填为前景，再把轮廓居中到能容纳全部样本的2次幂正方形；本轮主轮廓为512x512，按2x2面积池化依次生成256、128、64、32、16、8级。每级同时比较50%占用阈值后的按位二值IoU和0-255占用相似度，并统计精确重复值。两段锁定静态回放共1140帧中，512/256/128级仍各有1140种二值值，64级首次重复，32级为320种，16级为32种，8级仅2种且1138个重复帧；8级相邻精确重复率99.4728%，但0-255占用值仍为1140种，说明二值粗压缩获得稳定离散值的同时丢失了持续面积差异。该能力只用于外剪影稳定性/快速扫描实验，不得据此删除真实背景穿孔；完整协议和运行命令见 `docs/HIERARCHICAL_FILLED_CONTOUR_STABILITY.md`。

同日新增 `scripts/analyze_all_hierarchical_filled_contours.py`，把两段静态回放中的全部轮廓纳入同一分级协议。脚本按“两段长期持续 / 单段长期持续 / 中短期或接替轮廓”分桶，只比较段内同 ID 且 `previous_gap_frames=1` 的帧对，并按每个轮廓自己的基础正方形处理；重复值以 `(contour_id, digest)` 计数，避免把不同存在的相同粗位图合并。53个 ID、72个 track、13139条轮廓帧记录的测试中，全部轮廓8级相邻精确重复率为67.2381%，两段长期持续组为76.0545%，明显低于主轮廓 ID 1 的99.4728%；ID 5和9即使长期持续也只有38.6643%和34.7979%。因此8级只能用于快速候选扫描，不能作为所有静态轮廓99%稳定的通用事实；需要用16/32级或原始轮廓复核。逐 track 输出同时记录面积、距离和画面中心位置，本次混合场景未显示可推广的单变量单调规律；建立自适应阈值前需要控制变量数据。完整结果和边界仍见 `docs/HIERARCHICAL_FILLED_CONTOUR_STABILITY.md`。

同日新增 `scripts/analyze_compressed_contour_discrimination.py`，验证压缩轮廓能否区分不同存在候选。完整8/16/32级按位特征包保存在本地 `analysis_runs/compressed_contour_discrimination_001/compressed_binary_features.npz`，可用 `--reuse-feature-cache` 直接重算；仓库仅保存 `docs/codex_analysis/compressed_contour_discrimination_001/` 下的聚合证据。两段中8级无跨 ID 歧义帧仅63.8456%/65.4423%，单值最多被8/15个 ID 共用；16级无歧义帧为91.8244%/93.0823%；32级在13139条样本中没有跨 ID 精确碰撞。对两段均长期持续的8个 ID，32级双向跨段唯一识别率为99.6930%/99.1886%，但包含接替 track 的段内识别仍只有84.2839%/88.8778%。因此当前默认解释为：8级负责快速扫描，16级负责粗筛，32级提供长期存在的形状身份候选；最终存在归属仍必须结合位置、尺寸、深度、运动和历史，不能只凭压缩轮廓裁决。协议、数据边界和复现命令见 `docs/COMPRESSED_CONTOUR_DISCRIMINATION.md`。

上述全轮廓稳定性、跨 ID 碰撞、时间切分识别、跨回放识别、门禁判定和后续真值/未知拒绝测试路线，已汇总为 `资料/D455全轮廓分级压缩与存在区分测试报告_v0.1.md`。

随后新增 `scripts/analyze_32_contour_similarity.py`，直接比较32x32轮廓的同 ID 正样本和同帧不同 ID 负样本，并执行0.1%步进阈值扫描。两段长期 track 合并后，正样本 IoU p05/p50 为63.6408%/91.3793%，负样本 p50/p95 为14.5754%/62.6506%，AUC为0.989862；最佳平衡阈值约64.7%，TPR 94.6983%、FPR 3.4942%，将 FPR 压到1%以内需约71.5%，此时TPR为90.4218%。由于同 ID 突变和 `9/10`、`8/9` 等不同 ID 高相似样本仍有重叠，当前只把 `>=72%` 视为强形状候选、`65%-72%` 视为待联合复核区，低于65%也不能单独断开身份。完整结果追加在 `docs/COMPRESSED_CONTOUR_DISCRIMINATION.md` 和正式测试报告中。

```powershell
msbuild .\StaticStabilityProbe\StaticStabilityProbe.vcxproj /p:Configuration=Release /p:Platform=x64 /m
.\x64\Release\StaticStabilityProbe.exe --replay-dir=datasets\near_single_object --repeat-frame=0 --repeat-count=100 --out-dir=analysis_runs\static_stability_repeat_001
.\x64\Release\StaticStabilityProbe.exe --capture-dir=datasets\static_stability_auto_001 --out-dir=analysis_runs\static_stability_auto_001 --frames=600 --warmup-frames=300
.\x64\Release\StaticStabilityProbe.exe --capture-dir=datasets\static_stability_locked_001 --out-dir=analysis_runs\static_stability_locked_001 --frames=600 --warmup-frames=300 --settle-frames=30 --lock-controls
```

## 云端分析协议

P0-P6 已建立 `eval/visual_evolution/` 记录契约、历史样例、校验器、需求候选生成器、适用方法查询、单目标任务筹办、任务级评估、显式晋级、回退和适用方法选择脚本；`run_batch.py` 与 converter 已能传递需求/任务/方法元数据而不改变 D455.exe 参数。`compare_runs.py` 已执行 regression gate，`evaluate_visual_task.py` 现在会按 fixed/holdout/shadow 的同场景基线分别比较，并要求同一候选跨分片通过。独立 `datasets/hand_occlusion_reappear_holdout` 已于 2026-07-10 采集并完成 0030/0034/0035 评分；任务级裁决为 `fail`，没有方法获准晋级，P7 实时影子继续阻塞。

仓库现在固定使用 `analysis_runs/<run_id>/` 作为云端分析包目录，协议说明见 `docs/ANALYSIS_PROTOCOL.md`，评判指标见 `docs/EVALUATION_FEATURES.md`，特征采集性能策略见 `docs/FEATURE_OPTIMIZATION.md`，运行索引见 `docs/RUN_INDEX.md`。需求、特征值、中文方法函数和实现流程的资料说明见 `资料/需求特征值方法流程说明.md`；需求如何由真实差距产生、任务如何选择方法、方法如何学习晋级的设计见 `资料/视觉方法需求任务学习晋级闭环详细设计.md`，配套流程图见 `资料/视觉方法需求任务学习晋级闭环流程图.md`，分阶段实施计划见 `资料/视觉能力进化闭环实现计划.md`。这些材料目前是设计和计划规格，不代表 D455 已经实现自动需求生成、任务调度或方法自动晋级。每次同步一次运行目录即可按 `run_id / branch / commit / config / metrics / sample_frames` 复查问题；不要把分析数据散落到 `recordings/` 里再靠聊天记录解释。最小六件套为：

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

缓存刷新现在会进入可度量输出：D455 在 `profile.csv` 追加 `color_contour_refreshed`、`color_contour_cache_reused`、`color_contour_refresh_motion`、`color_contour_refresh_unknown_spike`、`color_contour_refresh_far_loss`、`color_contour_motion_delta_percent`、ROI 刷新/候选面积/拒绝原因、ROI motion mask 像素、motion bbox、padding 后 bbox、ROI 面积上限、ROI 内刷新 region 数、ROI stereo reuse/failed 数和 preserved stereo 数；converter 会把它们写入 `frame_metrics.csv`，并在刷新帧写 `events.csv` 的 `color_contour_refresh` 事件。`run_score.json` 也汇总 refresh/cache/reuse/ROI 计数，后续 motion gate 不再只靠 p95 和 far_score 间接推断触发策略是否有效。

`candidate_0016` / `candidate_0017` 是 `candidate_0015` 的保守变体：0016 使用 `--color-contour-frame-interval=180`、motion 阈值 35%、关闭 unknown-spike refresh，仅保留 far-loss 恢复；0017 保持 120 帧间隔，但把 motion 阈值提高到 40%、unknown 阈值提高到 12%。它们用于 motion-sensitive replay gate 的下一轮对照，不应在 `slow_pan_far_object` 和 `hand_occlusion_reappear` 评分前提升为 best。

`replay_static_refresh_variants_001` 已跑完 0016/0017 在 `near_single_object`、`far_cabinet`、`depth_hole_black_object` 上的静态 smoke，6/6 pass，且 `candidate_0017` 分数和 p95 最好；但该结果中 scored frames 的 `color_contour_refresh_count=0`、`color_contour_cache_reuse_count=3`，只说明静态缓存复用成本可控，不说明 motion/unknown/far-loss 触发在运动场景有效。最小摘要在 `leaderboards/replay_static_refresh_variants_001/`。

`configs/best/best_replay_static_refresh.json` 将 `candidate_0017` 固化为当前静态 refresh bucket 赢家；它不替换 `best_replay_static_far_distance.json`，后者仍保留为来自 `candidate_0014` 的低频粗距基础桶。`best_replay_static_refresh` 的适用范围限定为 deterministic/static replay 和低运动场景，不能代表 motion best。`scripts/select_winners.py` 的 leaderboard 现在直接输出 `color_refresh_count`、`color_refresh_motion_count`、`color_refresh_unknown_spike_count`、`color_refresh_far_loss_count`、`color_cache_reuse_count` 和 `color_stereo_reuse_p50`，后续 `replay_motion_gate_001` 不需要再逐个翻 `run_score.json` 才能判断刷新触发是否真的发生。

`slow_pan_far_object` 的第一段真实慢速平移 replay 显示，0015/0017 能触发 `color_contour_refresh_motion`，但刷新帧的 `gray_prepare_ms` 达到约 380-416ms，主要开销来自对全部彩图轮廓执行双目粗距匹配。为此新增 `--stereo-contour-max-regions-per-frame=N`，默认 32 保持旧行为；`candidate_0018` 先设为 8，但按面积优先仍会命中最大、最贵的区域。随后新增 `--stereo-contour-max-roi-area-percent=N`，默认 100 保持旧行为；`candidate_0019` 设为 12%，并把每帧实际 stereo match 尝试限制为 16 次，目标是跳过巨大平面/背景模板，让 motion refresh 不再产生 400ms 级别卡顿。风险是大区域可能暂时只有 2D 归属、没有双目粗距。

ROI/数量筛选只能小幅降低刷新帧耗时，不能根治同步 `matchTemplate` 卡顿。`--stereo-contour-reuse-cached-on-refresh` 改为在已有缓存后刷新彩图轮廓、但把上一批 stereo 粗距按 bbox IoU/中心距离转移到新轮廓；`candidate_0020` 使用该策略，目标是让 motion refresh 更新轮廓归属而不在同一帧重跑双目匹配。这个策略的距离会有短时滞后，后续更完整的方案应把 stereo 粗距做成分帧或异步后台刷新。

如果复用 stereo 后刷新帧仍超时，瓶颈就转移到彩图分割本身，尤其是 `pyrMeanShiftFiltering`。`candidate_0021` 在 0020 基础上把 `--color-segmentation-mean-shift-spatial=0` 和 `--color-segmentation-mean-shift-color=0`，用于验证轻量彩图轮廓刷新是否足够通过慢速平移 replay；风险是彩色连通区域会更碎，需要靠 leaderboard 的 split/merge 和后续可视化复核约束。

`candidate_0022` 继续压低轻量彩图刷新成本：颜色 bins 从 6 降到 4，最多保留 16 个彩图区域，最小区域提高到 1200 px。它是 motion gate 的性能探针，不代表最终细粒度分割策略；如果它通过但画面颗粒度明显下降，下一步应改为 ROI 局部刷新或分帧刷新，而不是继续全局降质量。

`candidate_0023` 是更激进的全局刷新下界探针：bins=3、最多 8 个彩图区域、min area=2000。它只用于判断全局彩图刷新是否存在可过 100ms p95 的参数空间；即使通过，也不能直接当最终质量方案。

`candidate_0024` 进一步把全局刷新压到最多 4 个大彩图区域、min area=3000，用来确认硬门槛的理论下界。它若通过，只能说明“极粗全局刷新能跑快”，不能说明分割质量达标。

真实慢速平移数据 `datasets/slow_pan_far_object` 已完成本地采集，包含 180 帧 color/depth16/left IR/right IR，并已按 case contract finalized/reviewed；检查视频为本地 `recordings/slow_pan_far_object_color_20260706_1549.mp4`，数据集和视频不进入 Git。`replay_motion_slow_pan_001` 跑了 0013、0014、0015、0017、0018-0031。评分器现在用完整 `profile.csv` 计算性能 p95，用导出帧 `frame_metrics.csv` 计算分割质量，并在 leaderboard 保留导出采样 p95、最大帧耗时、超 100ms 帧数作为顿挫风险指标。`candidate_0030` 是当前 slow-pan 阶段赢家：p95 63.2137ms、max 91.658ms、超 100ms 帧数为 0、far_score 10.0；它基于 0028 的粗粒度 async+cooldown 路线，增加 motion ROI 刷新开关和 ROI 诊断字段。该结果已进入 `configs/best/best_replay_motion_slow_pan.json`，但它仍是 coarse slow-pan bucket best，不是 overall best；本次 slow-pan 中 ROI 候选面积为 307200px 且 4 次因过大被拒绝，所以这不证明 ROI 局部刷新已产生收益。

`local_motion_roi_probe` 是独立于 slow-pan 的 ROI 机制验证 case：`scripts/generate_local_motion_roi_probe.py` 会从已有静态 D455 replay 的一帧自动生成局部移动目标，输出 color/depth16/left IR/right IR 和 reviewed manifest；它是 synthetic probe，只验证 ROI 刷新管线，不代表真实场景分割质量。`candidate_0032` 基于 0030，但把 `--color-contour-refresh-motion-delta-percent` 降到 3%，用于触发小目标局部运动；`candidate_0033` 保持同一阈值，只用于验证 ROI stereo-retention 代码修正。`replay_local_motion_roi_probe_001` 的最新结果显示：0030 仍不触发 ROI；0032/0033 均触发 4 次局部 ROI 刷新，ROI 面积 p95 为 76800px，没有 `rejected_large`，`far_stereo_failed_event_count` 已从旧结果的 6 降到 0。修正点是 `mergeColorContourRoiRefresh()` 在 ROI 内新 region 没有匹配 stereo evidence 时保留重叠的 cached stereo-bearing region。这个 case 只通过 G1：ROI refresh 不破坏既有 far stereo evidence；它没有通过 G2，因为 ROI refreshed region 自身仍有 `roi_stereo_failed_count=6`、`roi_stereo_reuse_count=0`，只是通过 `roi_preserved_stereo_count=3` 保住远场粗距证据。

G1/G2/G3 已拆成独立门禁：G1=`roi_stereo_g1_preservation_pass`，要求 ROI 触发、没有 `far_stereo_failed`，并存在 preserved/reused/built stereo evidence；G2=`roi_stereo_g2_rebuild_pass`，要求 ROI 新 region 没有 stereo failed，并且 `roi_stereo_reuse_count > 0` 或 `roi_stereo_built_count > 0`。为验证 G2，`generate_local_motion_roi_probe.py` 新增 `--right-ir-shift-x`，可让 synthetic 目标在右 IR 中产生正视差；`local_motion_roi_stereo_probe` 使用 `--right-ir-shift-x=-30` 和 1200mm 深度生成。`candidate_0034` 是 G2 direct-build 探针：关闭 async/cache reuse，让 ROI 同步路径直接跑 stereo；它证明 direct build 部分有效（`roi_stereo_built_count=4`），但 revealed-background 碎片仍造成 failed=4。`candidate_0035` 在 0034 基础上开启 `--color-contour-refresh-roi-drop-stereo-failed`，只保留 ROI refresh 后带 stereo evidence 的局部 region，`roi_stereo_built_count=4`、`roi_stereo_failed_count=0`，使 direct-build 专项门禁通过。G3 是视觉复核门禁：`scripts/generate_roi_drop_review.py` 会对比 0034/0035，同时输出 runtime ROI drop 计数和 final-segmentation 中缺失的 baseline region，并把 stereo-bearing 缺失 region 标为高优先级 target-risk。`replay_local_motion_roi_stereo_probe_001` 的 synthetic G3 review 显示 0035 丢弃的是同一个墙面背景碎片，不是 synthetic 目标；但真实遮挡/重现场景未验证，所以 0035 仍是 probe，不替代 `candidate_0030` 的 slow-pan best。

`scripts/generate_hand_occlusion_reappear_proxy.py` 是 G3 的自动遮挡/重现 proxy 生成器：它从已 reviewed 的静态 D455 replay 帧中注入带 stereo evidence 的目标和一个近处遮挡块，生成 color/depth16/left IR/right IR，并同时写出可直接传给 `run_batch.py` 的临时 cases YAML。默认输出到 `analysis_runs/generated_cases/hand_occlusion_reappear_proxy`，避免把生成数据写入 `datasets/`；如需替代真实 case 路径，必须显式传 `--out-case=datasets\hand_occlusion_reappear` 并单独确认数据同步策略。`scripts/generate_roi_drop_review.py` 现在会给出 `g3_status=pass|warning|red`：`dropped_stereo_region_count > 0` 是 red；no-stereo drop、runtime drop 像素过大或 `contour_lost_event_count > 0` 是 warning；`missing_baseline_region_count=0`、`dropped_stereo_region_count=0`、`contour_lost_event_count=0` 且 runtime drop 未过阈值才是 pass。有 drop 记录时脚本会把最多 1 帧 source/baseline/filtered PNG 复制到 `g3_drop_review/sample_frames/` 作为云端最小视觉证据。`replay_hand_occlusion_reappear_proxy_001` 跑了 0034/0035：0035 `score=93.670`、`roi_stereo_built_count=10`、`roi_stereo_failed_count=0`、runtime drop=1/9103px，final-segmentation 对比中 `missing_baseline_region_count=0`、`dropped_stereo_region_count=0`，因此 proxy G3 为 pass。`replay_local_motion_roi_stereo_probe_001` 的 G3 为 warning，因为 drop 的是 no-stereo fragment 且 runtime drop 像素超过 20000，但没有 stereo-bearing target drop。这些只说明 controlled probe/proxy 下没有发现目标丢失，不代表真实手部遮挡已经通过。

真实 `hand_occlusion_reappear` 已完成第一段 120 帧本地采集，并通过 reviewed manifest 校验；数据集仍位于本机 `datasets/hand_occlusion_reappear`，不进入 Git。`replay_hand_occlusion_reappear_real_001` 跑了 `candidate_0030`、`candidate_0034`、`candidate_0035`：0030 以 `score=93.584`、p95=69.4254ms、max=88.561ms 保持真实遮挡场景最佳；0034/0035 本轮都通过 hard gate，但仍分别有 4/1 帧超过 100ms。0035 的 G3 red-line 为 pass：`runtime_roi_stereo_dropped_count=0`、`missing_baseline_region_count=0`、`dropped_stereo_region_count=0`、`contour_lost_event_count=0`。新增 ROI 源诊断显示，0035 的 motion mask 本身不大（p95=1668.3px），但稀疏 motion 像素被合并成单个大 bbox（p95=296832px），padding 后 ROI p95=303744px，超过 `color_contour_refresh_roi_max_pixels_p50=107520px`，因此 7 次被 `rejected_large`。这说明当前瓶颈是“单 union bbox 把稀疏运动包成近整帧”，不是 drop 安全性；0035 仍保持 probe-only，下一步应做多连通域/分块 ROI 或 ROI 源分离。

独立 `hand_occlusion_reappear_holdout` 已完成第二段 120 帧同步 color/depth16/left-IR/right-IR 采集和 reviewed manifest 校验，原始数据仍只保存在本机。`replay_hand_occlusion_reappear_holdout_001` 中 0030/0034/0035 都通过单运行 hard gate，但 0030 明显领先：score=94.163、p95=65.566ms、near/far=20/10；0034/0035 分别只有 score=66.881/66.483、near/far=6/0，且有 1/2 帧超过 100ms。任务级评估同时确认 0034/0035 在历史固定回放上相对 0030 的 regression gate 失败，因此 `task-hand-occlusion-local-refresh-001` 收口为 `failed`，需求继续保持 `confirmed`，不生成 promotion，也不进入 P7 影子运行。

`--color-contour-refresh-roi-component-clamp` 是第一版稀疏 motion 连通域裁剪：默认关闭；开启后只在 union ROI 超过面积上限时按 motion 连通域面积排序，选择能落入上限的局部 ROI。`candidate_0036` 在 0035 上启用该策略并保留 drop policy；`candidate_0037` 启用同样裁剪但关闭 drop policy，用来分离 ROI 局部性和 fragment drop。`replay_hand_occlusion_roi_component_clamp_002` 显示 component clamp 的局部性确实生效：0036/0037 的 ROI candidate p95 从 303744px 降到 72576px，`rejected_large_count` 从 7 降到 0。但两者对 0035 全帧基线的 G3 仍为 red，分别缺失 4/3 个 stereo-bearing baseline region；0036 还记录 runtime ROI drop 6 次/48285px。因此 0036/0037 都是失败探针，不晋级。结论是：单 ROI component clamp 可以定位局部运动，但会跳过其它 motion/stereo-bearing 区域；下一步应实现真正多 ROI/分片刷新，或把未选组件转为保守 cache-preserve，而不是继续只选一个局部 bbox。

`--color-contour-refresh-roi-multi-component` 是第二版稀疏 motion ROI 探针：默认关闭；开启后 component clamp 不再把多个运动连通域合并成一个 bbox，而是按 padded component ROI 列表刷新，并用 `--color-contour-refresh-max-component-rois=N` 与 `--color-contour-refresh-max-roi-area-percent=N` 控制总预算。`candidate_0038` 使用 8 个 ROI / 35% 面积预算，不 drop stereo-failed fragment；`candidate_0039` 加回 drop policy；两者在真实 `hand_occlusion_reappear` 上仍为 G3 red，因为仍会缺失 stereo-bearing baseline region。`candidate_0040` 把预算放宽到 32 个 ROI / 70%，`replay_hand_occlusion_roi_multi_component_002` 中 G3 从 red 降为 warning：`dropped_stereo_region_count=0`、`runtime_roi_stereo_dropped_count=0`，但仍有 3 个 no-stereo baseline fragment 缺失，p95=73.4076ms、max=110.43ms、>100ms=1。它只证明 0038 的红线主要来自 ROI 预算过紧，是 upper-bound probe；当前真实遮挡/重现场景仍以 `candidate_0030` 为 motion bucket best。下一步应在 35%-70% 之间调预算、做 source-aware ROI 或分片刷新，不能把 0040 当作 ROI production 候选。

G3 review 已升级为优先使用 `final_segmentation_frame_*_ids.png` 做像素级 label overlap，只有 ids 不可用时才回退到 bbox IoU/中心距离；同一 baseline region 如果在 filtered segmentation 中仍有足够像素归属，或几何上存在对应新 region，就不再误判为 drop。`replay_hand_occlusion_roi_area_threshold_001` 对 65%/67%/68%/70% ROI 面积预算复测后显示：`candidate_0040`（70%）在真实 hand-occlusion 上为 G3 pass；`candidate_0044/0045/0046`（65%-68%）为 G3 warning，仅缺 no-stereo fragment，没有 stereo-bearing drop，其中 `candidate_0046` p95=70.744ms、max=94.391ms、ROI p95=182016px。`replay_slow_pan_roi_budget_check_001` 进一步确认这些低阈值 ROI 探针不能替代 `candidate_0030`：0040/0041 在 slow-pan 上 near/far 分下降到 16/6，p95 约 98-99ms，而 0030 仍保持 score=93.390、p95=70.717ms。当前结论是：0040/0046 只作为 hand-occlusion ROI 预算证据，motion bucket best 仍是 `candidate_0030`；生产方向应做 source-aware trigger，避免把相机平移也按局部 ROI 低阈值刷新处理。

`--color-contour-refresh-skip-rejected-motion-roi` 是第一版相机平移代理门控，默认关闭。它不直接依赖 IMU，而是利用 replay 中可复现的画面状态：当一次刷新只由 motion 触发、已有彩图轮廓 cache、ROI 为空或被判为过大，并且不是 startup / interval / unknown-spike / far-loss 时，直接复用 cache，避免 `extractColorContourRegionsInRois(..., empty)` 退回全帧刷新。profile/leaderboard 新增 `color_contour_refresh_roi_rejected_refresh_skipped_count` 记录该门控触发次数。`candidate_0047` 基于 0041 开启此门控；`replay_slow_pan_roi_reject_skip_001` 显示它跳过 6 次 rejected motion ROI，把 0041 在 slow-pan 的 near/far 从 16/6 恢复到 20/10，score 从 81.272 提升到 92.387，但仍有 max=108.006ms、>100ms=1，低于 `candidate_0030` 的 score=92.903、max=97.3ms。`replay_hand_occlusion_roi_reject_skip_001` 显示真实 hand-occlusion 中 0047 没有触发 skip，保留 7 次局部 ROI 刷新，G3 相对 0035 为 warning 而非 red：没有 stereo-bearing drop 和 contour lost，但仍缺一个 no-stereo 背景 fragment。结论是：姿态/画面状态预判方向可行，当前实现只是 replay-compatible camera-pan proxy probe，不晋级 best；下一步可把 live IMU gyro/pose 和该 rejected-ROI 证据合并成真正的 camera-motion gate。

第一版多线程实现为 `--async-color-contour-refresh`：启动帧仍同步建立首个 cache；之后 motion/interval/unknown/far-loss 触发只在后台 worker 空闲时提交彩图轮廓刷新任务，主线程继续使用旧 cache；worker 完成后按 bbox IoU/中心距离转移 cached stereo 粗距并切换新 cache。`--async-color-contour-low-priority` 会在 Windows 下把 worker 线程降为 `THREAD_PRIORITY_BELOW_NORMAL`；0026 实测没有改善 slow-pan p95/尖峰，只保留为可控开关。`--color-contour-refresh-min-gap-frames=N` 会跳过过近的非 startup 刷新请求并复用 cache，profile/leaderboard 记录 `color_contour_refresh_cooldown_skipped_count`。`--color-contour-refresh-motion-roi` 会尝试只刷新 motion diff 对应区域，并用 `--color-contour-refresh-roi-padding-px` 与 `--color-contour-refresh-max-roi-area-percent` 控制 ROI；如果相机平移或手部遮挡导致稀疏 motion 点被单个 union bbox 包成近整帧，会退回全图刷新并记录 `color_contour_refresh_roi_rejected_large_count`、motion mask/bbox/padding/max-threshold 诊断。score 同时保留全帧 `color_contour_async_worker_ms_p95` 和只统计非零 worker 帧的 `color_contour_async_worker_ms_positive_p95`；0030 的 positive worker p95 为 21.9194ms。下一步 ROI 方向应优先做多 ROI/分片处理，而不是继续放宽单个大 ROI 的面积阈值。

评分器同步收紧了远场保留口径：`far_retention` 不再只看是否没有 `far_stereo_failed` 事件，而是先按 `approx_stereo_contour_pixels + image_only_contour_pixels + depth_hole_candidate_pixels` 的像素占比给基础分，再用 `stereo_matched_cluster_count` 给双目粗距加分。converter 也只在“存在彩图轮廓但完全没有有效 stereo 距离”时记录 `far_stereo_failed`，避免把部分轮廓未匹配误判成整帧远场粗距失败。这样自动优化不会因为关闭彩图/双目而虚假拿到远场满分。

确定性目录 replay 是自动优化进入真实评测的入口。目录格式为 `datasets/<case_id>/frames/000000_color.png`、`000000_depth16.png`、`000000_ir_left.png`、`000000_ir_right.png` 和 `case_manifest.json`；`depth16.png` 按 16-bit 毫米深度读取，右红外缺失时只影响双目轮廓粗距。D455 支持 `--replay-dir=datasets\<case_id>`，不需要连接相机；`scripts/run_batch.py --execute` 会按 `eval/cases.yaml` 的 `replay:` 字段依次执行 D455、converter 和 `score_run.py`。为避免启动帧污染 p95 和 unknown 指标，converter 和 run_batch 支持 `--ignore-first-n-frames=30`：

```powershell
python scripts\run_batch.py --candidates configs\candidates\round_001 --cases eval\cases.yaml --analysis-export-every-n=30 --ignore-first-n-frames=30 --execute
```

固定 case 的采集入口是 `--capture-replay-dir=datasets\<case_id>`，它会从真实 D455 保存 color、对齐后的毫米 depth16、左/右 IR 和 `case_manifest.json`。第一批不要采太长，建议每个 case 60 到 120 帧，先建立 `near_single_object`、`far_cabinet`、`depth_hole_black_object` 三个可复跑输入，再跑小型 Round 001：

```powershell
.\tools\Capture-ReplayCases.ps1 -Frames 120 -Warmup 30
```

生成局部 ROI 机制 probe 时不需要人工参与：

```powershell
python scripts\generate_local_motion_roi_probe.py --source-case datasets\near_single_object --out-case datasets\local_motion_roi_probe --frames 120 --force
python scripts\run_batch.py --candidates configs\candidates\round_001 --candidate-id candidate_0030 --candidate-id candidate_0032 --candidate-id candidate_0033 --cases eval\cases.yaml --case-id local_motion_roi_probe --max-frames-override=120 --analysis-export-every-n=15 --ignore-first-n-frames=15 --validate-replay --require-replay-ir --require-reviewed-manifest --execute
python scripts\generate_local_motion_roi_probe.py --source-case datasets\near_single_object --out-case datasets\local_motion_roi_stereo_probe --frames 120 --right-ir-shift-x=-30 --object-depth-mm=1200 --expect-roi-stereo-rebuild --force
python scripts\run_batch.py --candidates configs\candidates\round_001 --candidate-id candidate_0033 --candidate-id candidate_0034 --candidate-id candidate_0035 --cases eval\cases.yaml --case-id local_motion_roi_stereo_probe --max-frames-override=120 --analysis-export-every-n=15 --ignore-first-n-frames=15 --validate-replay --require-replay-ir --require-reviewed-manifest --execute
python scripts\generate_roi_drop_review.py --baseline-run analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034 --filtered-run analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035 --case-dir datasets\local_motion_roi_stereo_probe --out leaderboards\replay_local_motion_roi_stereo_probe_001\g3_drop_review
python scripts\generate_hand_occlusion_reappear_proxy.py --source-case datasets\near_single_object --out-case analysis_runs\generated_cases\hand_occlusion_reappear_proxy --frames 120 --force
python scripts\run_batch.py --candidates configs\candidates\round_001 --candidate-id candidate_0034 --candidate-id candidate_0035 --cases analysis_runs\generated_cases\hand_occlusion_reappear_proxy_cases.yaml --case-id hand_occlusion_reappear_proxy --max-frames-override=120 --analysis-export-every-n=15 --ignore-first-n-frames=15 --validate-replay --require-replay-ir --require-reviewed-manifest --execute
python scripts\generate_roi_drop_review.py --baseline-run analysis_runs\replay_hand_occlusion_reappear_proxy_001\hand_occlusion_reappear_proxy_candidate_0034 --filtered-run analysis_runs\replay_hand_occlusion_reappear_proxy_001\hand_occlusion_reappear_proxy_candidate_0035 --case-dir analysis_runs\generated_cases\hand_occlusion_reappear_proxy --out leaderboards\replay_hand_occlusion_reappear_proxy_001\g3_drop_review
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
