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

实时模式默认直接用稳定轮廓掩码拼接彩图，以降低相机移动时的显示滞后。需要诊断 RGB 边缘能否补齐稳定轮廓时，可以显式启用严格 RGB 轮廓补齐：只在稳定轮廓的小邻域里用彩图生成候选补齐轮廓；如果候选和原稳定轮廓的 IoU、面积变化、中心偏移都在阈值内，且候选不会进入其他稳定轮廓的隔离带，左下/右下两格才采用彩图补齐后的轮廓，否则继续使用原稳定轮廓。未知背景不会被强行分配给任何前景轮廓，也不会为了补齐而把两个轮廓融合：

```powershell
.\x64\Release\D455.exe --color-contour-completion
```

主窗口模式默认不录制，优先保持实时窗口低延迟。需要保存当前 2x2 同步画面时，显式传入 `--record-video` 或 `--record-video=...`；默认路径为 `D:\D455\recordings\d455_record_时间戳.avi`：

```powershell
.\x64\Release\D455.exe --record-video
.\x64\Release\D455.exe --record-video=recordings\d455_stable_contours.avi --record-fps=30
```

性能分析时可以关闭窗口显示和验收录制，只保留 CSV 指标；CSV 会追加各阶段耗时字段，方便区分算法、渲染、显示和录制开销：

```powershell
.\x64\Release\D455.exe --acceptance-baseline --acceptance-no-record --no-display --max-frames=600
```

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

姿态读取是可选旁路能力，默认不改变 RGBD 分割和稳定观察单元。打开 `--pose-read` 后，程序会在设备支持时同时启用 accel/gyro motion stream：roll/pitch 由加速度重力方向估计，`yaw_rel` 由陀螺仪短时积分得到，只能作为本次启动后的相对航向参考，会随时间漂移。默认会把姿态摘要叠加到原始彩图窗口和录制视频；验收 CSV 只在启用姿态时追加 `pose_*` 字段：

```powershell
.\x64\Release\D455.exe --pose-read
.\x64\Release\D455.exe --acceptance-baseline --pose-read --pose-log --pose-log-every-n=30 --max-frames=300
.\x64\Release\D455.exe --pose-read --no-pose-overlay --pose-smooth-percent=10
```

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
