# 像素簇视觉外设子工程

`PixelClusterSensor` 是独立的 C++20 工程，已登记到根目录 `D455.slnx`。它不修改或替换 `D455.exe` 的分割、显示、参数和现有候选配置。

## 本轮目标与边界

将一份相机输入转换为可独立读回的像素簇观测包，供自我继续组织和判断存在。输出完整颜色、像素归属、轮廓、当前深度、深度状态和独立的颜色引导插值；不输出体素，不确认存在，不建立第二份世界事实仓。

本轮实现的是本机供包端及参考客户端。正式自我接收端的 DTO、材料所有者、接纳接口和启动接线没有在本工程中假定或实现。JSON 中的浮点观测材料也不是向正式特征服务新增浮点二次特征实例。

## 构建与运行

依赖与仓库原有子工程一致：Visual Studio v145、Windows SDK、`D:\vcpkg` 中的 RealSense、OpenCV；另使用已安装的 nlohmann/json 和系统 BCrypt SHA256。可用 MSBuild 属性覆盖 `VcpkgRoot`。库版本写入每份观察包。

```powershell
msbuild .\PixelClusterSensor\PixelClusterSensor.vcxproj /p:Configuration=Release /p:Platform=x64 /m
.\x64\Release\PixelClusterSensor.exe --list-devices
python .\PixelClusterSensor\client.py --camera --frames 1 --validate
```

`client.py` 负责通过 UTF-8 JSONL 管道管理子进程，结束时关闭输入并等待设备释放；不会留下后台服务。协议服务不读取控制台按键、不监听网络，也不执行任意 shell 指令。

```powershell
python .\PixelClusterSensor\client.py --replay PATH\sequence.json --frames 2 --validate
python .\PixelClusterSensor\validate_packet.py PATH\frame.json --output .codex_tmp\PixelClusterSensor\review
python .\PixelClusterSensor\test_cluster_protocol.py --output .codex_tmp\PixelClusterSensor\cluster_protocol_tests_new
python .\PixelClusterSensor\test_cluster_conversion.py --output .codex_tmp\PixelClusterSensor\cluster_conversion_tests_new
python .\PixelClusterSensor\test_cluster_tracker.py --output .codex_tmp\PixelClusterSensor\cluster_tracker_tests_new
python .\PixelClusterSensor\test_cluster_resync.py --output .codex_tmp\PixelClusterSensor\cluster_resync_tests_new
python .\PixelClusterSensor\test_cluster_control.py --output .codex_tmp\PixelClusterSensor\cluster_control_tests_new
python .\PixelClusterSensor\run_cluster_control_gate.py --replay PATH\sequence.json --frames 120 --minimum-cluster-pixels 1024 --retained-cluster-pixels 512 --confirmation-frames 5 --output .codex_tmp\PixelClusterSensor\cluster_control_gate_new
python .\PixelClusterSensor\test_run_cluster_control_gate.py --output .codex_tmp\PixelClusterSensor\cluster_control_gate_tests_new
python .\PixelClusterSensor\evaluate_cluster_scenario_capture.py --sequence PATH\sequence.json --scenario T3_local_motion --output .codex_tmp\PixelClusterSensor\t3_content_gate_new
python .\PixelClusterSensor\test_evaluate_cluster_scenario_capture.py --output .codex_tmp\PixelClusterSensor\scenario_content_tests_new
python .\PixelClusterSensor\render_cluster_scenario_review.py --sequence PATH\sequence.json --scenario T3_local_motion --output .codex_tmp\PixelClusterSensor\t3_visual_review_new
python .\PixelClusterSensor\test_render_cluster_scenario_review.py --output .codex_tmp\PixelClusterSensor\scenario_visual_review_tests_new
python .\PixelClusterSensor\capture_and_verify_cluster_scenario.py --scenario T3_local_motion --frames 120 --serial 215122256633 --output .codex_tmp\PixelClusterSensor\t3_real_capture_new
python .\PixelClusterSensor\capture_and_verify_cluster_scenario.py --scenario T7_dark_depth_hole --frames 120 --serial 215122256633 --output .codex_tmp\PixelClusterSensor\t7_real_capture_new
python .\PixelClusterSensor\test_capture_and_verify_cluster_scenario.py --output .codex_tmp\PixelClusterSensor\scenario_gate_tests_new
python .\PixelClusterSensor\test_cluster_stream.py --output .codex_tmp\PixelClusterSensor\cluster_stream_tests_new
python .\PixelClusterSensor\convert_cluster_observation.py PATH\frame.json --output .codex_tmp\PixelClusterSensor\cluster_packet_new
python .\PixelClusterSensor\cluster_tracker.py PATH\packet_1.json PATH\packet_2.json --output .codex_tmp\PixelClusterSensor\tracked_packets_new
python .\PixelClusterSensor\cluster_stream.py --replay PATH\sequence.json --frames 3 --output .codex_tmp\PixelClusterSensor\cluster_stream_new
python .\PixelClusterSensor\export_raw_sequence.py --camera --frames 120 --output .codex_tmp\PixelClusterSensor\raw_sequence_capture_new
python .\PixelClusterSensor\test_export_raw_sequence.py --output .codex_tmp\PixelClusterSensor\raw_sequence_export_tests_new
python .\PixelClusterSensor\evaluate_cluster_stability.py --runs RUN_A RUN_B RUN_C --output .codex_tmp\PixelClusterSensor\cluster_stability_new
python .\PixelClusterSensor\test_evaluate_cluster_stability.py --output .codex_tmp\PixelClusterSensor\cluster_stability_tests_new
python .\PixelClusterSensor\run_cluster_stability_matrix.py --replay PATH\sequence.json --frames 600 --repetitions 3 --max-missing-frames 3 --occlusion-confirmation-frames 2 --minimum-cluster-pixels 1024 --retained-cluster-pixels 512 --maximum-tentative-match-cost 200000 --confirmation-frames 5 --clustering-mode 深度主导 --output .codex_tmp\PixelClusterSensor\cluster_stability_matrix_new
python .\PixelClusterSensor\test_run_cluster_stability_matrix.py --output .codex_tmp\PixelClusterSensor\cluster_stability_matrix_tests_new
python .\PixelClusterSensor\capture_and_verify_static.py --camera --frames 600 --repetitions 3 --max-missing-frames 3 --occlusion-confirmation-frames 2 --minimum-cluster-pixels 1024 --retained-cluster-pixels 512 --maximum-tentative-match-cost 200000 --confirmation-frames 5 --output .codex_tmp\PixelClusterSensor\static_capture_gate_new
python .\PixelClusterSensor\test_capture_and_verify_static.py --output .codex_tmp\PixelClusterSensor\static_capture_gate_tests_new
python .\PixelClusterSensor\cluster_protocol.py PATH\packet.json
python .\PixelClusterSensor\test_protocol.py --output .codex_tmp\PixelClusterSensor\tests_new_run
python .\PixelClusterSensor\test_camera.py --continuous-frames 20 --output .codex_tmp\PixelClusterSensor\camera_new_run
msbuild .\PixelClusterSensor\tests\SourceTimingTests.vcxproj /p:Configuration=Release /p:Platform=x64 /m
.\x64\Release\SourceTimingTests.exe
```

客户端本身仅用 Python 标准库；独立读回和测试需要 NumPy、Pillow。测试输出目录必须不存在，以免覆盖旧证据。项目不下载库、不自动安装环境、不改写 `datasets/`、历史 `analysis_runs/` 或用户录制材料。

### 簇级供包 P0/P1/P2/P3 影子桥

`cluster_protocol.py` 是 `PCS.ClusterObservation/1` 的严格读回器和 `PCS.ContourChain8/1` 轮廓编解码器。它校验包头、簇记录、状态枚举、深度证据分账、精确深度越权、轮廓闭合/拓扑、材料 SHA256 和覆盖像素分区。`convert_cluster_observation.py` 是无状态 P1 桥接，只将已发布的 `PCS.Observation/1` 转为 `FullSnapshot + Scan`。`cluster_tracker.py` 是离线 P2 短期跟踪器，可产生相机候选号、增量包、遮挡事件和丢失墓碑，并可从全量+增量重建活跃候选。

真实观察中可能出现一、两个像素的分割碎片；它们不能编码为闭合 `ContourChain8`。桥接层会丢弃退化环；若某个源簇没有有效外环，则把其像素降级计入全局 `Unknown`，并在包指标写出 `退化轮廓环数` 与 `降级未知像素数`。`--minimum-cluster-pixels` 是新候选及未确认候选的逐帧准入阈值；小于阈值的簇降级为 `Unknown`。`--retained-cluster-pixels` 只允许已经确认的轨迹暂时降到较低像素数后继续参与关联，默认取新候选阈值的一半，不能帮助 `Tentative` 碎片累计确认帧。实际阈值、被过滤簇数和像素数写入包指标、逐帧 CSV 和矩阵配置快照。该策略降低供包粒度，不能当作画面覆盖或物理稳定性的改善。它不补画轮廓，也不把这类碎片伪装成可靠簇或距离证据。

`cluster_stream.py` 是 P3 Python 影子/评测桥：它通过既有 `Client` 启动唯一持有相机的 `PixelClusterSensor.exe`，逐帧转换并跟踪，将簇级包和 `run_manifest.json`、`packet_metrics.csv`、`events.csv` 写到新目录。首帧发布全量快照；无簇级变化的后续帧发布空 `Heartbeat`，不重复发送静态轮廓；新增、移动、遮挡、重现和丢失仍发布 `Delta`。默认要求候选连续出现5帧才从 `Tentative` 晋为 `Active`；未确认阶段除满足新候选像素阈值外，每次关联代价还必须不高于 `--maximum-tentative-match-cost=200000`，防止相邻小碎片靠高代价接力被错误确认。每条轨迹另保留一份可靠关联参考：只有低代价匹配才更新参考，高代价当前观测可以发布但不能逐帧拖走后续关联基准。未确认候选消失时发布 `Removed/Retired`；已确认候选默认连续缺失2包才发布 `Occluded`，第1包缺失只通过增量语义保留旧状态，不发布新测量、不刷新证据年龄；连续缺失3包发布 `Lost`。只有已对外发布遮挡后的返回才标记 `Reappeared`。这些阈值会影响动态发现和陈旧证据年龄，必须在动态集另行验收。它不拥有相机、不修改 `PCS.Observation/1`，也不是 C++ 实时接线或扫描/观察/跟踪调度实现。

`cluster_receiver.py` 是 P2 的严格影子接收器。它按 `会话标识 + 跟踪时期 + 输出序号 + 前置场景版本 + 依赖全量序号` 应用包；增量缺口、未见过的迟到包、基线不符、场景版本不符、同序号冲突内容和包标识复用都会停止拼接并请求全量。完全相同的重复包幂等返回，不改状态。`cluster_tracker.py` 可将下一帧强制发布为自包含 `FullSnapshot`，并把未超期遮挡候选的最后轮廓重打包为历史证据，当前深度计数归零，不把旧测量冒充当前值。T9 合成故障注入已覆盖丢增量、重复、乱序、时期切换、进程重启和直接状态对照；这不证明 USB 物理断连恢复或正式消费者接线。

`cluster_control.py` 是 P3 的 Python 影子控制与调度层，已实现 `开始/停止扫描`、按相机候选号 `观察簇候选`、`开始/停止跟踪簇候选`、`请求全量快照`、`读取簇级结果`、`读取/释放详细材料`。每个源帧只生成一份不可变簇包，扫描、观察和跟踪任务共享该包引用；观察只另建有界轮廓材料句柄，跟踪只附加候选视图和原样自我绑定令牌。结果队列按条数、序列化字节和帧龄硬限，满时在跟踪状态推进前回滚并背压，不静默丢包。合成混合负载已验证扫描 6/6 帧保留、跟踪5帧有界结束、一次观察材料哈希读回、释放和全量请求。首版只支持全画面扫描和按跟踪号观察，不支持 ROI/分辨率/最低频率实际调度、按全量序号加帧内号观察、原始像素材料、C++ 实时接线或性能结论。停止的是定向跟踪任务，不把仍由全幅扫描看到的相机候选伪造为 `Retired`。

`run_cluster_control_gate.py` 把 `PCS.RawSequence/1` 或实时相机接到上述影子调度层，逐帧生成唯一簇包、自动选取最大候选做有界跟踪和一次观察，并写出运行清单与逐帧耗时。真实静态录制前120帧已得到扫描120/120、跟踪119、观察材料哈希通过和零未释放材料；但同一运行 p50约1145ms、p95约1419ms、max约1764ms，明确不满足100ms生产门禁。该慢速包含 C++ 单帧观察、磁盘材料、Python转换和影子事务快照，只能作为协议/调度证据，不能用来推断 C++ 内联实现必然同样慢，也不能因“可优化”而忽略当前未达实时门槛。

`evaluate_cluster_scenario_capture.py` 在算法评分前先验证实录是否真的包含目标场景。`T3_local_motion` 要求至少60帧、至少20%的相邻帧存在变化，且变化像素和包围框的 p95 分别不超过全画面35%和50%，避免把整帧平移误写成局部运动。`T7_dark_depth_hole` 要求暗区 p50 至少占画面0.3%，暗区内原始深度0像素达到分辨率归一化下限，并在至少80%的帧中存在。固定像素门槛已被移除，避免分辨率改变结论。它只验证“材料含有局部变化/暗区深度孔洞”，不确认现实对象身份、场景真值或分割正确性，输出仍标记人工视觉复核待完成。

`capture_and_verify_cluster_scenario.py` 串联新采 `PCS.RawSequence/1`、上述内容门禁和簇级控制门禁，保存原始清单、内容指标、控制结果和顶层决策。60帧合成局部运动端到端测试已通过。真实 T3/T7 必须省略 `--replay-source` 从相机新采；旧 `d455_directory_replay_v1` 数据缺少完整标定与逐帧时间合同，不能转换后冒充本门禁通过。

`render_cluster_scenario_review.py` 从每段序列最多抽取6个等距样本，保留原图并生成复核叠加图。T3 以红色显示相邻帧变化、绿色显示变化包围框；T7 以黄色显示暗区、红色显示暗区与原始深度0值交集。图片 SHA256、图例和固定复核问题写入 `review_manifest.json`，并生成本地 `review.html`。采集编排会自动生成这些材料，但状态始终为 `pending`，程序不会替人确认红色区域就是目标或真实孔洞。

`export_raw_sequence.py` 是连续输入的受限导出器。它仍通过 `Client` 让 C++ 服务唯一持有相机，从每份已发布观察包只复制 `color.png`、`source_depth.png`，并提取实际内参、外参、深度单位、源帧号、逐流时间戳和共同时间域，生成可重放的 `PCS.RawSequence/1`。它不把补全深度、标签、轮廓、宿主发布时间或处理耗时写为源材料；输出的材料来源固定为 `历史回放`。合成“导出后回放”闭环已通过。当前源只含 RGBD，不含 IR 或 IMU，不能作为双目 IR 粗距、姿态补偿或绝对曝光时间的验证材料。

`evaluate_cluster_stability.py` 读取一个或多个已完成的 `cluster_stream.py` 输出目录，产出 `run_decision.json`、逐包指标和事件表。硬判据是严格包读回、连续输出序号、全量加增量重建、同输入多次运行的归一化包序列一致，以及静态输入中不出现 `Lost`、`Reappeared`、`Occluded` 生命周期事件。`本帧簇编号` 只在单帧有效，跨帧重用不能作为候选号切换证据；评估器把它写为 `frame_local_id_reassignment_count` 诊断项，但不将其计入通过条件。归一化只排除包标识、会话标识、发布宿主时间和上游观察包的会话相关清单 SHA256；源时间、簇、轮廓、证据、输出序号和跟踪关联仍必须一致。它只评价已给定流的静态一致性，不能证明物理场景静止、动态跟踪、世界身份或绝对深度精度。

`释放观察材料` 是已实现的 C++ 控制指令，参数为已经被调用方完整读取的 `输出序号`。服务拒绝释放仍在连续观察结果队列中的材料，释放后删除该观察包目录并归还它占用的包数/字节预算；重复释放明确返回 `unknown_material`。`cluster_stream.py` 和 `export_raw_sequence.py` 都只在成功复制所需内容后调用此指令，因此可在默认有界预算内处理最多2048帧，不把已消费的完整逐像素材料无限累积在服务端。

`run_cluster_stability_matrix.py` 是静态稳定性验收入口。它要求真实或合成的完整 `PCS.RawSequence/1`，默认顺序运行 600 帧、3 次重复并调用稳定性决策器；根目录写入输入清单 SHA256、Git 修订、配置快照、实际 `聚簇模式`、确认帧数和每次子运行状态。`--clustering-mode 深度主导|轮廓主导` 可在同一原始序列上严格比较两条路径；`--start-frame` 可从同一原始序列选择独立窗口，跳过的观察材料会立即释放，簇级输出序号仍从1连续开始。决策器逐包重建活跃状态，并要求每次运行至少出现一个候选，空输出不能通过。它另外记录 `tentative_removed_count` 和 `added_candidate_count`，防止把短命候选从遮挡状态机移除后误称为分割稳定。它拒绝帧数不足的输入，不能以旧 `datasets/*` 或名义帧率填补时间、标定证据。已用 600 帧合成静态输入完成 3 次重复：包验证、归一化确定性、重建、跟踪号切换和生命周期事件均通过。

真实静态序列的首轮600帧暴露了两类长期问题：已确认轨迹会被高代价小碎片逐帧拖离可靠主体；单包分割缺失会立即误报遮挡/重现。加入可靠关联参考和2包遮挡确认、3包丢失期限后，同一 `PCS.RawSequence/1` 顺序运行3次，每次600帧，包验证、规范化确定性、增量重建、候选存在和静态生命周期门禁全部通过；`static_lifecycle_event_count=0`、`run_max_active_tracks=[8,8,8]`。三轮仍有363次未确认候选移除、381次新增和2907次帧内局部号重用诊断，底层分割噪声没有消失，帧内号重用也不等于跨帧 ID 切换。结合 T9 合成故障注入，P2 Python 影子链的 T1/T2/T9 退出门禁已通过；轮廓相似度/中心抖动、真实动态发现、物理断连和 C++ 实时恢复仍未验证。

`capture_and_verify_static.py` 将真实静态采集和验收串成一项有界操作：通过 C++ 服务采集一份受限 RGBD 原始序列，再以同一序列运行指定次数的稳定性矩阵；顶层 `run_manifest.json` 链接导出清单和矩阵决策。`--replay-source` 只用于合成集成测试，真实验收必须使用 `--camera`。它不会在相机缺失、采集失败、帧数不足或矩阵失败时输出通过结论。

它不是实时管道的新默认输出：P2/P3 只在合成静态、遮挡和墓碑场景完成验证，未在真实动态回放中通过。未实现 C++ 实时接线、心跳生产、调度指令、详细材料租约、姿态补偿或精确三维升级。即使源包包含范围内深度，转换结果也只输出 `UnknownDistance` 和当前/估算/缺失/范围外证据计数，不把配置范围当作精度校准。旧 `datasets/*` 回放缺少 `PCS.RawSequence/1` 所要求的完整标定与逐帧时间域，不能为动态跟踪补造这些事实。完整字段、边界和分阶验收见 [PROTOCOL.md](PROTOCOL.md) 和 [簇级测试方案](../资料/20260920_D455簇级扫描观察跟踪稳定供包测试方案_v0.1.md)。

## 处理路径

### 原始启动诊断

```powershell
.\x64\Release\PixelClusterSensor.exe --probe-startup --frames=120 --sessions=3 --output-root=.codex_tmp/PixelClusterSensor/startup_new
python .\PixelClusterSensor\analyze_startup.py .codex_tmp/PixelClusterSensor/startup_new/probe.json --output .codex_tmp/PixelClusterSensor/startup_analysis_new
python .\PixelClusterSensor\test_startup.py --output .codex_tmp/PixelClusterSensor/startup_test_new
```

这是诊断入口，不是自我的新控制指令。它复用 `Source`，不运行后续分割，先把原始输入保存在有界内存中，关闭源后再写PNG，避免本轮PNG写盘改变读取负载。默认120帧、3次会话，范围为2至300帧、1至5次会话；原图缓冲上限512MiB，单会话读取预算20秒、尝试次数不超过请求帧数三倍。超时检查在调用边界，SDK单次等待仍有原来的1000ms预算，不承诺操作系统硬实时。每次打开重置源，不改曝光、增益和发射器。

原始诊断明确记录并重试 `stale_frame/unsynchronized_frame/missing_stream`，但不改变正常 `--stdio` 的严格失败语义。任意重试/源帧缺口都必须进入离线连续性检查；“请求数量完成”不是“无漏帧采集通过”。异常结束也保存已完成帧，诊断目录禁止覆盖。支持 `--replay-manifest=PCS.RawSequence清单` 做合成和EOF反例测试，保留原材料来源，不能标成实拍。

离线窗口使用固定15对帧、0.4至1秒窗口。按全帧及4x4分区的深度有效比例、有效/缺测翻转、共同有效点深度差、RGB平均差和变化像素比判定；缺测过多、时间不连续或样本不足时不产生“变化较小候选”。后验距离分带和区域统计仅解释失败，不改变门槛。即使产生候选，也不证明曝光/热稳定、物理静止或绝对测距正确；本轮实测联合门禁全部未通过，正常供包的 `预热状态=未证明稳定` 不变。

真实数据与诊断结果仅保存本地，不上传原始图片。完整规格与证据见 [启动验证报告](../docs/codex_analysis/pixel_cluster_sensor_startup_20260909.md)。

### 离线时域估计探针

时域估计是独立离线探针，不在下面的正常供包路径中。运行方式：

```powershell
python .\PixelClusterSensor\temporal_estimator.py PATH\probe.json --output .codex_tmp/PixelClusterSensor/temporal_new
python .\PixelClusterSensor\test_temporal_estimator.py --output .codex_tmp/PixelClusterSensor/temporal_test_new
```

输入为 `PCS.RawStartupProbe/1`；默认跳过每会话前30帧，不声称第31帧已预热稳定。每18帧为独立评测单元，前后各9帧，3/5/9帧方法分别使用各半段末尾的N帧，输出时刻一致。前后估计之间、不同单元之间均无共享输入。每对仅在全部18帧共同有效的像素及[0.3,3.5)m分带上作公平比较，拒绝集合、覆盖率和原始对照另外记录；不能把这一条件化样本当作全画面质量。

估计要求最新帧有效、至少80%样本有效、有效历史均在[0.3,3.5)m内。门禁中位数还要求时间深度跨度不超过 `0.04 + 0.02 * 最小深度米`，并检查窗口内各彩图对最新彩图的全局变化；彩图门禁只提供全局否决，没有把未配准的彩图像素直接映射到原始深度像素。无空间混合、无递归补全、无相机运动补偿，无法保证缓慢运动时不滞后。所有参数固化于脚本CONFIG及输出 `config_snapshot.json`，本次新采复验前已固定，没有按结果放宽。

输出包含 `paired_metrics.csv`、`summary.json` 和各会话第一有效单元的估计样本NPZ，样本保存当前原始深度、独立估计及状态、样本数、拒绝位和支持样本年龄；源帧信息在summary样本目录中，标定需通过其输入probe复核。它们位于原始深度像面，**不是**可直接替换彩图配准深度的 `PCS.Observation/1` 包。年龄由源时间戳计算，不是已校准的曝光到结果延迟。实测及运动反例见 [时域估计报告](../docs/codex_analysis/pixel_cluster_sensor_temporal_20260909.md)。

### 静态累积探针

按“保留已有信息，当前帧修正”的路线增加 `static_accumulator.py`。不是多帧平均：当前可用深度直接更新账本，当前0缺测才尝试复用至少3次相容观测支持、最近实测年龄不超过200ms的历史值。缺测复用不刷新历史年龄；源中断、彩图相对静态参考图变化、局部深度冲突及其邻域、范围外/饱和或超龄会使活动历史失效。原始文件不删除，但缓存不是永久无损历史仓库。

```powershell
python .\PixelClusterSensor\static_accumulator.py PATH\probe.json --output .codex_tmp/PixelClusterSensor/accumulator_new --repeats 3 --save-every-n 30
python .\PixelClusterSensor\test_static_accumulator.py --output .codex_tmp/PixelClusterSensor/accumulator_tests_new
```

每次update输出当前彩图、原始深度、累积深度、证据状态、最近实测年龄、来源索引和支持次数，全部为独立快照。状态明确分开当前观测与历史候选；彩图/深度仍各自在原始坐标系，不是新 `PCS.Observation/1`，暂不进入正常供包或分割。默认每30帧及末帧保存NPZ，设 `--save-every-n=1` 可逐帧保存；CSV逐帧记录，输出目录禁止覆盖。

360帧与另段120帧复验中，状态闪烁减少，但当前有效数值波动不变；同色遮挡且缺深度仍能留下旧候选。固定阈值、Python层计时对照、内存边界、字段及读回证据见 [静态累积验证](../docs/codex_analysis/pixel_cluster_sensor_accumulation_20260909.md)。不因缺测看起来减少就宣称物理深度更完整或方法可以晋级。

### 旧动态录制测试

`evaluate_dynamic_accumulation.py` 可直接读取旧 `d455_directory_replay_v1` 的配准彩图/毫米深度序列，算法仍使用上述静态累积器，不更改门槛。缺少时间戳时必须明确 `--nominal-fps 30`，其时效只是假设值；有 `source_timestamps.csv` 时不使用nominal时间。输出声明旧坐标与时间证据限制，不生成虚假 `PCS.RawStartupProbe/1`。

```powershell
python .\PixelClusterSensor\evaluate_dynamic_accumulation.py datasets/hand_occlusion_reappear --nominal-fps 30 --repeats 3 --output .codex_tmp/PixelClusterSensor/dynamic_new
python .\PixelClusterSensor\test_dynamic_accumulation.py --output .codex_tmp/PixelClusterSensor/dynamic_tests_new
```

慢移、手部遮挡和车载合计420帧全部没有历史复用；颜色门禁频繁整帧重置，动态累积收益未成立。局部颜色冲突、重观测深度冲突无历史分母时输出null，而非0%错误。详见 [动态累积测试](../docs/codex_analysis/pixel_cluster_sensor_dynamic_accumulation_20260909.md)。默认供包不变。

### 正常供包

1. 复核原始颜色、depth16、内外参、单位、尺寸和逐流时间。未知畸变模型、损坏材料及不相容时间明确失败。
2. 使用 RealSense SDK 从原始深度像素反投影，经真实深度到颜色外参变换，再投影到彩图像面。按最近像素中心落点并用 Z-buffer 解决竞争，同时保存源深度像素索引。不以 `resize` 代替配准，不复制源相机 Z 冒充彩图相机 Z。
3. 当前可用深度建立四邻接连续表面区域。连通判据为局部深度差不超过 `邻接深度差米 + 邻接深度相对差 * min(Z1,Z2)`；颜色不强制拆开有深度支撑的彩绘平面。
4. 剩余像素以 CIELAB 的 Delta E 76 邻接色差形成图像候选。纯缺测区域若未触及画面边缘、且只有一个颜色相容的周边深度区域，可继承其候选标签。多个深度锚点不会通过缺测区域合并。范围外观测不会因周边近场锚点被升级。
5. 对孤立缺测连通区做颜色引导插值。只读本帧、同簇、当前可用的邻居，使用空间距离与中心像素色差加权。前后区域冲突、样本不足、深度跨度过大、缺测面积过大或触及视野边缘时不补；估算结果绝不作为下一次补全的输入。
6. 从标签图生成无近似简化的完整 OpenCV 边界链和内外环。像素成员以标签图为准；内环只是拓扑，不自动确认物理穿孔。
7. 将数组、标定、配置、来源、请求编号、簇目录及 SHA256 写入未发布目录。完整写出后重命名为正式目录，最后返回材料引用。失败残留的 `pending_*` 目录不属于已发布观察，不自动覆盖或清理。

这一基线是有明确规则的区域候选算法，不是通用实例分割，也不是旧主程序冠军方案的迁移。图像候选可以覆盖所有像素，但不能据此声称物理归属正确率为100%；包中的该指标为 `null`。

### 轮廓主导实验路径

`聚簇模式=轮廓主导` 实现“彩图闭合区域先给出像素归属候选，可靠深度只约束空间拆分”的实验口径：

1. 全画面先以 CIELAB 四邻接色差形成彩图区域，所有像素先进入图像候选账本。
2. 仅 `当前深度状态=1` 的范围内可靠深度建立连续表面种子；范围外深度保留在簇内作为观测属性，不参与可靠边界拆分，也不升级为精确深度。
3. 同一彩图区域内，被缺测像素隔开的相容深度种子按深度门槛合并；不同彩图区域的可靠深度若在共享边界连续，也可合并，避免彩绘平面被颜色切碎。
4. 一个彩图区域只有一个可靠深度根时，缺深度像素继承该候选；存在多个互不相容深度根时，缺深度部分保持独立图像候选，不桥接前后表面。
5. 完全缺深度、未触及画面边缘且只被一个可靠深度簇包围的闭合区域继承外围标签，避免把人体或物体内部的传感器缺测直接输出成黑洞。该规则不补造深度值；真实开口仍需背景深度、时序显露或其他反证确认。

该模式输出 `彩图初始区域数`、`当前深度种子区域数`、`跨颜色连续深度合并数`、`跨缺测相容深度合并数` 和 `封闭缺深度继承像素数`。处理算法标识为 `contour-owner-depth-constraint/1`；原路径为 `depth-anchor-color-owner/1`。

合成协议反例已验证：彩绘连续平面不因颜色拆开、同色深度台阶仍拆分、缺测区域不桥接两个深度表面、封闭 3x3 缺测区保留外围归属、范围外深度不晋级。真实静态同源短门禁（30帧、3次、最小簇1024像素）中，轮廓主导路径产生138个静态生命周期事件，深度主导对照为78个，两者均未通过静态稳定门禁；轮廓主导的单次源处理均值约80.5ms，对照约51.1ms。因而本轮只证明帧内语义和协议路径，**不证明跨帧更稳定或性能更好**，默认仍保留 `深度主导`。后续必须处理可靠深度边界抖动和彩图区域跨帧稳定，再讨论晋级。

## 默认配置

所有参数均为第一版可复算实验配置，不是本机已完成精度标定的生产阈值。距离范围内观测也不自动获得精确三维保证。

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| 聚簇模式 | 深度主导 | 已验证基线；可显式切换为实验性的轮廓主导路径 |
| 可用深度近界米 | 0.3 | 范围内观测资格的近界 |
| 可用深度远界米 | 3.5 | 范围外保留原值、颜色及图像归属，不冒充近场精度 |
| 邻接深度差米 | 0.04 | 局部表面连通绝对差 |
| 邻接深度相对差 | 0.02 | 随距离增长的连通容差项 |
| 归属色差 | 20 | 无可用深度区域的局部颜色连通阈值 |
| 启用补全 | true | 是否生成独立插值层；原始材料不变 |
| 补全最大缺测连通像素 | 4 | 超过此面积不补，最多可请求16 |
| 补全最少样本 | 3 | 3x3邻域至少需要的当前可用样本，允许3至8 |
| 补全深度跨度米 | 0.03 | 有色差支持的深度样本跨度上限 |
| 补全色差 | 20 | 中心与邻居色差门槛，同时作为高斯权重尺度 |

配准可产生新缺测像素，尤其在多流不重叠区域和遮挡处；这不会删除彩图像素。单一颜色和缺测形状不足以可靠判定所有真实开口，首版不声明解决了这一物理歧义。

## 控制实现范围

已实现：查询设备能力、查询运行状态、打开设备、关闭设备、读取配置与标定、设置处理配置、获取单帧观察、开始连续观察、停止连续观察、读取观察结果、释放观察材料、取消请求、退出。

- 实时相机首版仅支持 `640x480@30` 的 RGBD 组合；具体硬件能否打开仍以实际 SDK 返回为准。请求其它组合拒绝，不静默降级。
- 打开时不修改曝光、白平衡、增益或发射器。首帧不宣称已预热稳定；源时间域保留，不把宿主接收时间当曝光时间。
- 启动阶段首帧配对尚未通过时，在单次1000ms采集预算内筛除缺流、旧帧、时间域不同或时间差超过50ms的帧对；预算耗尽明确失败，不发布错误配对。首帧配对通过后不自动筛掉异常，仍明确失败。配置读回的 `首帧配对已通过` 仅指输入门禁通过，不代表包已发布或曝光/深度质量已稳定。
- 实时源信息和配置读回保留 `启动拒绝帧对累计数`、`最近启动拒绝帧对`；拒绝诊断带原生帧号、时间戳、时间域及原因。逐帧 `预热状态=未证明稳定`，不会把启动缺测多的帧包装成稳定观测。这是2026-09-09实机发现后的默认采集修正，不改变算法配置。
- 单帧含义是本会话未交付过的源帧，不承诺曝光发生在请求之后，不提供“缓存当新帧”。
- 处理参数通过预期配置版本门禁修改，未指定字段保持现值；整组校验通过后生效。连续观察中必须先停止任务再修改。
- 连续任务以请求编号为任务编号，结果经 `读取观察结果` 拉取；队列最多32份引用。`完整处理` 队列满后背压，若任务内实际输入出现源帧缺口则失败，不能宣称完整连续采样；`实时优先` 可丢弃最旧未消费引用，但准确计数且保留已写材料。
- 输入完整性不能凭模式名称保证：真实相机继续运行时，背压可能导致 SDK 层丢帧；源流缺口独立记录。回放不会在队列满时继续读取。
- 取消在连续任务帧边界生效，最多还需完成当前采集/处理/发布；单帧不做抢占取消。采集等待上限1000ms，SDK设备打开及文件系统操作不承诺硬实时上界。
- 关闭设备前需先取完任务结果。关闭管道或退出会取消活动任务、释放设备，但保留已经发布的文件。结果引用尚未拉取时，异常断开后只能在输出目录检查；没有实现跨进程任务恢复。

成像参数运行时控制、动态采集规格、关注区域、局部复核、IMU订阅、输出字段订阅、共享内存租约、专用录制和自动重连均返回 `unsupported_command`。不以空回执声称支持。

## 存储、确定性与验收边界

默认输出根为 `.codex_tmp/PixelClusterSensor`。每个设备打开会话使用随机唯一子目录，不覆盖前次输出。每个进程默认最多128个包、256 MiB，分别可用 `--max-packets`、`--max-bytes` 调整；单包上限96 MiB、单图像上限921600像素、最多32768簇，轮廓扫描和点数另有预算。无后台删除或自动云端上传功能。

完整目录重命名是发布可见性边界，不是断电持久性或跨进程事务保证。完整帧观察与自我的事实采用分开。请求去重仅在该进程内有效，账本满后拒绝新请求，不淘汰后重新执行；关闭输入始终可结束进程。

独立读回验证内容长度、SHA256、标签与簇目录、内外环、原色重建、深度有效性、原始深度保留和几何重投影。合成反例同时检查深度台阶、斜面、彩绘平面、背景开口、孤立缺测、颜色权重及不确定降级。同输入重复验证完整数组一致；没有证明跨平台逐位一致。

`test_camera.py` 必须显式运行才打开硬件，不纳入无人值守的合成测试。默认12帧实时优先任务，可设2至20帧；另测试单帧、故意150ms间隔引起的完整模式源缺口、停止/读完结果、同进程重开三次、EOF释放后跨进程重开。每个包都独立读回，校验放在关闭相机后执行，避免改变采集负载。报告分开统计会话首帧、后续帧和实时优先任务；单帧往返时间包含等待、处理和写盘，不是曝光到结果的真实年龄。输出为本地 `camera_report.json`、原始RGBD与处理材料、第二帧读回图，不自动上传。故意制造的 `source_gap` 是拒绝机制通过，不能计作完整连续采集成功。

协议见 [PROTOCOL.md](PROTOCOL.md)，实施阶段见 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md)。检验记录见 [首轮验证](../docs/codex_analysis/pixel_cluster_sensor_initial_20260908.md) 和 [真实相机调试](../docs/codex_analysis/pixel_cluster_sensor_camera_20260909.md)。实机供包已验证，但重建一致不能代替真实分割质量、绝对测距准确性或现有 fixed replay gate；本轮不晋级任何 best 配置。
