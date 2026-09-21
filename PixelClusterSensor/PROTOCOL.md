# 像素簇视觉外设协议 v0.1

这是新子工程的供包与控制协议，不是数字生命项目现有 ABI 或事实格式的替代。现有格式为 `PCS.Control/1`、`PCS.RawSequence/1`、`PCS.Observation/1`和 2026-09-20 新增的 `PCS.ClusterObservation/1`，不预分配受端的正式类型编码。

## 控制传输

本机标准输入输出，每行一个 UTF-8 JSON 对象，以 LF 结束，允许 CRLF。日志走 stderr，stdout 仅输出回执，不混入画面或调试文字。不接受重复 JSON 字段、未知顶层字段、超过32层的 JSON 或超过65536字节的请求。

```json
{
  "协议": "PCS.Control/1",
  "请求编号": "请求-0001",
  "指令": "打开设备",
  "参数": {"来源": "实时相机", "设备序列号": ""},
  "截止Unix毫秒": 9999999999999
}
```

示例截止时间只用于展示字段；调用方实际应提供有界、尚未到期的截止时间。时间由本机系统时钟解释，不是相机时钟。请求编号为1至128个UTF-8字节；所有设备会话内指令还需提供返回的 `会话标识`。

| 指令 | 参数 | 结果语义 |
| --- | --- | --- |
| 查询设备能力 | 空对象 | 设备清单、已实现与未实现能力、规格和资源限额；不打开采集 |
| 查询运行状态 | 空对象 | 设备、任务、队列、最后错误、最近包及存储使用量 |
| 打开设备 | 实时：来源、可选设备序列号、可选宽/高/帧率；回放：来源=目录回放、清单路径 | 返回新会话、当前配置版本及实际标定；已有会话时拒绝 |
| 读取配置与标定 | 空对象 | 完整实际标定、深度单位和完整处理配置 |
| 设置处理配置 | README中的可选处理参数 | 顶层必须带匹配的 `预期配置版本`；通过后返回新版本及实际值 |
| 获取单帧观察 | 空对象 | 同步返回一份完整观察包引用；新接收不等于请求后曝光 |
| 开始连续观察 | 帧数1至10000；模式=完整处理或实时优先；最长秒数1至3600，默认60；间隔毫秒0至10000，默认0 | 命令完成表示任务已启动，不表示所有帧已完成 |
| 读取观察结果 | 任务编号；最多帧数1至8，默认8 | 返回并移出有界队列的结果引用，以及独立的任务状态 |
| 释放观察材料 | 已读取的输出序号 | 删除已发布观察包，归还服务端包数和字节预算；队列中未消费的引用拒绝释放 |
| 停止连续观察、取消请求 | 任务编号 | 在帧边界取消指定连续任务；不撤销已发布材料 |
| 关闭设备 | 空对象 | 队列有未取结果则拒绝；否则取消任务并释放源 |
| 退出 | 空对象 | 取消任务、释放设备并退出；若设备已打开，仍需匹配会话 |

未知或未实现指令明确失败，不调用命令行 shell、不使用拼接指令执行。

成功回执：

```json
{"协议":"PCS.Control/1","请求编号":"请求-0001","状态":"完成","结果":{"会话标识":"生成的会话标识","配置版本":"1"}}
```

失败回执：

```json
{"协议":"PCS.Control/1","请求编号":"请求-0002","状态":"失败","错误":{"代码":"stale_session","说明":"设备会话不匹配"}}
```

解析失败且无法取得请求编号时编号为 `null`。同编号同完整内容返回原回执，包括取结果请求的原引用，不再次消费或采集；同编号不同内容返回 `request_conflict`。改变截止时间也属于改变请求内容。状态查询要使用新编号取得新状态。

请求账本在整个供包进程内保留，最多4096条且有16 MiB预留预算，不跨进程持久化。请求截止时间限制开始执行资格，连续任务另有自己的时限。任务状态为无任务、运行中、完成、取消、失败或未完成；提前EOF、源帧缺口、预算不足不返回完整成功。已经完成的部分包仍可读取。

## 观察包

`frame.json` 记录 `格式=PCS.Observation/1`、发布状态、会话、输出序号、源信息、控制来源、配置版本、SDK版本、算法与参数、完整标定、图像尺寸、状态编码、材料描述符、簇目录及指标。

每个描述符包括相对文件名、编码、形状、字节数及完整 SHA256。清单的 SHA256 在控制回执中另行交付，避免把摘要当材料。接收方需要读取全部材料，不能只看一张叠加图。

| 材料 | 编码及形状 | 含义 |
| --- | --- | --- |
| color.png | PNG_RGB8，H×W×3 | 全帧彩图，解码后RGB通道；内部OpenCV BGR转换不改变颜色 |
| source_depth.png | PNG_U16，原深度H×W | 输入深度原值；单位及0/65535无效码在标定中明确 |
| labels.bin | uint32_le，H×W | 本帧局部簇编号，0未分配；不是世界身份 |
| ownership.bin | uint8，H×W | 0未解决，1当前深度支撑候选，2图像候选，3周边单锚点继承候选 |
| depth_m.bin | float32_le，H×W | 经外参变换的彩图相机光轴Z，单位米 |
| depth_state.bin | uint8，H×W | 0无观测，1在配置范围内，2在配置范围外；1不等于绝对精度保证 |
| depth_source_index.bin | int32_le，H×W | 对应原始深度图的行优先索引，-1无采样；配准竞争选择近表面，等深度保留先遇到的源像素 |
| filled_m.bin | float32_le，H×W | 独立估算值，不覆盖depth_m |
| fill_state.bin | uint8，H×W | 0未补全，1当前帧颜色引导插值 |
| contours.bin | int32_le，N×3 | 每项为像素中心x、y、边界原因位；不做轮廓近似压缩 |

浮点数组只允许有限数值；无效槽存0，必须以状态判定无效，不能读成实际距离。彩图坐标以左上像素中心为(0,0)，u向右、v向下；相机三维坐标X向右、Y向下、Z向前。数组行优先、无额外行填充。

源信息保留逐流帧号、时间戳及时间域、接收Unix毫秒和可知的源帧缺口。接收时间不证明曝光时间，未建立绝对采集时间校准时明确标false。回放不能被标为实时相机。尚未计算真实端到端结果年龄；不拿处理时间或接收时间差冒充它。

2026-09-09 实时源补充启动诊断，格式仍为向后兼容的 `PCS.Observation/1`：`启动拒绝帧对累计数`、`最近启动拒绝帧对`（无拒绝时null；否则含代码、逐流源帧号、时间戳、时间域），以及 `预热状态=未证明稳定`。累计数为本次设备打开会话累计值，不能逐帧求和当作总拒绝数。`读取配置与标定` 的实际配置同步提供两项启动诊断及 `首帧配对已通过`；这个标志不等于观察包已完成发布。

仅首帧配对尚未通过时允许在单次1000ms采集等待预算内跳过不相容帧对；通过一次后不再隐式重试异常。相同时间域且差值不超过50ms才进入处理，门槛未放宽。超时或异常仍返回失败；没有自动重开设备、修改时间戳或预热合格承诺。会话首份可接受输入之前的丢弃由启动诊断记录，不伪造成完整源序列。`System Time` 域的配对差是SDK提供的时间差，不能当成独立校准过的曝光同步误差。

### 原始诊断与元数据

实机源信息新增只读 `逐流元数据`，分别记录彩图、深度的SDK字段。每个字段包含 `支持`、十进制字符串 `原始值`（不支持时null），读取异常另有 `读取错误`，不把缺失值变为0。字段包括Actual Exposure、Gain Level、Auto Exposure、Frame Timestamp、Sensor Timestamp、Backend Timestamp、Time Of Arrival、Actual Fps；名称来自SDK，数值未经换算。尤其驱动后端时间单位/语义不能自动当作设备曝光时间，SDK文档与本机实际行为需另行核对。当前实机缺少曝光、增益及设备帧/曝光时间元数据；不修改驱动或注册表以强行启用。

诊断CLI的 `PCS.RawStartupProbe/1` 与 `PCS.Observation/1` 分开：`probe.json` 含采集参数、实际标定、会话、拒绝事件、逐帧源信息和彩图/深度PNG文件引用（字节数与SHA256）。保留不同逐流帧号，不转换成要求同号的旧回放清单。没有像素簇、世界身份或自我接纳语义。`PCS.StartupAnalysis/1` 是后续离线实验结果，不回写原始材料或预热状态；不能当成当前观察。

### 簇与轮廓

每个簇记录局部编号、实际像素数、XYWH包围矩形、当前范围内/范围外/补全/未解决深度计数，以及轮廓目录。编号按最小源像素行优先排序生成，可重复计算，但不承诺跨不同帧保持身份。

轮廓目录记录全局点数组起点、点数、同簇父轮廓索引、内环标志及闭合区域边界标志。首版采用 OpenCV `RETR_TREE + CHAIN_APPROX_NONE` 的8方向像素中心链，**不是设计讨论中的像素方格顶点格式**，不混送旧坐标格式的比较器。标签图才是精确成员集合，不能仅凭轮廓填充恢复所有像素。

边界原因位：1算法区域分界，2相邻深度缺测，4视野截断，可组合。区域闭合不等于物体完整；所有 `物理孔洞确认` 当前为false，不把内环拓扑升级为真实开口确认。真实背景有观测时保留为另一簇，不用填充覆盖；完全缺测且颜色相同的穿孔仍存在不可判定歧义。

包不携带已确认存在身份、体素、完整物体尺寸或世界位置。原图大小、颜色和来源不因归一化轮廓比较被覆盖。

## 簇级观察包

`PCS.ClusterObservation/1` 是供自我高频读取的簇级包，不是取代 `PCS.Observation/1` 的原始材料包。正常情况下它不重复输出 RGB、Depth、IR、标签图或逐像素数组；需要实际材料时由后续有界租约句柄读取。

簇级包的三个身份必须分开：

```text
帧内簇编号 != 相机跟踪候选编号 != 自我确认的存在身份
```

相机只能产生前两者。自我绑定令牌可由自我提供且相机原样回传，但相机不解释、不新建、不修改其含义。

### P0/P1/P2/P3 当前实现范围

已提供独立严格读回器 `cluster_protocol.py`、无状态转换器 `convert_cluster_observation.py` 与离线短期跟踪器 `cluster_tracker.py`。转换器只将一份完整 `PCS.Observation/1` 转成 `FullSnapshot + Scan` 簇级包；跟踪器再将连续的全量快照转成首帧全量、后续增量：

- P2 按包围矩 IoU、中心位移、颜色差、面积比和 32x32 形状指纹作硬门禁与代价。对每个有限联通竞争组使用全局最小代价一对一分配；节点数超过 64 的组不关联，宁可显式新建候选也不拿不可审核的贪心结果冒充确认。未确认候选的关联还受最大代价门禁，默认 `200000/1000000`；已确认动态轨迹不受该确认门槛限制。当前仅有合成动态反例和真实静态短窗口验证，未经真实动态回放验收。
- 首帧返回 `FullSnapshot`，后续帧返回带基线序号的 `Delta`。新候选和 `Tentative` 关联必须达到新候选像素阈值；候选连续达到配置的确认帧数后才进入 `Active`。只有 `Active` 轨迹可以在后续帧使用较低的保留像素阈值，避免预跟踪硬过滤造成一次缺测，同时不让低于准入门槛的碎片自行晋级。未确认候选消失时发布 `Removed/Retired` 墓碑，遮挡状态为 `Unknown`，不得写成物理遮挡。只有已确认候选临时无匹配时才发布 `Occluded`，连续缺失达期限后发布 `Lost/Retired`；重现时连续可见帧数从1重新计算。无帧内簇的状态记录不携带当前轮廓、深度或材料。
- 未实现实时 C++ 供包接线、心跳生产、扫描/观察/跟踪调度指令、姿态补偿或详细材料租约。
- 既有 `PCS.Observation/1` 的“配置范围内观测”不等于经标定证明的精确三维。因此转换器一律输出 `UnknownDistance`，同时保留当前实测、当前插值、缺失、范围外像素计数；不得伪造 `PreciseDepth3D`。
- P1 形状指纹是 `label-mask-center-square/1`，保留当前标签图的结构。它不把拓扑内环自动认定为真实穿孔；物理孔洞仍需额外背景证据。
- P3 `cluster_stream.py` 是 Python 影子/评测桥，唯一相机所有者仍是它通过 `Client` 启动的 C++ 服务。它逐帧执行 P1 转换和 P2 跟踪，写出 `cluster_packets/`、`run_manifest.json`、`packet_metrics.csv` 和 `events.csv`；首帧为全量，静态无变化帧为无簇变化的 `Heartbeat`，新增/移动/遮挡/重现/丢失为 `Delta`。合成静态、遮挡和重现已验证连续候选号和重建。心跳只证明源流和供包链仍工作，首版不在心跳中刷新逐簇证据年龄；它不等于 C++ 实时接线或扫描/观察/跟踪控制指令。
- `export_raw_sequence.py` 从已发布观察包导出受限的 `PCS.RawSequence/1`：仅复制彩图和原始深度，并保留实际标定、深度单位、帧号、时间戳和共同时间域。导出清单的材料来源恒为 `历史回放`，不把处理派生层或宿主时间写成源证据。合成导出后回放已经验证；当前 C++ 输入未采集 IR/IMU，因此导出不承诺这两类流。
- `evaluate_cluster_stability.py` 为已完成的 P3 流输出独立决策文件。它验证包、连续簇级序号和逐包全量加增量重建，并在多次同输入运行间比较归一化包序列；每次运行至少必须出现一个候选，空包流不能获得稳定性通过。决策同时记录已确认候选生命周期事件、未确认候选移除数和新增候选数，防止用状态降级掩盖分割抖动。包标识、会话标识、发布宿主时间和包含这些字段的上游观察清单 SHA256 被明确排除，其余源时间、簇证据、轮廓和跟踪关联仍须一致。它不把静态回放一致性解释为物理静止、动态跟踪或世界身份。
- C++ 已实现 `释放观察材料`。调用者只能在已完整读取对应材料且它不在连续结果队列中时按输出序号释放；服务删除目录、归还包数和字节预算，重复/跨会话释放返回明确失败。P3 影子桥和原始序列导出器在复制所需内容后立即释放，当前可在有界预算内处理最多2048帧；不以后台淘汰替代调用者的显式确认。

### 包头和簇记录

包头必须含 `格式`、`发布状态=完整`、`包标识`、`会话标识`、`跟踪时期`、`输出序号`、`场景版本`、`包类型`、`任务意图`、`依赖全量序号`、`源时间`、`发布Unix毫秒`、`结果年龄毫秒`、`配置版本`、`标定版本`、`坐标系`、`图像尺寸WH`、`相机姿态`、`处理区域`、`输入质量`、`全局覆盖摘要`、`材料`和`簇变化`。读回器拒绝未知顶层字段、非法枚举、非有限数值、超界图像/材料和非规范十进制序号。

每个簇记录包含帧内编号、跟踪候选、变化/跟踪状态、图像范围、轮廓、分级形状指纹、颜色、距离模式、深度证据分账、运动、遮挡、关联证据、时效和详细材料句柄。P1 不具备的字段明确为 `null` 或已定义的 `Unknown` 状态，而不冒充具备证据。

### 精确轮廓材料

首版精确轮廓用直接包内的 `contours.bin`，编码 `PCS.ContourChain8/1`。每个环保存规范化起点 `XY`、点数、字节偏移和有效位数；每个边使用 3 bit 表示 8 方向步进。外环顺时针、内环逆时针，起点取 `(y,x)` 字典序最小点；最后一步必须回到起点，编码的高位填充为 0。读回器对环闭合、连通、字典序、孔洞层级和材料 SHA256 进行校验。

目前校验命令：

```powershell
python .\PixelClusterSensor\test_cluster_protocol.py --output .codex_tmp\PixelClusterSensor\cluster_protocol_tests_new
python .\PixelClusterSensor\test_cluster_conversion.py --output .codex_tmp\PixelClusterSensor\cluster_conversion_tests_new
python .\PixelClusterSensor\test_cluster_tracker.py --output .codex_tmp\PixelClusterSensor\cluster_tracker_tests_new
python .\PixelClusterSensor\test_cluster_stream.py --output .codex_tmp\PixelClusterSensor\cluster_stream_tests_new
python .\PixelClusterSensor\convert_cluster_observation.py PATH\frame.json --output .codex_tmp\PixelClusterSensor\cluster_packet_new
python .\PixelClusterSensor\cluster_tracker.py PATH\packet_1.json PATH\packet_2.json --output .codex_tmp\PixelClusterSensor\tracked_packets_new
python .\PixelClusterSensor\cluster_stream.py --replay PATH\sequence.json --frames 3 --output .codex_tmp\PixelClusterSensor\cluster_stream_new
python .\PixelClusterSensor\export_raw_sequence.py --camera --frames 120 --output .codex_tmp\PixelClusterSensor\raw_sequence_capture_new
python .\PixelClusterSensor\test_export_raw_sequence.py --output .codex_tmp\PixelClusterSensor\raw_sequence_export_tests_new
python .\PixelClusterSensor\evaluate_cluster_stability.py --runs RUN_A RUN_B RUN_C --output .codex_tmp\PixelClusterSensor\cluster_stability_new
python .\PixelClusterSensor\test_evaluate_cluster_stability.py --output .codex_tmp\PixelClusterSensor\cluster_stability_tests_new
python .\PixelClusterSensor\run_cluster_stability_matrix.py --replay PATH\sequence.json --frames 600 --repetitions 3 --minimum-cluster-pixels 1024 --retained-cluster-pixels 512 --maximum-tentative-match-cost 200000 --confirmation-frames 5 --output .codex_tmp\PixelClusterSensor\cluster_stability_matrix_new
python .\PixelClusterSensor\test_run_cluster_stability_matrix.py --output .codex_tmp\PixelClusterSensor\cluster_stability_matrix_tests_new
python .\PixelClusterSensor\capture_and_verify_static.py --camera --frames 600 --repetitions 3 --minimum-cluster-pixels 1024 --retained-cluster-pixels 512 --maximum-tentative-match-cost 200000 --confirmation-frames 5 --output .codex_tmp\PixelClusterSensor\static_capture_gate_new
python .\PixelClusterSensor\test_capture_and_verify_static.py --output .codex_tmp\PixelClusterSensor\static_capture_gate_tests_new
python .\PixelClusterSensor\cluster_protocol.py PATH\packet.json
```

## 回放输入

回放清单不是旧 `datasets/*/case_manifest.json` 的兼容别名。源输入必须有适用标定、深度单位和明确时间域；不调用主程序的估算内参回退。

```json
{
  "格式":"PCS.RawSequence/1",
  "材料来源":"合成夹具",
  "设备标识":"synthetic-only",
  "深度单位米":0.001,
  "彩图内参":{"宽":2,"高":2,"焦距X":100,"焦距Y":100,"主点X":0.5,"主点Y":0.5,"畸变模型":0,"畸变系数":[0,0,0,0,0]},
  "深度内参":{"宽":2,"高":2,"焦距X":100,"焦距Y":100,"主点X":0.5,"主点Y":0.5,"畸变模型":0,"畸变系数":[0,0,0,0,0]},
  "深度到彩图外参":{"旋转列优先":[1,0,0,0,1,0,0,0,1],"平移米":[0,0,0]},
  "帧列表":[{"彩图":"color.png","深度":"depth.png","源帧号":"1","彩图时间戳毫秒":0,"深度时间戳毫秒":0,"时间域":"synthetic_clock"}]
}
```

输入PNG必须与对应标定尺寸一致。路径相对清单目录，禁止目录逃逸及逃逸符号链接。帧列表非空且不超过2048项，源帧号为严格递增的无符号64位十进制字符串；缺口保留。首版一项对应同号的归档源帧，不能在导入时把不同相机原生帧号随意改成同号来掩盖差异；需要保留不同逐流原生编号的历史归档适配另行扩展。

支持 RealSense 畸变枚举0、2、4，分别为无畸变、Inverse-Brown-Conrady、Brown-Conrady；采用该SDK的投影/反投影语义并记录版本。其它模型拒绝。逐流时间域必须相同，时间差需不超过50ms；50ms是原型输入门槛，不证明运动场景已经足够同步。
