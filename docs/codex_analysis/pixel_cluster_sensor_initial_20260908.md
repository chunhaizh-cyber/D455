# PixelClusterSensor 首轮实现与验证

日期：2026-09-08。分支：`codex/visual-capability-loop-plan`。

## 结论

独立供包工程已建立；合成输入的单帧、连续观察、材料完整性和控制协议验证通过。没有进行候选质量晋级，没有修改旧评分门槛、best配置或主程序算法。

本轮读取 RealSense 设备清单返回 `{"设备":[]}`。实时相机采集、真实对象分簇精度和正式自我接线均不在已通过结论中。相机采集路径已实现，但必须连接设备后另行运行。

## 变更范围

- 新增 `PixelClusterSensor/`：独立MSBuild工程、只读原始材料输入、单帧处理、完整观察包、JSONL控制服务、参考客户端、独立读取验证和合成测试。
- `D455.slnx` 只登记新工程；`README.md` 说明新方向、默认行为、参数和验证。
- 未改 `D455.cpp`，未改写 `datasets/`、历史 `analysis_runs/`、录制文件或用户未跟踪的原规划文档。
- 主程序的旧目录回放估算内参不复用到新工程。新输入缺标定或不支持的畸变模型会失败。

## 已执行验证

```powershell
msbuild .\PixelClusterSensor\PixelClusterSensor.vcxproj /p:Configuration=Release /p:Platform=x64 /m /verbosity:minimal
msbuild .\PixelClusterSensor\PixelClusterSensor.vcxproj /p:Configuration=Debug /p:Platform=x64 /m /verbosity:minimal
msbuild .\D455.vcxproj /p:Configuration=Release /p:Platform=x64 /m /verbosity:minimal
python .\PixelClusterSensor\test_protocol.py --output .codex_tmp/PixelClusterSensor/tests_release_final
python .\PixelClusterSensor\test_protocol.py --exe .\x64\Debug\PixelClusterSensor.exe --output .codex_tmp/PixelClusterSensor/tests_debug_final
python .\PixelClusterSensor\client.py --replay .codex_tmp/PixelClusterSensor/tests_release_final/fixtures/plane/sequence.json --frames 2 --validate --output .codex_tmp/PixelClusterSensor/client_smoke
.\x64\Release\PixelClusterSensor.exe --list-devices
```

前三个构建成功。编译存在第三方OpenCV头文件C4864警告，没有通过修改第三方头或屏蔽警告掩盖它。

Debug、Release各21组测试通过。完整报告和图像分别留在 `.codex_tmp/PixelClusterSensor/tests_debug_final/test_report.json`、`.codex_tmp/PixelClusterSensor/tests_release_final/test_report.json` 及其邻近目录；这些目录是本地生成证据，不自动提交。参考客户端命令行路径另外完成两帧采集、独立读回和正常关闭，均通过。

| 验证组 | 通过的内容 |
| --- | --- |
| 恒深平面、多色平面、连续斜面 | 声明范围内保持连续区域，不以颜色花纹直接拆开有深度支撑的平面 |
| 同色深度台阶 | 保持前后区域分开 |
| 单点缺深度 | 像素和颜色保留，原始缺测状态不变，补全值独立保存 |
| 中心颜色引导 | 以缺测点自身颜色选择邻居，异色邻居不进入该合成例的补全值 |
| 多锚点缺口 | 不通过缺测区域连接前后两组深度，不平均出虚构中间表面 |
| 大缺测区 | 不递归传播插值，不删除原色及区域候选 |
| 可见背景开口 | 背景保留独立标签及内环，不以填充覆盖 |
| 全无深度、远距、饱和值 | 保留图像候选和原始材料，正确区分缺测、范围外与插值 |
| 非零外参 | 输出是彩图相机光轴Z，不是直接复制源深度相机Z |
| Brown/Inverse-Brown畸变 | SDK配准与独立数组重投影检查通过，不默认无畸变 |
| 控制协议 | 请求去重、同号冲突、旧会话/配置、过期、字段错误、重复JSON、超长请求及配置读回 |
| 输入拒绝 | 缺标定、错误模型/旋转、整数溢出、源编号重复、回放冒充实时、尺寸错配、时间不相容、路径逃逸 |
| 连续观察完整模式 | 40帧，32引用满队列时背压，最后按源顺序完整交付40帧 |
| 连续观察实时模式 | 40帧，保留32引用，明确记录8份未消费引用丢弃，未冒充全交付 |
| 取消、提前源EOF、控制EOF | 帧边界取消，提前源EOF失败但保留已完成包，管道关闭后进程退出 |
| 源断续、任务期限 | 完整任务发现源帧缺口即失败，到期未完成不会报告成功 |
| 配额与损坏材料 | 包数预算限制生效，篡改数组SHA256读回失败 |

每个几何案例还重复输入两次，比较完整数组文件描述符及簇目录，验证同一构建下重复一致。独立读取按簇重建颜色，验证与合成源RGB逐像素一致、原始depth16不变、标签/轮廓/统计/来源相容。

## 不能从本轮推导的结论

1. 不证明真实场景中已经得到正确物体实例；连通表面与物体身份不同。
2. 不证明绝对深度或尺寸精度；重投影检查主要验证实现与声明标定的一致性。
3. 不证明所有真实孔洞都能识别；同色且缺测的前景/背景仍可能歧义，物理孔洞确认字段仍为false。
4. 不证明处理实时性。测试是小尺寸合成夹具，不与旧candidate分数或100ms门槛混评。
5. 不证明动态曝光、IMU、关注区域、录制、重连或共享内存租约已实现；能力查询明确列为未支持。
6. 不证明跨进程幂等或停电后事务持久性；完整目录发布、进程内去重和世界事实采用分开。
7. 不证明正式自我已经消费；当前是本地供包端及参考消费者，受端合法ABI仍需确认。

## 下一验证前置

连接真实D455，再执行 `python PixelClusterSensor/client.py --camera --frames 3 --validate`，检查设备标定、实际源帧、原始材料、配准缺测和关闭重开。之后才进入真实场景分割及错误插值评测，不直接扩大旧候选矩阵。
