# 居中二值轮廓 8x8 打包与静态相似度测试

## 目标

本诊断用于测量同一静止场景连续经过主分割、稳定跟踪和存在内部穿孔过滤后，同一存在的最终轮廓形状和位置是否变化。输入直接使用 `ExistenceContourHoleResult.ownershipMask`，不重新运行一套独立边缘算法。

当前硬目标为：同一 `observation_id` 且 `previous_gap_frames == 1` 的连续帧对，居中轮廓前景 IoU 和位置相似度必须分别严格大于 `99%`。两项不能互相补偿；任一连续帧对未同时过线，整体目标即为 `fail`。

该输出仍是观察候选材料，不是已确认现实存在或世界事实。

## 归一化与压缩

每个采样帧按以下顺序处理：

1. 按稳定 `observation_id` 分离每个轮廓，并把该轮廓内部转换成黑白二值图；已确认的真实背景穿孔从对应轮廓中扣除。
2. 取单个轮廓前景像素的最小包围框。
3. 将包围框内的完整二值轮廓平移到最小 `8` 倍数宽高的矩形画布中央。
4. 每个 `8x8` 块按行优先顺序压成一个小端 `uint64_t`，左上像素为 bit 0。
5. 保存为 `.b8x8`；`N = canvas_width / 8 * canvas_height / 8`，载荷正好是 `N` 个 `uint64_t`。

这个过程不缩放、不降采样，前景和真实背景穿孔均可逐位恢复。居中只消除轮廓在原画面中的整体平移；原图尺寸和原始包围框坐标仍保存在文件头中。

## 文件格式 v1

文件头固定为 88 字节：

```text
magic[8] = B8X8CNT\0
version:uint32 = 1
header_bytes:uint32 = 88
frame_id:uint64
contour_id:uint64            # 稳定 observation_id
source_width/source_height:uint32
bbox_x/bbox_y:int32
bbox_width/bbox_height:uint32
canvas_width/canvas_height:uint32
blocks_x/blocks_y/block_count:uint32
foreground_pixel_count:uint64
flags:uint32               # bit 0 表示已居中
payload:uint64[block_count]
```

所有整数按 little-endian 保存。画布补齐区域必须为 0；离线校验器会检查文件尺寸、块数、padding 位和前景 popcount。

## 相似度口径

连续帧和首个有效参考帧都按内容包围框重新居中到共同的最小 8 倍数画布，再计算：

- `similarity_percent`：全部二值位的 Hamming 一致率；
- `iou_percent`：前景交并比；
- `changed_pixels`：异或后的变化像素数；
- `foreground_pixels`：当前帧前景像素数。
- `center_shift_pixels`：两个原始 bbox 中心的欧氏位移；
- `position_similarity_percent`：`100 * (1 - center_shift_pixels / source_diagonal)`，下限截到 0。

Hamming 一致率包含黑色背景，轮廓较稀疏时可能偏高；判断形状稳定性应优先看前景 IoU，并结合变化像素数和包围框尺寸。

640x480 画面对角线为 800px，因此位置相似度严格 `>99%` 等价于 bbox 中心位移严格 `<8px`；位移刚好 8px 时等于 99%，仍判失败。

每个轮廓还写出：

- `bbox_area_percent`：bbox 面积占原画面的比例；
- `foreground_area_percent`：二值前景像素占原画面的比例；
- `mean_depth_mm / observed_depth_min_mm / observed_depth_max_mm`：当前稳定材料的深度摘要；
- `center_radius_percent`：bbox 中心到画面中心的距离，占画面半对角线的比例。

离线分析只对连续帧对执行 99% 门禁，并按当前帧特征分桶：面积 `<1% / 1%-5% / >=5%`，距离 `<=1500mm / 1500-2500mm / >2500mm / unknown`，中心径向位置 `<=33% / 33%-66% / >66%`。同时输出 `log10(foreground_pixels)`、有效 `mean_depth_mm`、`center_radius_percent` 与轮廓/位置相似度的 Pearson 和 Spearman 系数。分桶和相关性是固定回放内的描述，不证明因果。

## 运行

```powershell
.\x64\Release\D455.exe `
  --replay-dir=datasets\static_stability_locked_001 `
  --max-frames=120 `
  --no-display `
  --existence-contour-hole-filter `
  --binary-contour-stability `
  --binary-contour-output=analysis_runs\binary_contour_static_99_relation_001 `
  --binary-contour-warmup-frames=30 `
  --binary-contour-save-every-n=1

python .\scripts\analyze_binary_contour_stability.py `
  --input-dir=analysis_runs\binary_contour_static_99_relation_001
```

需要把 99% 目标失败转换成非零进程退出码时，增加 `--require-target-pass`。文件/指标复核失败返回 1；复核通过但 99% 目标失败返回 2。

主程序按 `frame_id + contour_id` 生成 `.b8x8`、轮廓级 `binary_contour_similarity.csv` 和帧级 `binary_contour_frames.csv`。离线脚本只在同一稳定 `contour_id` 内比较，并重新解析压缩位独立计算相似度，生成：

```text
binary_contour_similarity_recomputed.csv
binary_contour_stability_summary.json
binary_contour_stability_report.md
```

只有离线重算与 C++ CSV 一致、文件格式全部通过校验时，`validation_status` 才为 `pass`。该状态只说明测量链可信，不说明稳定目标达成；目标结论单独记录在 `continuous_frame_target.status`。

## 开关

- `--binary-contour-stability`：启用诊断，默认关闭。
- `--binary-contour-output=<dir>`：输出目录；指定后自动启用。
- `--binary-contour-warmup-frames=N`：忽略前 N 帧，默认 30。
- `--binary-contour-save-every-n=N`：每 N 帧采样一次，默认 1。
