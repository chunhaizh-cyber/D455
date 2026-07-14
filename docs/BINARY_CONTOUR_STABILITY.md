# 居中二值轮廓 8x8 打包与静态相似度测试

## 目标

本诊断用于测量同一静止场景连续经过主分割、稳定跟踪和存在内部穿孔过滤后，最终轮廓形状是否变化。输入直接使用 `ExistenceContourHoleResult.ownershipMask`，不重新运行一套独立边缘算法。

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

Hamming 一致率包含黑色背景，轮廓较稀疏时可能偏高；判断形状稳定性应优先看前景 IoU，并结合变化像素数和包围框尺寸。

## 运行

```powershell
.\x64\Release\D455.exe `
  --replay-dir=datasets\static_stability_locked_001 `
  --max-frames=120 `
  --no-display `
  --existence-contour-hole-filter `
  --binary-contour-stability `
  --binary-contour-output=analysis_runs\binary_contour_static_locked_001 `
  --binary-contour-warmup-frames=30 `
  --binary-contour-save-every-n=1

python .\scripts\analyze_binary_contour_stability.py `
  --input-dir=analysis_runs\binary_contour_static_locked_001
```

主程序按 `frame_id + contour_id` 生成 `.b8x8`、轮廓级 `binary_contour_similarity.csv` 和帧级 `binary_contour_frames.csv`。离线脚本只在同一稳定 `contour_id` 内比较，并重新解析压缩位独立计算相似度，生成：

```text
binary_contour_similarity_recomputed.csv
binary_contour_stability_summary.json
binary_contour_stability_report.md
```

只有离线重算与 C++ CSV 一致、文件格式全部通过校验时，`validation_status` 才为 `pass`。本轮只描述静态差异分布，不预设“轮廓稳定合格”阈值。

## 开关

- `--binary-contour-stability`：启用诊断，默认关闭。
- `--binary-contour-output=<dir>`：输出目录；指定后自动启用。
- `--binary-contour-warmup-frames=N`：忽略前 N 帧，默认 30。
- `--binary-contour-save-every-n=N`：每 N 帧采样一次，默认 1。
