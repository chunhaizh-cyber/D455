# 压缩轮廓区分存在能力测试

- 输入段数：2
- 轮廓帧记录：13139
- 压缩层级：8, 16, 32
- 身份测试最短 track：60 帧

## 不同ID精确值碰撞

| 回放 | 边长 | ID数 | 不同值 | 跨ID碰撞值 | 无歧义帧 | 单值最多ID |
|---|---:|---:|---:|---:|---:|---:|
| binary_contour_static_long_repeat_001 | 8 | 32 | 167 | 33 | 63.8456% | 8 |
| binary_contour_static_long_repeat_recheck_001 | 8 | 40 | 186 | 30 | 65.4423% | 15 |
| binary_contour_static_long_repeat_001 | 16 | 32 | 997 | 59 | 91.8244% | 5 |
| binary_contour_static_long_repeat_recheck_001 | 16 | 40 | 1023 | 65 | 93.0823% | 6 |
| binary_contour_static_long_repeat_001 | 32 | 32 | 3954 | 0 | 100.0000% | 1 |
| binary_contour_static_long_repeat_recheck_001 | 32 | 40 | 4053 | 0 | 100.0000% | 1 |

## 轮廓ID识别

每个候选 ID 用训练帧压缩位图的逐位多数值建立一个模板；测试帧按二值IoU与所有模板比较。只有正确模板严格高于所有其他模板才计为唯一正确，平分不算可区分。

| 范围 | 训练→测试 | 边长 | ID数 | 测试帧 | 唯一正确 | 含平分正确 | margin p05 / p50 |
|---|---|---:|---:|---:|---:|---:|---:|
| temporal_within_run | binary_contour_static_long_repeat_001 → binary_contour_static_long_repeat_001 | 8 | 14 | 3156 | 79.0875% | 86.1534% | -50.0000% / 50.0000% |
| temporal_within_run_long | binary_contour_static_long_repeat_001 → binary_contour_static_long_repeat_001 | 8 | 9 | 2565 | 97.3489% | 98.4405% | 26.1905% / 62.5000% |
| temporal_within_run | binary_contour_static_long_repeat_recheck_001 → binary_contour_static_long_repeat_recheck_001 | 8 | 13 | 3003 | 74.5588% | 89.9101% | -40.0000% / 33.3333% |
| temporal_within_run_long | binary_contour_static_long_repeat_recheck_001 → binary_contour_static_long_repeat_recheck_001 | 8 | 9 | 2565 | 97.9727% | 98.9084% | 26.1905% / 62.5000% |
| cross_run_shared_id | binary_contour_static_long_repeat_001 → binary_contour_static_long_repeat_recheck_001 | 8 | 8 | 4560 | 97.5877% | 98.0921% | 7.1429% / 62.5000% |
| cross_run_shared_id | binary_contour_static_long_repeat_recheck_001 → binary_contour_static_long_repeat_001 | 8 | 8 | 4560 | 87.0833% | 98.8816% | 0.0000% / 62.5000% |
| temporal_within_run | binary_contour_static_long_repeat_001 → binary_contour_static_long_repeat_001 | 16 | 14 | 3156 | 81.3054% | 82.2877% | -43.8095% / 35.8144% |
| temporal_within_run_long | binary_contour_static_long_repeat_001 → binary_contour_static_long_repeat_001 | 16 | 9 | 2565 | 97.4269% | 97.4269% | 5.5556% / 61.3900% |
| temporal_within_run | binary_contour_static_long_repeat_recheck_001 → binary_contour_static_long_repeat_recheck_001 | 16 | 13 | 3003 | 87.0463% | 87.0463% | -28.2553% / 54.1667% |
| temporal_within_run_long | binary_contour_static_long_repeat_recheck_001 → binary_contour_static_long_repeat_recheck_001 | 16 | 9 | 2565 | 98.8304% | 98.8304% | 12.5000% / 63.6364% |
| cross_run_shared_id | binary_contour_static_long_repeat_001 → binary_contour_static_long_repeat_recheck_001 | 16 | 8 | 4560 | 87.3684% | 87.3684% | -15.2778% / 64.7368% |
| cross_run_shared_id | binary_contour_static_long_repeat_recheck_001 → binary_contour_static_long_repeat_001 | 16 | 8 | 4560 | 87.1930% | 87.1930% | -1.4620% / 63.6364% |
| temporal_within_run | binary_contour_static_long_repeat_001 → binary_contour_static_long_repeat_001 | 32 | 14 | 3156 | 84.2839% | 84.3156% | -30.4162% / 36.1302% |
| temporal_within_run_long | binary_contour_static_long_repeat_001 → binary_contour_static_long_repeat_001 | 32 | 9 | 2565 | 98.8304% | 98.8304% | 19.4161% / 53.8636% |
| temporal_within_run | binary_contour_static_long_repeat_recheck_001 → binary_contour_static_long_repeat_recheck_001 | 32 | 13 | 3003 | 88.8778% | 88.8778% | -17.4463% / 46.3889% |
| temporal_within_run_long | binary_contour_static_long_repeat_recheck_001 → binary_contour_static_long_repeat_recheck_001 | 32 | 9 | 2565 | 99.2593% | 99.2593% | 19.4444% / 53.8636% |
| cross_run_shared_id | binary_contour_static_long_repeat_001 → binary_contour_static_long_repeat_recheck_001 | 32 | 8 | 4560 | 99.6930% | 99.6930% | 7.2368% / 62.1151% |
| cross_run_shared_id | binary_contour_static_long_repeat_recheck_001 → binary_contour_static_long_repeat_001 | 32 | 8 | 4560 | 99.1886% | 99.1886% | 7.1711% / 64.9761% |

精确值无碰撞只说明该样本中的位图值没有完全相同，不等于相似轮廓可可靠分类；身份识别结果才是本轮主要区分指标。
轮廓 ID 是观察身份，不是已经裁决的现实存在身份；接替 ID 之间的混淆可能表示 tracker 换号，而不是现实存在无法区分。
temporal_within_run_long 和跨回放测试只纳入覆盖至少 cross_run_min_percent 的长期 ID，用于减少接替标签污染。
