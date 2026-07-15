# 32x32轮廓直接相似度与阈值测试

- 特征记录：13139
- 共同长期ID：1, 2, 3, 4, 5, 6, 7, 9
- 正样本：同ID连续帧或两段中同长期ID
- 负样本：同帧不同ID

| 范围 | 正样本 | 负样本 | 正样本IoU p05 / p50 / p95 | 负样本IoU p05 / p50 / p95 | AUC |
|---|---:|---:|---:|---:|---:|
| within_all:binary_contour_static_long_repeat_001 | 6573 | 35046 | 22.1982% / 91.0112% / 100.0000% | 4.0650% / 14.9425% / 47.5244% | 0.970452 |
| within_long:binary_contour_static_long_repeat_001 | 5121 | 20520 | 65.3061% / 91.8605% / 100.0000% | 3.7975% / 14.6976% / 62.5000% | 0.989920 |
| within_all:binary_contour_static_long_repeat_recheck_001 | 6494 | 34288 | 24.7450% / 90.4762% / 100.0000% | 3.9604% / 14.8148% / 50.0000% | 0.974101 |
| within_long:binary_contour_static_long_repeat_recheck_001 | 5121 | 20520 | 62.1622% / 90.6977% / 99.5037% | 3.3784% / 14.4638% / 62.6667% | 0.989750 |
| within_all:combined | 13067 | 69334 | 23.3645% / 90.6977% / 100.0000% | 4.0000% / 14.8810% / 48.9583% | 0.972306 |
| within_long:combined | 10242 | 41040 | 63.6408% / 91.3793% / 99.7519% | 3.5714% / 14.5754% / 62.6506% | 0.989862 |
| cross_shared_long:aligned_frame | 4560 | 31920 | 60.6061% / 87.5000% / 99.0099% | 3.2258% / 13.5468% / 42.8571% | 0.993753 |

## 自动阈值

| 范围 | 最佳平衡阈值 | TPR | FPR | 平衡准确率 | FPR<=1%阈值 | 对应TPR |
|---|---:|---:|---:|---:|---:|---:|
| within_all:binary_contour_static_long_repeat_001 | 45.7% | 89.8829% | 5.2588% | 92.3120% | 69.9000% | 80.2982% |
| within_long:binary_contour_static_long_repeat_001 | 65.3% | 95.0010% | 3.2066% | 95.8972% | 71.7000% | 90.4316% |
| within_all:binary_contour_static_long_repeat_recheck_001 | 45.7% | 91.7770% | 5.5180% | 93.1295% | 69.8000% | 81.3520% |
| within_long:binary_contour_static_long_repeat_recheck_001 | 64.7% | 94.2394% | 3.5039% | 95.3678% | 71.3000% | 90.4706% |
| within_all:combined | 45.7% | 90.8242% | 5.3870% | 92.7186% | 69.8000% | 80.8678% |
| within_long:combined | 64.7% | 94.6983% | 3.4942% | 95.6021% | 71.5000% | 90.4218% |
| cross_shared_long:aligned_frame | 47.2% | 98.7500% | 3.5871% | 97.5815% | 64.2000% | 92.4342% |

## 阈值迁移

阈值只在 `within_long:binary_contour_static_long_repeat_001` 上选择，再固定应用到其他范围。

| 范围 | 固定阈值 | TPR | FPR | 平衡准确率 |
|---|---:|---:|---:|---:|
| within_all:binary_contour_static_long_repeat_001 | 65.3% | 83.5844% | 1.8775% | 90.8534% |
| within_long:binary_contour_static_long_repeat_001 | 65.3% | 95.0010% | 3.2066% | 95.8972% |
| within_all:binary_contour_static_long_repeat_recheck_001 | 65.3% | 83.9852% | 1.9278% | 91.0287% |
| within_long:binary_contour_static_long_repeat_recheck_001 | 65.3% | 93.7122% | 3.2212% | 95.2455% |
| within_all:combined | 65.3% | 83.7836% | 1.9024% | 90.9406% |
| within_long:combined | 65.3% | 94.3566% | 3.2139% | 95.5713% |
| cross_shared_long:aligned_frame | 65.3% | 91.6447% | 0.5420% | 95.5514% |

## 主要重叠来源

| 类型 | 回放 | ID | p05 | p50 | p95 |
|---|---|---|---:|---:|---:|
| 同ID低相似 | binary_contour_static_long_repeat_recheck_001 | 8 | 17.3778% | 97.1429% | 100.0000% |
| 同ID低相似 | binary_contour_static_long_repeat_001 | 7 | 19.1781% | 90.4762% | 100.0000% |
| 同ID低相似 | binary_contour_static_long_repeat_001 | 9 | 20.8331% | 97.2222% | 100.0000% |
| 同ID低相似 | binary_contour_static_long_repeat_001 | 4 | 52.6882% | 82.8947% | 95.8099% |
| 同ID低相似 | binary_contour_static_long_repeat_recheck_001 | 7 | 53.9293% | 81.4815% | 95.8333% |
| 不同ID高相似 | binary_contour_static_long_repeat_001 | 9/10 | 30.0449% | 70.6667% | 75.0000% |
| 不同ID高相似 | binary_contour_static_long_repeat_recheck_001 | 8/9 | 27.2407% | 70.2703% | 75.0000% |
| 不同ID高相似 | binary_contour_static_long_repeat_recheck_001 | 5/9 | 56.4862% | 62.6667% | 68.0556% |
| 不同ID高相似 | binary_contour_static_long_repeat_001 | 5/10 | 56.3221% | 62.3377% | 67.6056% |
| 不同ID高相似 | binary_contour_static_long_repeat_recheck_001 | 5/8 | 31.2010% | 62.0253% | 66.2583% |

不同ID是观察标签，不是人工现实存在真值；接替ID可能属于同一现实对象。
自动阈值来自当前静态闭集数据，只是描述性候选，不能直接写入生产配置。
