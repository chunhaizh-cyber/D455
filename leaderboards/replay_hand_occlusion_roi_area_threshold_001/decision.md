# Hand Occlusion ROI Area Threshold 001 Decision

Decision: keep `candidate_0030` as motion bucket best. Keep `candidate_0040` as the only automatic G3-pass hand-occlusion ROI probe. Keep `candidate_0046` as a smaller/faster warning probe, not a promotion.

The G3 review script now checks label-map pixel overlap from `final_segmentation_frame_*_ids.png` before falling back to bbox/center matching. This avoids treating label-id changes or region splits as drops when the baseline pixels are still assigned or a matching region exists.

| candidate | pass | score | p95 ms | max ms | ROI p95 | G3 |
|---|---:|---:|---:|---:|---:|---|
| candidate_0046 | true | 93.386 | 70.7440 | 94.391 | 182016.0 | warning |
| candidate_0040 | true | 92.886 | 74.0818 | 105.859 | 214828.8 | pass |
| candidate_0045 | true | 92.486 | 76.7488 | 96.478 | 182016.0 | warning |
| candidate_0044 | true | 92.434 | 77.0920 | 102.684 | 182016.0 | warning |
| candidate_0035 | true | 89.868 | 94.2102 | 122.810 | 303744.0 | baseline |

Interpretation:

- `candidate_0040` is safest by automatic G3: no missing baseline regions after pixel-overlap + geometry matching.
- `candidate_0046` is faster and smaller, with no stereo-bearing drop, but it still misses no-stereo fragments on frames 60 and 105.
- The next production step is not more area-threshold tuning. It should separate camera pan from local occlusion/reappear, or use source-aware ROI triggers.
