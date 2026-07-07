# G3 Review Decision

- case_id: `hand_occlusion_reappear`
- baseline: `candidate_0034`
- filtered: `candidate_0035`
- review status: `real_replay_pass_but_roi_not_local`
- production status: `not_promoted`

## Evidence

The real replay was captured locally into `datasets\hand_occlusion_reappear` with 120 color/depth16/left-IR/right-IR frames and finalized with `reviewed=true` by case contract.

Key leaderboard metrics:

```text
candidate_0030:
  score=93.584
  pass=true
  frame_ms_p95=69.4254
  frame_ms_max=88.561
  over_100ms_count=0
  contour_lost_events=0
  far_score=10.0

candidate_0035:
  score=89.994
  pass=true
  frame_ms_p95=93.3696
  frame_ms_max=105.051
  over_100ms_count=1
  contour_lost_events=0
  roi_rejected_large_count=7
  roi_candidate_pixels_p95=303744
  roi_motion_mask_pixels_p95=1668.3
  roi_motion_bbox_pixels_p95=296832.0
  roi_after_padding_pixels_p95=303744.0
  roi_max_pixels_p50=107520.0
  roi_stereo_dropped_count=0

candidate_0034:
  score=90.191
  pass=true
  frame_ms_p95=92.0580
  frame_ms_max=113.279
  over_100ms_count=4
  roi_rejected_large_count=7
```

The G3 report for candidate_0034 vs candidate_0035:

```text
g3_status=pass
runtime_roi_stereo_dropped_count=0
missing_baseline_region_count=0
dropped_stereo_region_count=0
dropped_no_stereo_region_count=0
contour_lost_event_count=0
```

## Decision

G3 red-line check passes on this real capture: `candidate_0035` did not drop stereo-bearing or no-stereo baseline regions, and no contour-lost event was recorded.

Do not promote `candidate_0035`. On the real hand-occlusion capture, the ROI path did not become local: all seven ROI candidates were rejected as too large. The new source diagnostics show that the motion mask itself is small, but sparse motion pixels are merged into one large bbox before padding. This points to ROI construction quality, not drop safety, as the next bottleneck. `candidate_0030` remains the better motion bucket candidate for this replay.
