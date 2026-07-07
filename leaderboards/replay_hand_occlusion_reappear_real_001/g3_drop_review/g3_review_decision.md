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
  score=92.562
  pass=true
  frame_ms_p95=76.2424
  frame_ms_max=85.739
  contour_lost_events=0
  far_score=10.0

candidate_0035:
  score=89.184
  pass=true
  frame_ms_p95=98.773
  frame_ms_max=132.654
  over_100ms_count=5
  contour_lost_events=0
  roi_rejected_large_count=7
  roi_candidate_pixels_p95=303744
  roi_stereo_dropped_count=0

candidate_0034:
  score=89.000
  pass=false
  frame_ms_p95=101.6394
  hard_fail=total_frame_ms_p95 > 100
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

Do not promote `candidate_0035`. On the real hand-occlusion capture, the ROI path did not become local: all seven ROI candidates were rejected as too large, so candidate_0035 mostly exercised full-frame refresh behavior and ran close to the 100ms hard budget. `candidate_0030` remains the better motion bucket candidate for this replay.
