# G3 Review Decision

- case_id: `hand_occlusion_reappear_proxy`
- baseline: `candidate_0034`
- filtered: `candidate_0035`
- review status: `synthetic_proxy_pass`
- production status: `not_promoted`

## Evidence

The proxy case was generated from `datasets\near_single_object` into a local analysis directory, then replayed for `candidate_0034` and `candidate_0035`.

Key leaderboard metrics:

```text
candidate_0035:
  score=93.670
  pass=true
  frame_ms_p95=68.8496
  frame_ms_max=77.678
  contour_lost_events=0
  roi_stereo_built_count=10
  roi_stereo_failed_count=0
  roi_stereo_dropped_count=1
  roi_stereo_dropped_pixels=9103

candidate_0034:
  score=93.637
  pass=true
  frame_ms_p95=69.0688
  frame_ms_max=84.297
  contour_lost_events=0
  roi_stereo_built_count=10
  roi_stereo_failed_count=1
```

The upgraded G3 report separates runtime drop counters from final-segmentation comparison:

```text
runtime_roi_stereo_dropped_count=1
runtime_roi_stereo_dropped_pixels=9103
missing_baseline_region_count=0
dropped_stereo_region_count=0
dropped_no_stereo_region_count=0
```

This means `candidate_0035` did execute the drop path once, but no baseline color region disappeared from the final segmentation comparison. There is no evidence in this synthetic proxy that a stereo-bearing target was removed.

## Decision

G3 passes for this synthetic occlusion/reappear proxy only. It strengthens the evidence that `candidate_0035` can drop a stereo-failed ROI fragment without losing the generated target in a controlled occlusion/reappear sequence.

Do not promote `candidate_0035` to production. The real `datasets\hand_occlusion_reappear` capture is still required because this proxy is generated, not a real camera hand-occlusion scene.
