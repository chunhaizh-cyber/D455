# ROI Component Clamp Decision

- run_id: `replay_hand_occlusion_roi_component_clamp_002`
- case_id: `hand_occlusion_reappear`
- production status: `not_promoted`
- current motion bucket: `candidate_0030`

## Result

`candidate_0036` proves that component-clamped ROI construction can make the ROI local:

```text
candidate_0035 roi_candidate_pixels_p95=303744
candidate_0036 roi_candidate_pixels_p95=72576
candidate_0036 roi_rejected_large_count=0
candidate_0036 roi_component_clamped_count=7
```

But it does not pass the safety gate:

```text
candidate_0036 G3 vs 0035 = red
dropped_stereo_region_count=4
runtime_roi_stereo_dropped_count=6
```

`candidate_0037` disables ROI fragment dropping and still fails G3:

```text
candidate_0037 G3 vs 0035 = red
dropped_stereo_region_count=3
runtime_roi_stereo_dropped_count=0
```

## Decision

Do not promote `candidate_0036` or `candidate_0037`.

The useful conclusion is causal: the oversized ROI came from sparse motion components being unioned into one large bbox, and component clamp fixes that part. The remaining failure is strategy-level: the current pipeline can process only one selected ROI, so distant motion/stereo-bearing regions can be skipped. The next candidate should implement true multi-ROI or sliced refresh, or explicitly preserve cache evidence for unselected components.

`candidate_0030` remains the real hand-occlusion motion bucket candidate.
