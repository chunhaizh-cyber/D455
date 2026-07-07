# G3 Review Decision

- case_id: `local_motion_roi_stereo_probe`
- baseline: `candidate_0034`
- filtered: `candidate_0035`
- review status: `synthetic_probe_pass`
- production status: `not_promoted`

## Evidence

The review package reports 4 runtime ROI stereo drops, totaling 22952 pixels. The final-segmentation comparison also reports 4 missing baseline regions, on frames 30, 45, 90, and 105. All four records refer to the same baseline fragment:

```text
bbox=[152,218,49,142]
pixel_count=5738
estimated_distance_mm=0
matched_stereo_points=0
stereo_evidence=0
```

Visual inspection of frame 30 compared:

- source: `datasets\local_motion_roi_stereo_probe\frames\000030_color.png`
- baseline overlay: `analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034\final_segmentation_frame_000030_overlay.png`
- filtered overlay: `analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035\final_segmentation_frame_000030_overlay.png`

The dropped fragment is a thin wall/background fragment to the left of the synthetic red/yellow target. The target contour itself remains present after filtering.

## Decision

G3 passes for this synthetic probe only: `candidate_0035` does not delete the generated target in the reviewed local-motion stereo case.

Do not promote `candidate_0035` to production. The drop policy still needs a real occlusion/reappear replay, because a real image-only ROI fragment might be meaningful even when it has no stereo evidence.
