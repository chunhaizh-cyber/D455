# G3 ROI Drop Visual Review

- baseline_run: `analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034`
- filtered_run: `analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035`
- runtime_roi_stereo_dropped_count: 4
- runtime_roi_stereo_dropped_pixels: 22952
- missing_baseline_region_count: 4
- missing_baseline_region_pixels: 22952
- dropped_stereo_region_count: 0
- dropped_no_stereo_region_count: 4
- frames_with_drops: 30, 45, 90, 105
- review_status: needs_visual_review

G3 asks whether `--color-contour-refresh-roi-drop-stereo-failed` removes only unsafe fragments, not real occlusion/reappear target contours. Stereo-bearing dropped regions are treated as high-priority target-risk evidence.

| frame | region | pixels | bbox | stereo | priority | source | baseline | filtered |
|---:|---:|---:|---|---:|---|---|---|---|
| 30 | 3 | 5738 | 152,218,49,142 | 0 | fragment_review | [source](datasets\local_motion_roi_stereo_probe\frames\000030_color.png) | [baseline](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034\final_segmentation_frame_000030_overlay.png) | [filtered](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035\final_segmentation_frame_000030_overlay.png) |
| 45 | 3 | 5738 | 152,218,49,142 | 0 | fragment_review | [source](datasets\local_motion_roi_stereo_probe\frames\000045_color.png) | [baseline](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034\final_segmentation_frame_000045_overlay.png) | [filtered](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035\final_segmentation_frame_000045_overlay.png) |
| 90 | 3 | 5738 | 152,218,49,142 | 0 | fragment_review | [source](datasets\local_motion_roi_stereo_probe\frames\000090_color.png) | [baseline](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034\final_segmentation_frame_000090_overlay.png) | [filtered](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035\final_segmentation_frame_000090_overlay.png) |
| 105 | 3 | 5738 | 152,218,49,142 | 0 | fragment_review | [source](datasets\local_motion_roi_stereo_probe\frames\000105_color.png) | [baseline](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034\final_segmentation_frame_000105_overlay.png) | [filtered](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035\final_segmentation_frame_000105_overlay.png) |
