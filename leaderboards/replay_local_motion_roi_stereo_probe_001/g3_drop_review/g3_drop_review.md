# G3 ROI Drop Visual Review

- baseline_run: `analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034`
- filtered_run: `analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035`
- g3_status: `warning`
- g3_red_reasons: none
- g3_warning_reasons: dropped_no_stereo_region_count > 0, runtime_roi_stereo_dropped_pixels > 20000
- runtime_roi_stereo_dropped_count: 4
- runtime_roi_stereo_dropped_pixels: 22952
- contour_lost_event_count: 0
- missing_baseline_region_count: 4
- missing_baseline_region_pixels: 22952
- dropped_stereo_region_count: 0
- dropped_no_stereo_region_count: 4
- frames_with_drops: 30, 45, 90, 105
- review_status: needs_visual_review

G3 asks whether `--color-contour-refresh-roi-drop-stereo-failed` removes only unsafe fragments, not real occlusion/reappear target contours. Stereo-bearing dropped regions are a hard red signal; no-stereo drops and oversized runtime drops are warnings that require visual review.

| frame | region | pixels | bbox | stereo | priority | source | baseline | filtered |
|---:|---:|---:|---|---:|---|---|---|---|
| 30 | 3 | 5738 | 152,218,49,142 | 0 | fragment_review | [source](sample_frames/frame_000030_region_3_source_color.png) | [baseline](sample_frames/frame_000030_region_3_baseline_overlay.png) | [filtered](sample_frames/frame_000030_region_3_filtered_overlay.png) |
| 45 | 3 | 5738 | 152,218,49,142 | 0 | fragment_review | [source](datasets\local_motion_roi_stereo_probe\frames\000045_color.png) | [baseline](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034\final_segmentation_frame_000045_overlay.png) | [filtered](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035\final_segmentation_frame_000045_overlay.png) |
| 90 | 3 | 5738 | 152,218,49,142 | 0 | fragment_review | [source](datasets\local_motion_roi_stereo_probe\frames\000090_color.png) | [baseline](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034\final_segmentation_frame_000090_overlay.png) | [filtered](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035\final_segmentation_frame_000090_overlay.png) |
| 105 | 3 | 5738 | 152,218,49,142 | 0 | fragment_review | [source](datasets\local_motion_roi_stereo_probe\frames\000105_color.png) | [baseline](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034\final_segmentation_frame_000105_overlay.png) | [filtered](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035\final_segmentation_frame_000105_overlay.png) |
