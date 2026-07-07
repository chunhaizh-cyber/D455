# G3 ROI Drop Visual Review

- baseline_run: `analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0035`
- filtered_run: `analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0040`
- g3_status: `warning`
- g3_red_reasons: none
- g3_warning_reasons: dropped_no_stereo_region_count > 0
- runtime_roi_stereo_dropped_count: 0
- runtime_roi_stereo_dropped_pixels: 0
- contour_lost_event_count: 0
- missing_baseline_region_count: 3
- missing_baseline_region_pixels: 26687
- dropped_stereo_region_count: 0
- dropped_no_stereo_region_count: 3
- frames_with_drops: 15, 45, 75
- review_status: needs_visual_review

G3 asks whether `--color-contour-refresh-roi-drop-stereo-failed` removes only unsafe fragments, not real occlusion/reappear target contours. Stereo-bearing dropped regions are a hard red signal; no-stereo drops and oversized runtime drops are warnings that require visual review.

| frame | region | pixels | bbox | stereo | priority | source | baseline | filtered |
|---:|---:|---:|---|---:|---|---|---|---|
| 15 | 4 | 9479 | 0,0,125,185 | 0 | fragment_review | [source](sample_frames/frame_000015_region_4_source_color.png) | [baseline](sample_frames/frame_000015_region_4_baseline_overlay.png) | [filtered](sample_frames/frame_000015_region_4_filtered_overlay.png) |
| 45 | 4 | 4808 | 0,359,64,121 | 0 | fragment_review | [source](datasets\hand_occlusion_reappear\frames\000045_color.png) | [baseline](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0035\final_segmentation_frame_000045_overlay.png) | [filtered](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0040\final_segmentation_frame_000045_overlay.png) |
| 75 | 4 | 12400 | 0,0,143,189 | 0 | fragment_review | [source](datasets\hand_occlusion_reappear\frames\000075_color.png) | [baseline](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0035\final_segmentation_frame_000075_overlay.png) | [filtered](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0040\final_segmentation_frame_000075_overlay.png) |
