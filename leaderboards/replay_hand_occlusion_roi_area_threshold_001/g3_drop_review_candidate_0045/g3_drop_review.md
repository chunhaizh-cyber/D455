# G3 ROI Drop Visual Review

- baseline_run: `analysis_runs\replay_hand_occlusion_roi_area_threshold_001\hand_occlusion_reappear_candidate_0035`
- filtered_run: `analysis_runs\replay_hand_occlusion_roi_area_threshold_001\hand_occlusion_reappear_candidate_0045`
- g3_status: `warning`
- g3_red_reasons: none
- g3_warning_reasons: dropped_no_stereo_region_count > 0
- runtime_roi_stereo_dropped_count: 0
- runtime_roi_stereo_dropped_pixels: 0
- contour_lost_event_count: 0
- missing_baseline_region_count: 2
- missing_baseline_region_pixels: 124697
- dropped_stereo_region_count: 0
- dropped_no_stereo_region_count: 2
- frames_with_drops: 60, 105
- review_status: needs_visual_review

G3 asks whether `--color-contour-refresh-roi-drop-stereo-failed` removes only unsafe fragments, not real occlusion/reappear target contours. Stereo-bearing dropped regions are a hard red signal; no-stereo drops and oversized runtime drops are warnings that require visual review.

| frame | region | pixels | bbox | stereo | overlap % | priority | source | baseline | filtered |
|---:|---:|---:|---|---:|---:|---|---|---|---|
| 60 | 2 | 27115 | 0,0,156,329 | 0 | 49.478 | fragment_review | [source](sample_frames/frame_000060_region_2_source_color.png) | [baseline](sample_frames/frame_000060_region_2_baseline_overlay.png) | [filtered](sample_frames/frame_000060_region_2_filtered_overlay.png) |
| 105 | 1 | 97582 | 0,0,411,327 | 0 |  | fragment_review | [source](datasets\hand_occlusion_reappear\frames\000105_color.png) | [baseline](analysis_runs\replay_hand_occlusion_roi_area_threshold_001\hand_occlusion_reappear_candidate_0035\final_segmentation_frame_000105_overlay.png) | [filtered](analysis_runs\replay_hand_occlusion_roi_area_threshold_001\hand_occlusion_reappear_candidate_0045\final_segmentation_frame_000105_overlay.png) |
