# G3 ROI Drop Visual Review

- baseline_run: `analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0035`
- filtered_run: `analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0038`
- g3_status: `red`
- g3_red_reasons: dropped_stereo_region_count > 0
- g3_warning_reasons: dropped_no_stereo_region_count > 0
- runtime_roi_stereo_dropped_count: 0
- runtime_roi_stereo_dropped_pixels: 0
- contour_lost_event_count: 0
- missing_baseline_region_count: 10
- missing_baseline_region_pixels: 285359
- dropped_stereo_region_count: 3
- dropped_no_stereo_region_count: 7
- frames_with_drops: 15, 30, 45, 60, 75, 90
- review_status: target_risk_needs_visual_review

G3 asks whether `--color-contour-refresh-roi-drop-stereo-failed` removes only unsafe fragments, not real occlusion/reappear target contours. Stereo-bearing dropped regions are a hard red signal; no-stereo drops and oversized runtime drops are warnings that require visual review.

| frame | region | pixels | bbox | stereo | priority | source | baseline | filtered |
|---:|---:|---:|---|---:|---|---|---|---|
| 15 | 4 | 9479 | 0,0,125,185 | 0 | fragment_review | [source](datasets\hand_occlusion_reappear\frames\000015_color.png) | [baseline](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0035\final_segmentation_frame_000015_overlay.png) | [filtered](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0038\final_segmentation_frame_000015_overlay.png) |
| 30 | 2 | 56654 | 0,0,413,300 | 0 | fragment_review | [source](datasets\hand_occlusion_reappear\frames\000030_color.png) | [baseline](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0035\final_segmentation_frame_000030_overlay.png) | [filtered](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0038\final_segmentation_frame_000030_overlay.png) |
| 45 | 2 | 54001 | 0,0,414,325 | 0 | fragment_review | [source](datasets\hand_occlusion_reappear\frames\000045_color.png) | [baseline](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0035\final_segmentation_frame_000045_overlay.png) | [filtered](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0038\final_segmentation_frame_000045_overlay.png) |
| 45 | 4 | 4808 | 0,359,64,121 | 0 | fragment_review | [source](datasets\hand_occlusion_reappear\frames\000045_color.png) | [baseline](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0035\final_segmentation_frame_000045_overlay.png) | [filtered](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0038\final_segmentation_frame_000045_overlay.png) |
| 60 | 2 | 27115 | 0,0,156,329 | 0 | fragment_review | [source](datasets\hand_occlusion_reappear\frames\000060_color.png) | [baseline](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0035\final_segmentation_frame_000060_overlay.png) | [filtered](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0038\final_segmentation_frame_000060_overlay.png) |
| 60 | 4 | 26933 | 296,0,344,141 | 1 | high_target_risk | [source](sample_frames/frame_000060_region_4_source_color.png) | [baseline](sample_frames/frame_000060_region_4_baseline_overlay.png) | [filtered](sample_frames/frame_000060_region_4_filtered_overlay.png) |
| 75 | 1 | 43674 | 116,0,307,247 | 0 | fragment_review | [source](datasets\hand_occlusion_reappear\frames\000075_color.png) | [baseline](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0035\final_segmentation_frame_000075_overlay.png) | [filtered](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0038\final_segmentation_frame_000075_overlay.png) |
| 75 | 2 | 25503 | 302,0,338,141 | 1 | high_target_risk | [source](datasets\hand_occlusion_reappear\frames\000075_color.png) | [baseline](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0035\final_segmentation_frame_000075_overlay.png) | [filtered](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0038\final_segmentation_frame_000075_overlay.png) |
| 75 | 4 | 12400 | 0,0,143,189 | 0 | fragment_review | [source](datasets\hand_occlusion_reappear\frames\000075_color.png) | [baseline](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0035\final_segmentation_frame_000075_overlay.png) | [filtered](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0038\final_segmentation_frame_000075_overlay.png) |
| 90 | 3 | 24792 | 304,0,336,142 | 1 | high_target_risk | [source](datasets\hand_occlusion_reappear\frames\000090_color.png) | [baseline](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0035\final_segmentation_frame_000090_overlay.png) | [filtered](analysis_runs\replay_hand_occlusion_roi_multi_component_002\hand_occlusion_reappear_candidate_0038\final_segmentation_frame_000090_overlay.png) |
