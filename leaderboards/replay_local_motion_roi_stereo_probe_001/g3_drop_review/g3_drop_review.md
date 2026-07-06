# G3 ROI Drop Visual Review

- baseline_run: `analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034`
- filtered_run: `analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035`
- dropped_fragment_count: 4
- dropped_fragment_pixels: 22952
- frames_with_drops: 30, 45, 90, 105
- review_status: needs_visual_review

G3 asks whether `--color-contour-refresh-roi-drop-stereo-failed` removes only no-stereo fragments, not real occlusion/reappear target contours.

| frame | region | pixels | bbox | source | baseline | filtered |
|---:|---:|---:|---|---|---|---|
| 30 | 3 | 5738 | 152,218,49,142 | [source](datasets\local_motion_roi_stereo_probe\frames\000030_color.png) | [baseline](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034\final_segmentation_frame_000030_overlay.png) | [filtered](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035\final_segmentation_frame_000030_overlay.png) |
| 45 | 3 | 5738 | 152,218,49,142 | [source](datasets\local_motion_roi_stereo_probe\frames\000045_color.png) | [baseline](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034\final_segmentation_frame_000045_overlay.png) | [filtered](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035\final_segmentation_frame_000045_overlay.png) |
| 90 | 3 | 5738 | 152,218,49,142 | [source](datasets\local_motion_roi_stereo_probe\frames\000090_color.png) | [baseline](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034\final_segmentation_frame_000090_overlay.png) | [filtered](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035\final_segmentation_frame_000090_overlay.png) |
| 105 | 3 | 5738 | 152,218,49,142 | [source](datasets\local_motion_roi_stereo_probe\frames\000105_color.png) | [baseline](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0034\final_segmentation_frame_000105_overlay.png) | [filtered](analysis_runs\replay_local_motion_roi_stereo_probe_001\local_motion_roi_stereo_probe_candidate_0035\final_segmentation_frame_000105_overlay.png) |
