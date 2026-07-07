# G3 ROI Drop Visual Review

- baseline_run: `analysis_runs\replay_hand_occlusion_reappear_proxy_001\hand_occlusion_reappear_proxy_candidate_0034`
- filtered_run: `analysis_runs\replay_hand_occlusion_reappear_proxy_001\hand_occlusion_reappear_proxy_candidate_0035`
- g3_status: `pass`
- g3_red_reasons: none
- g3_warning_reasons: none
- runtime_roi_stereo_dropped_count: 1
- runtime_roi_stereo_dropped_pixels: 9103
- contour_lost_event_count: 0
- missing_baseline_region_count: 0
- missing_baseline_region_pixels: 0
- dropped_stereo_region_count: 0
- dropped_no_stereo_region_count: 0
- frames_with_drops: none
- review_status: no_dropped_fragments

G3 asks whether `--color-contour-refresh-roi-drop-stereo-failed` removes only unsafe fragments, not real occlusion/reappear target contours. Stereo-bearing dropped regions are a hard red signal; no-stereo drops and oversized runtime drops are warnings that require visual review.

| frame | region | pixels | bbox | stereo | priority | source | baseline | filtered |
|---:|---:|---:|---|---:|---|---|---|---|
