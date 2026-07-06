# Replay Small Gate Result

Input cases:

- `datasets/near_single_object`
- `datasets/far_cabinet`
- `datasets/depth_hole_black_object`

All three local replay inputs passed frame-pair, IR, PNG format, and case-contract manifest validation.

## Baseline Before FarBackground Fallback

Baseline/default failed all three cases:

- score: 65.0
- cluster coverage p50: 77.609 to 82.861
- unknown p50: 17.139 to 22.391
- p95 frame time: 264.9263 to 382.9582 ms
- hard fails: coverage, unknown, frame time

## FarBackground Fallback

Changing `unassigned_full_frame_remainder` from `Unknown` to low-confidence `FarBackground` fixed the coverage and unknown hard gates:

- score: 77.0
- cluster coverage p50: 100.0
- unknown p50: 0.0
- remaining hard fail: p95 frame time > 100 ms

## Candidate 0013

`candidate_0013` disables heavy color segmentation and stereo contour distance after the baseline args:

- `--no-color-segmentation`
- `--no-stereo-contour-distance`

Measured result:

| case_id | score | pass | coverage_p50 | unknown_p50 | frame_ms_p95 |
|---|---:|---|---:|---:|---:|
| near_single_object | 92.915 | true | 100.0 | 0.0 | 73.8853 |
| far_cabinet | 92.681 | true | 100.0 | 0.0 | 75.4504 |
| depth_hole_black_object | 92.398 | true | 100.0 | 0.0 | 77.3379 |

Boundary: this is a fast candidate, not the final far-distance solution. It trades stereo contour rough distance for deterministic replay throughput. The next improvement should recover far rough distance with sampling or a lower-frequency stereo path instead of re-enabling per-frame heavy stereo.

## Candidate 0014 And Far-Retention Scoring Fix

`candidate_0014` keeps color segmentation and stereo contour distance enabled, but adds:

- `--color-contour-frame-interval=120`

This refreshes color contour extraction and stereo contour distance on the first replay frame, then reuses cached `ColorContourRegion` data on the scored frames. It is a deterministic replay / low-motion bridge, not the final live-camera motion solution.

The scoring path was also corrected:

- converter now records `far_stereo_failed` only when final color regions exist and none has a valid stereo distance.
- `far_retention` now depends on retained `approx_stereo_contour_pixels + image_only_contour_pixels + depth_hole_candidate_pixels`, with extra credit when `stereo_matched_cluster_count` is present.
- this prevents a candidate from getting full far-retention credit merely by disabling color/stereo work.

Measured result after the scoring fix:

| case_id | candidate | score | pass | frame_ms_p95 | far_pixels_p50 | stereo_matched_p50 | far_retention |
|---|---|---:|---|---:|---:|---:|---:|
| near_single_object | candidate_0013 | 91.346 | true | 64.3404 | 22573 | 0 | 7.0 |
| near_single_object | candidate_0014 | 92.293 | true | 78.0335 | 23242 | 18 | 10.0 |
| far_cabinet | candidate_0013 | 91.006 | true | 66.6079 | 232497 | 0 | 7.0 |
| far_cabinet | candidate_0014 | 92.510 | true | 76.5890 | 231394 | 20 | 10.0 |
| depth_hole_black_object | candidate_0013 | 90.728 | true | 68.4671 | 68499 | 0 | 7.0 |
| depth_hole_black_object | candidate_0014 | 92.238 | true | 78.4038 | 67248 | 19 | 10.0 |

Winner selection over `analysis_runs/replay_small_gate_far_scoring_fix` ranks all three `candidate_0014` runs above `candidate_0013`, so the automatic loop now prefers the low-frequency cached rough-distance path over turning the feature off.

## Static Matrix 001

Run root: `analysis_runs/replay_small_matrix_static_001`

Compared:

- `baseline/default`
- `candidate_0001`
- `candidate_0002`
- `candidate_0013`
- `candidate_0014`

Cases:

- `near_single_object`
- `far_cabinet`
- `depth_hole_black_object`

Result:

- 6/15 passed hard gate.
- `baseline/default`, `candidate_0001`, and `candidate_0002` all failed `total_frame_ms_p95 > 100`.
- `candidate_0014` had the best average static score, but p95 was close to the 100 ms hard limit.
- `candidate_0013` remains the faster speed baseline, but has lower far-retention score because it lacks stereo matched clusters.

| candidate | pass | avg_score | avg_p95_ms | avg_far_retention |
|---|---:|---:|---:|---:|
| baseline_default | 0/3 | 89.000 | 342.980 | 10.0 |
| candidate_0001 | 0/3 | 89.000 | 358.328 | 10.0 |
| candidate_0002 | 0/3 | 89.000 | 329.679 | 10.0 |
| candidate_0013 | 3/3 | 90.093 | 72.697 | 7.0 |
| candidate_0014 | 3/3 | 90.232 | 91.782 | 10.0 |

Minimum leaderboard artifacts are committed under `leaderboards/replay_small_matrix_static_001/`; full analysis outputs remain local.

## Candidate 0015

`candidate_0015` extends `candidate_0014` with stale-cache refresh triggers:

- `--color-contour-refresh-on-motion`
- `--color-contour-refresh-on-unknown-spike`
- `--color-contour-refresh-on-far-loss`

New contracts in `eval/cases.yaml`:

- `slow_pan_far_object`
- `hand_occlusion_reappear`

Current status: implemented but not validated, because the motion-sensitive replay datasets do not exist yet. Do not promote `candidate_0015` until these cases are captured or otherwise generated and scored.

Static smoke with `analysis_runs/replay_small_gate_candidate_0015_static_smoke3`:

| case_id | score | pass | frame_ms_p95 | hard_fail |
|---|---:|---|---:|---|
| near_single_object | 89.000 | false | 101.2208 | `total_frame_ms_p95 > 100` |
| far_cabinet | 89.000 | false | 104.2593 | `total_frame_ms_p95 > 100` |
| depth_hole_black_object | 89.291 | true | 98.0575 | none |

Conclusion: keep the trigger implementation and candidate file, but do not promote `candidate_0015`. The static overhead is still too close to the hard gate; motion-sensitive replay must drive the next optimization.

## Refresh Metrics And Conservative Variants

D455 now appends color-contour cache diagnostics to `profile.csv`:

- `color_contour_refreshed`
- `color_contour_cache_reused`
- `color_contour_refresh_startup`
- `color_contour_refresh_interval`
- `color_contour_refresh_motion`
- `color_contour_refresh_unknown_spike`
- `color_contour_refresh_far_loss`
- `color_contour_motion_delta_percent`
- `color_contour_region_count`
- `color_contour_stereo_valid_count`
- `color_contour_stereo_reuse_count`

The converter carries these into `frame_metrics.csv`, writes `color_contour_refresh` rows into `events.csv`, and `score_run.py` summarizes refresh/cache/reuse counts in `run_score.json`.

New variants:

- `candidate_0016`: interval 180, motion threshold 35%, far-loss refresh only, unknown-spike refresh disabled.
- `candidate_0017`: interval 120, motion threshold 40%, far-loss refresh enabled, unknown-spike threshold 12%.

Static smoke with `analysis_runs/replay_static_refresh_variants_001`:

| candidate | case_id | score | pass | frame_ms_p95 | refresh_count | cache_reuse_count | motion | unknown_spike | far_loss | stereo_reuse_p50 |
|---|---|---:|---|---:|---:|---:|---:|---:|---:|---:|
| candidate_0016 | near_single_object | 89.100 | true | 99.3312 | 0 | 3 | 0 | 0 | 0 | 18 |
| candidate_0016 | far_cabinet | 89.192 | true | 98.7208 | 0 | 3 | 0 | 0 | 0 | 20 |
| candidate_0016 | depth_hole_black_object | 89.953 | true | 93.6439 | 0 | 3 | 0 | 0 | 0 | 19 |
| candidate_0017 | near_single_object | 91.109 | true | 85.9329 | 0 | 3 | 0 | 0 | 0 | 18 |
| candidate_0017 | far_cabinet | 92.367 | true | 77.5449 | 0 | 3 | 0 | 0 | 0 | 20 |
| candidate_0017 | depth_hole_black_object | 92.342 | true | 77.7103 | 0 | 3 | 0 | 0 | 0 | 19 |

Minimum leaderboard artifacts are committed under `leaderboards/replay_static_refresh_variants_001/`; full analysis outputs remain local. `candidate_0017` is the current static refresh-variant winner, but scored frames only reused cached color contours. This validates lower static overhead, not motion refresh correctness.

`configs/best/best_replay_static_refresh.json` now promotes `candidate_0017` only for the static refresh bucket. It does not replace `best_replay_static_far_distance.json`, which remains the earlier `candidate_0014` low-frequency far-distance baseline. The generated leaderboard schema now includes refresh/cache/reuse columns so the next motion gate can be judged directly from `leaderboard.csv`.

First real `slow_pan_far_object` replay smoke showed the expected trigger but failed the performance gate: `candidate_0015` and `candidate_0017` both recorded `color_contour_refresh_motion=1`, while refresh frames spent roughly 380-416 ms in `gray_prepare_ms`. `candidate_0018` capped region count but still matched huge templates. `candidate_0019` adds `--stereo-contour-max-roi-area-percent=12` and caps actual match attempts with `--stereo-contour-max-regions-per-frame=16`.

`candidate_0020` adds `--stereo-contour-reuse-cached-on-refresh`: after startup, color contour refresh updates masks but transfers cached stereo estimates by bbox IoU/center proximity instead of synchronously running stereo matchTemplate on the refresh frame. This is the first practical attempt to remove the 400 ms refresh-frame hitch while preserving rough far-distance evidence.

If 0020 still fails, the next likely bottleneck is color segmentation itself. `candidate_0021` disables mean-shift color filtering with `--color-segmentation-mean-shift-spatial=0` and `--color-segmentation-mean-shift-color=0` while keeping cached stereo reuse.

`--async-color-contour-refresh` is now the first multi-threaded refresh path. It keeps the main loop on the existing cache, runs color contour extraction on a background worker, then applies the latest completed result with cached stereo transfer. `candidate_0025` enables this path on top of `candidate_0017`; profile and leaderboard now expose async submitted/applied/dropped counts, cache age, and worker time.

## Slow Pan Motion Smoke 001

`datasets/slow_pan_far_object` was captured locally from a real slow camera pan: 180 color/depth16/left-IR/right-IR frames, finalized with `reviewed=true`. The review video is local at `recordings/slow_pan_far_object_color_20260706_1549.mp4`; raw dataset and video are not committed.

Run: `analysis_runs/replay_motion_slow_pan_001`, `--max-frames-override=180`, `--analysis-export-every-n=30`, `--ignore-first-n-frames=30`.

Result summary after switching performance p95 to the full `profile.csv` stream instead of export-sampled `frame_metrics.csv`:

| candidate | pass | profile p95 ms | max ms | >100ms frames | sampled p95 ms | async applied | far score | note |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| candidate_0013 | true | 54.2832 | 82.686 | 0 | 82.3498 | 0 | 7.0 | fastest, but no stereo contour distance |
| candidate_0024 | true | 69.3079 | 115.311 | 1 | 111.5720 | 0 | 10.0 | current slow-pan profile-p95 leader; still has a visible spike risk |
| candidate_0017 | true | 82.2889 | 506.911 | 6 | 502.4370 | 0 | 10.0 | motion trigger works, sync refresh creates large spikes |
| candidate_0020 | true | 86.2305 | 489.235 | 5 | 482.0038 | 0 | 10.0 | cached stereo transfer works, color extraction still spikes |
| candidate_0025 | true | 93.4440 | 119.407 | 3 | 119.1408 | 5 | 10.0 | first async path; valid but not better than 0024 yet |
| candidate_0015 | false | 426.8040 | 470.235 | 14 | 464.6214 | 0 | 10.0 | motion refresh baseline remains unusable |

Conclusion: motion refresh detection is real. The first async path removes the 400-500ms synchronous refresh stall, but background full-frame extraction still competes with the main loop and produces 3 over-budget frames on this replay. Do not promote `candidate_0025` as motion best yet. `candidate_0024` is the current slow-pan profile-p95 leader, but its 115ms max frame means the next implementation should target jitter directly with ROI-local refresh, worker throttling, or split-frame work rather than only improving p95.

Current blocker is narrowed: `datasets/slow_pan_far_object` now exists locally and has produced a first motion smoke; `datasets/hand_occlusion_reappear` is still missing, so the motion gate is not complete.
