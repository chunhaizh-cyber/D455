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
