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

