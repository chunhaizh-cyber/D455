# Attention difference scan round 001

## Scope

This round implements the first measurable slice of
`资料/视觉注意力缓存差异扫描与并发ROI处理方案_v0.1.md` without changing the
pixel-cluster truth modes or near/far downgrade rules.

Implemented:

- opt-in low-resolution full-frame grayscale difference scan on every frame;
- phase-correlation translation diagnostics and a bounded cache-reuse gate;
- dirty-percent trigger and bounded multi-component motion ROIs;
- persistent fixed worker pool for independent color-contour ROI extraction;
- read-only worker input and deterministic merge in ROI input order;
- cache-version and result-age checks before async results are applied;
- explicit scan, cache, fallback, worker, merge, apply, and stale-result metrics;
- identical wiring in directory replay and live-camera processing paths.

Not implemented in this round:

- IMU or 3D reprojection of cached contours;
- per-pixel source labels for current measurement versus historical reuse;
- low-cost depth-anchor verification of unchanged color regions;
- local processing for the downstream depth clustering and final assignment stages;
- sliced full-frame refresh.

The feature is disabled by default. Enabling `--attention-difference-scan` also
enables the existing motion ROI, component clamp, and multi-component ROI path.
Full-frame processing remains the fallback for startup, unreliable alignment,
oversized dirty area, interval refresh, unknown spike, or far loss.

## Candidate

`configs/candidates/round_001/candidate_0048.json` is based on
`candidate_0030` and uses a 160-pixel scan width, 24-level grayscale threshold,
1% minimum dirty ratio, 25% maximum dirty ratio, four ROI workers, 5-pixel
translation tolerance, and a 15-frame async result age limit.

The 5-pixel translation tolerance is specific to this probe. A 2-pixel gate
misclassified synthetic local-object motion as camera motion in 36 scored
frames. Raising it to 5 allowed the ROI path to run, while slow camera pan still
caused 15 conservative full-refresh fallbacks. This is not a substitute for
IMU/3D cache reprojection.

## Validation

All cases used reviewed four-stream replay data, 120 frames, export every 30
frames, and ignored the first 30 warmup frames. Baseline values are from the
same build and replay command using `candidate_0030`.

| case | candidate | pass | score | p95 ms | max ms | over 100 ms | ROI tasks | cache reuse | full fallback |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| near_single_object | 0030 | yes | 96.397 | 50.664 | 69.461 | 0 | 0 | 0 | 0 |
| near_single_object | 0048 | yes | 95.902 | 53.961 | 74.465 | 0 | 7 | 88 | 2 |
| local_motion_roi_probe | 0030 | yes | 95.771 | 54.836 | 74.479 | 0 | 0 | 0 | 0 |
| local_motion_roi_probe | 0048 | yes | 95.597 | 56.000 | 71.359 | 0 | 37 | 90 | 0 |
| hand_occlusion_reappear_holdout | 0030 | yes | 95.195 | 58.683 | 82.953 | 0 | 0 | 0 | 0 |
| hand_occlusion_reappear_holdout | 0048 | yes | 95.436 | 57.070 | 84.272 | 0 | 16 | 90 | 0 |
| slow_pan_far_object | 0030 | yes | 94.515 | 63.218 | 90.872 | 0 | 0 | 0 | 0 |
| slow_pan_far_object | 0048 | yes | 93.469 | 70.194 | 108.834 | 1 | 13 | 75 | 15 |

For every final `candidate_0048` run:

- cluster coverage p50 was 100%;
- unknown p50 was 0%;
- contour-lost, merge, and split event counts were 0;
- stale async result count was 0;
- attention scan p95 was between 0.754ms and 0.895ms.

An earlier run of the same final candidate measured 52.170ms p95 on
`local_motion_roi_probe`; the final rerun measured 56.000ms. The ROI mechanism
is reproducible (37 tasks, 90 cache-reuse frames, zero stale results in both),
but the timing gain is not yet stable enough to claim a performance promotion.

Commands:

```powershell
msbuild .\D455.vcxproj /p:Configuration=Release /p:Platform=x64 /m
python -m py_compile scripts\convert_exports_to_analysis_run.py scripts\score_run.py scripts\select_winners.py
python scripts\run_batch.py --candidates configs\candidates\round_001 --cases eval\cases.yaml --candidate-id candidate_0030 --candidate-id candidate_0048 --case-id near_single_object --case-id local_motion_roi_probe --case-id hand_occlusion_reappear_holdout --case-id slow_pan_far_object --max-frames-override 120 --analysis-export-every-n 30 --ignore-first-n-frames 30 --validate-replay --require-replay-ir --require-reviewed-manifest --execute
```

## Decision

The mechanism is verified: unchanged frames reuse cache, independent local ROIs
run through the fixed pool, merge order is deterministic, fallback is visible,
and stale results are gated. The current replay evidence does not establish a
stable overall speed improvement.

Keep `candidate_0030` as the slow-pan bucket best. Keep `candidate_0048` as an
attention/ROI probe until camera-motion compensation and downstream local
processing remove the remaining full-frame work and slow-pan hitch.
