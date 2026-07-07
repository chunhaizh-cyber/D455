# Hand Occlusion ROI Multi Component 002 Decision

Run: `replay_hand_occlusion_roi_multi_component_002`

Decision: keep `candidate_0030` as the real hand-occlusion motion bucket best. Keep `candidate_0040` as a multi-ROI upper-bound probe only.

Key results:

| candidate | pass | score | p95 ms | max ms | >100ms | ROI p95 | G3 |
|---|---:|---:|---:|---:|---:|---:|---|
| candidate_0030 | true | 93.407 | 70.6062 | 88.089 | 0 | - | n/a |
| candidate_0040 | true | 92.987 | 73.4076 | 110.430 | 1 | 214828.8 | warning |
| candidate_0038 | true | 92.732 | 75.1086 | 101.176 | 1 | 87820.8 | red |
| candidate_0035 | true | 90.113 | 92.5756 | 117.130 | 5 | 303744.0 | baseline |

`candidate_0040` removes the hard G3 red condition seen in tighter multi-ROI probes:

```text
dropped_stereo_region_count=0
runtime_roi_stereo_dropped_count=0
```

It is not promoted because the automatic G3 result is still warning:

```text
missing_baseline_region_count=3
dropped_no_stereo_region_count=3
frames_with_drops=[15, 45, 75]
```

The implementation proves that true multi-component ROI wiring works and that the previous stereo-bearing drops were mainly a budget/coverage issue. The 70% ROI budget and 32-component cap are intentionally wide, so this result should guide a narrower follow-up rather than replace `candidate_0030`.

Next direction:

- Tune the multi-ROI budget between 35% and 70%.
- Split motion, far-loss, and unknown ROI sources before merging.
- Consider sliced multi-ROI refresh so wide coverage does not create >100ms spikes.
