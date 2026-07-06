# Smoke P1/P2 Handoff

## P1 Single-Frame Smoke

Four live-camera smoke runs were executed locally:

- `baseline_default` x `near_single_object`
- `baseline_default` x `far_cabinet`
- `candidate_0001` x `near_single_object`
- `candidate_0001` x `far_cabinet`

All four converted and scored successfully, but all were `pass=false`. This is expected for the transitional smoke stage and must not be used to select a winner. `select_winners.py` correctly produced `no_pass_candidate` for every pareto slot.

Main observed fail reasons:

- `cluster_coverage_percent_p50 < 95`
- `unknown_percent_p50 > 15` in three of four runs
- `total_frame_ms_p95 > 100`

## P2 Multi-Frame Export Smoke

A short live-camera run used:

```powershell
.\x64\Release\D455.exe --cluster-map --quality-segmentation --color-segmentation --color-refine-depth-masks --stereo-contour-distance --max-frames=35 --no-display --analysis-export-every-n=10 --profile-csv=analysis_runs\smoke_live_multiframe_p2\profile.csv --cluster-map-export=analysis_runs\smoke_live_multiframe_p2\cluster_map --final-segmentation-export=analysis_runs\smoke_live_multiframe_p2\final_segmentation
```

Confirmed outputs:

- 4 cluster-map metadata frames: 0, 10, 20, 30
- 4 final-segmentation metadata frames: 0, 10, 20, 30
- 35 timing rows in `profile.csv`
- converter produced 4 `frame_metrics.csv` rows
- `score_run.py` scored the multi-frame package

Observed smoke score:

- `pass=false`
- `cluster_coverage_percent_p50=79.919`
- `unknown_percent_p50=20.081`
- `total_frame_ms_p95=728.3902`
- `far_stereo_failed_event_count=4`

## Interpretation

P2 removes the final-frame-only bridge limitation for smoke analysis. It is still not a deterministic benchmark because the input is live camera data, and the first sampled frame can include startup cost. Winner selection must still wait for deterministic replay and baseline regression gates.

## Next Step

Implement deterministic replay, preferably directory replay first:

```text
datasets/<case_id>/
  frames/
    000001_color.png
    000001_depth16.png
    000001_ir_left.png
    000001_ir_right.png
  case_manifest.json
```

Then `run_batch.py` can move from dry-run planning to real repeatable execution.
