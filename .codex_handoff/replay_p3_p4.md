# P3/P4 Replay Execution Handoff

## Implemented

- D455 now supports deterministic directory replay via `--replay-dir=<case_dir>`.
- Replay frame layout:

```text
datasets/<case_id>/
  frames/
    000000_color.png
    000000_depth16.png
    000000_ir_left.png
    000000_ir_right.png
  case_manifest.json
```

- `depth16.png` is read as unsigned 16-bit millimeter depth with `depth_scale=0.001`.
- `ir_left` is used as the replay edge source when present; otherwise replay falls back to grayscale color.
- `ir_right` is optional, but stereo contour rough distance needs both IR frames.
- The first replay implementation is scoped to the non-display evaluation path: profile CSV, multi-frame analysis export, converter, and scoring.

## Batch Execution

`scripts/run_batch.py` now supports:

```powershell
python scripts\run_batch.py --candidates configs\candidates\round_001 --cases eval\cases.yaml --analysis-export-every-n=30 --ignore-first-n-frames=30 --execute
```

The execute chain is:

1. Run D455 with `--replay-dir=<case replay path>`.
2. Convert exports with `scripts\convert_exports_to_analysis_run.py`.
3. Score the run with `scripts\score_run.py`.

`--analysis-root` can redirect generated analysis packages away from the default `analysis_runs`.

## Warmup Filtering

`scripts/convert_exports_to_analysis_run.py` now supports:

```powershell
--ignore-first-n-frames=30
```

This drops early replay frames before writing `frame_metrics.csv`, `cluster_metrics.jsonl`, and `events.csv`, preventing frame 0 startup/tracker warmup from polluting p95 and unknown metrics.

## Local Validation

Validation used a temporary synthetic replay case under `%TEMP%`, not repository `datasets/`.

Confirmed:

- `D455.exe --replay-dir=<temp_case>` runs without a connected camera dependency for replay.
- Multi-frame cluster/final metadata is exported.
- Converter creates multi-row `frame_metrics.csv`.
- `score_run.py` writes `run_score.json`.
- `run_batch.py --execute` runs D455, converter, and scorer in order.
- `--ignore-first-n-frames=1` reduced a 3-frame synthetic replay package to 2 metric rows.

## Remaining Blocker

Real fixed evaluation cases still need to be captured under `datasets/`. Until then, measured optimization remains structurally ready but data-empty.
