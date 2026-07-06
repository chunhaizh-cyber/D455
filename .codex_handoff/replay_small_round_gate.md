# Replay Small Round Gate

Status: `run_batch.py` can now run bounded replay gates before the full 60-run grid.

Use this after the first fixed cases have been captured and reviewed:

```powershell
python scripts\validate_replay_dataset.py --case-dir datasets\near_single_object --case-dir datasets\far_cabinet --case-dir datasets\depth_hole_black_object --min-frames=120 --require-ir-left --require-ir-right --require-reviewed-manifest
python scripts\run_batch.py --candidates configs\baseline\default.json --cases eval\cases.yaml --case-id near_single_object --case-id far_cabinet --case-id depth_hole_black_object --max-frames-override=120 --analysis-export-every-n=30 --ignore-first-n-frames=30 --validate-replay --require-replay-ir --require-reviewed-manifest --execute
python scripts\run_batch.py --candidates configs\candidates\round_001 --candidate-id candidate_0001 --candidate-id candidate_0002 --cases eval\cases.yaml --case-id near_single_object --case-id far_cabinet --case-id depth_hole_black_object --max-frames-override=120 --analysis-export-every-n=30 --ignore-first-n-frames=30 --validate-replay --require-replay-ir --require-reviewed-manifest --execute
```

Useful switches:

- `--case-id <id>` can be repeated to select a fixed subset.
- `--candidate-id <id>` can be repeated when `--candidates` points at a directory.
- `--max-frames-override=120` keeps the first gate short even if `eval/cases.yaml` says 600.
- `--validate-replay` checks the replay directory before running D455.
- `--require-replay-ir` requires left and right IR frames for stereo contour candidates.
- `--require-reviewed-manifest` requires `reviewed: true`, reviewed scene notes, and non-empty expected fields.
- `--skip-missing-replay` is only for command-chain validation before capture is complete.

Boundary: skipped runs and missing replay inputs must not be counted as measured optimization evidence.
