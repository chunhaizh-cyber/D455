# Replay Capture P5 Handoff

Status: deterministic replay input can now be captured from a connected D455.

Recommended capture command:

```powershell
.\tools\Capture-ReplayCases.ps1 -Frames 120 -Warmup 30
```

The helper captures `near_single_object`, `far_cabinet`, and `depth_hole_black_object` by default. It pauses before each case so the scene can be arranged, passes `--quality-segmentation --stereo-contour-distance` to capture both IR streams, and runs `scripts\validate_replay_dataset.py` after each capture.

The capture writes:

- `frames\000000_color.png`
- `frames\000000_depth16.png` as aligned millimeter `uint16`
- `frames\000000_ir_left.png`
- `frames\000000_ir_right.png` when right IR capture is enabled by the selected options
- `case_manifest.json`

Important boundary: this only creates the fixed replay input format. A captured case is not benchmark truth until `case_manifest.json` has `reviewed: true`, non-placeholder `notes`, a non-empty `expected` object, and the case is replayed through `run_batch.py --execute`.

Recommended next run:

```powershell
python scripts\validate_replay_dataset.py --case-dir datasets\near_single_object --case-dir datasets\far_cabinet --case-dir datasets\depth_hole_black_object --min-frames=120 --require-ir-left --require-ir-right --require-reviewed-manifest
python scripts\run_batch.py --candidates configs\baseline\default.json --cases eval\cases.yaml --case-id near_single_object --case-id far_cabinet --case-id depth_hole_black_object --max-frames-override=120 --analysis-export-every-n=30 --ignore-first-n-frames=30 --validate-replay --require-replay-ir --require-reviewed-manifest --execute
```
