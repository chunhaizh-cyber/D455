# Replay Capture P5 Handoff

Status: deterministic replay input can now be captured from a connected D455.

New command:

```powershell
.\x64\Release\D455.exe --capture-replay-dir=datasets\near_single_object --capture-replay-frames=120 --capture-replay-warmup=30 --no-display
```

The capture writes:

- `frames\000000_color.png`
- `frames\000000_depth16.png` as aligned millimeter `uint16`
- `frames\000000_ir_left.png`
- `frames\000000_ir_right.png` when right IR capture is enabled by the selected options
- `case_manifest.json`

Important boundary: this only creates the fixed replay input format. A captured case is not benchmark truth until `case_manifest.json` is reviewed, expected behavior is filled in, and the case is replayed through `run_batch.py --execute`.

Recommended next run:

```powershell
python scripts\run_batch.py --candidates configs\candidates\round_001 --cases eval\cases.yaml --analysis-export-every-n=30 --ignore-first-n-frames=30 --execute
```

