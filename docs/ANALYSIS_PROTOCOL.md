# D455 Cloud Analysis Protocol

This protocol fixes the directory shape and file contracts for cloud-side review of D455 runs. A run is identified by `run_id`, tied to a branch and commit, and contains enough metrics and sample frames to reproduce the operator-visible failure without searching through a full video first. Metric definitions, thresholds, and scoring rules are maintained in `docs/EVALUATION_FEATURES.md`.

## Directory Layout

```text
analysis_runs/
  <run_id>/
    run_manifest.json
    config_snapshot.json
    frame_metrics.csv
    cluster_metrics.jsonl
    events.csv
    notes.md
    debug_summary.md
    run_score.json
    run_summary.md
    failure_report.md
    sample_frames/
      frame_000120_rgb.jpg
      frame_000120_ir_left.png
      frame_000120_ir_right.png
      frame_000120_depth16.png
      frame_000120_cluster_id.png
      frame_000120_mode_map.png
      frame_000120_overlay.jpg
    videos/
      debug_overlay.mp4
      recording.mp4
```

The minimum useful package is:

```text
run_manifest.json
config_snapshot.json
frame_metrics.csv
cluster_metrics.jsonl
events.csv
notes.md
sample_frames/
```

Videos are useful context, but `frame_metrics.csv` and `cluster_metrics.jsonl` are the primary diagnostic data.

## Git Rules

Small text files stay in normal Git:

```text
run_manifest.json
config_snapshot.json
frame_metrics.csv
cluster_metrics.jsonl
events.csv
notes.md
debug_summary.md
run_score.json
run_summary.md
failure_report.md
```

Large or binary artifacts are routed through Git LFS by `.gitattributes`:

```text
*.mp4
*.avi
*.bag
*.raw
*.bin
*.depth
*.npy
analysis_runs/**/sample_frames/*.png
analysis_runs/**/videos/*
```

Do not repeatedly overwrite large files inside the same run. Prefer a new run id such as `<run_id>_v2` when rerunning an experiment.

## Run Index

Every run that is synchronized should be appended to `docs/RUN_INDEX.md`.

```markdown
| run_id | branch | commit | scene | purpose | result | notes |
|---|---|---|---|---|---|---|
| 20260703_153000_case01 | analysis/run-20260703-case01 | 3d9c7b6 | indoor_far_objects | P5 full-pixel clustering | far contour lost | frame 180 cabinet disappears |
```

When requesting analysis, provide `repo`, `branch`, `commit`, `run_id`, and the main observed problem.

## run_manifest.json

Required fields:

```json
{
  "run_id": "20260703_153000_case01",
  "case_name": "indoor_far_objects_full_pixel_cluster",
  "purpose": "P5 full-pixel clustering + P6 far stereo contour rough distance",
  "git_commit": "3d9c7b6",
  "branch": "analysis/run-20260703-case01",
  "build_type": "Release",
  "command_line": ".\\x64\\Release\\D455.exe --cluster-map --acceptance-baseline --max-frames=600",
  "frame_count": 600,
  "camera": {
    "model": "Intel RealSense D455",
    "serial": "",
    "fps": 30,
    "color_resolution": [640, 480],
    "depth_resolution": [640, 480],
    "ir_left_resolution": [640, 480],
    "ir_right_resolution": [640, 480]
  },
  "scene": {
    "tags": ["indoor", "near_object", "far_object", "wall_background"],
    "description": "Near desktop objects, far cabinet, wall background.",
    "expected": [
      "Near objects output PreciseDepth3D",
      "Far cabinet outputs ApproxStereoContour or ImageOnlyContour",
      "Wall outputs BackgroundPlane",
      "unknown_percent stays low"
    ]
  }
}
```

## config_snapshot.json

This is the exact effective configuration for the run. It should include the command-line options that matter, effective defaults after clamp/runtime mode, and any manual sliders changed during the run.

Minimum keys:

```json
{
  "min_depth_mm": 250,
  "max_depth_mm": 3500,
  "far_max_depth_mm": 12000,
  "depth_slice_mm": 450,
  "realtime_30": true,
  "quality_segmentation": true,
  "color_segmentation": true,
  "color_refine_depth_masks": true,
  "stereo_contour_distance": true,
  "non_precise_color_ownership_min_percent": 10,
  "feature_profile": "normal",
  "feature_heavy_interval": 15,
  "feature_cluster_metrics_interval": 1,
  "feature_dump_on_event": true,
  "feature_event_ring_frames": 10,
  "pcl_mode": "sampled",
  "pcl_run_interval": 2,
  "pcl_run_on_change": true,
  "far_stereo_interval": 3,
  "far_stereo_sparse_only": true,
  "notes": "Manual slider changes or non-default runtime context."
}
```

Feature collection policy and performance tradeoffs are documented in `docs/FEATURE_OPTIMIZATION.md`.

## Visual Evolution Context

Task-driven replay may attach optional governance metadata without passing those fields to `D455.exe`:

```json
{
  "requirement_id": "visual-need-hand-occlusion-local-refresh",
  "task_id": "task-hand-occlusion-local-refresh-001",
  "method_id": "method-candidate-0030",
  "method_version": "candidate_0030",
  "evaluation_split": "validation"
}
```

The same values are written to `run_manifest.json` as top-level fields and to `config_snapshot.json` under `visual_evolution_context`. They identify why a run was executed and which method version was evaluated; they do not turn a replay result into a world fact or a production promotion. `run_batch.py` keeps these options in the converter command and command plan, while the D455 command remains compatible with the executable's existing arguments.

## frame_metrics.csv

Required header:

```csv
frame_id,timestamp_ms,rgb_valid,depth_valid,ir_left_valid,ir_right_valid,total_frame_ms,capture_align_ms,depth_filter_ms,boundary_ms,near_cluster_ms,far_contour_ms,stereo_match_ms,merge_cluster_ms,tracker_ms,visualization_ms,recording_ms,total_pixels,clustered_pixels,unknown_pixels,assignment_coverage_percent,cluster_coverage_percent,unknown_percent,precise_depth3d_pixels,approx_stereo_contour_pixels,image_only_contour_pixels,background_plane_pixels,far_background_pixels,depth_hole_candidate_pixels,precise_depth3d_pixel_percent,approx_stereo_contour_pixel_percent,image_only_contour_pixel_percent,background_pixel_percent,depth_hole_candidate_pixel_percent,reliable_depth_pixels,unreliable_depth_pixels,depth_hole_pixels,total_cluster_count,foreground_cluster_count,background_cluster_count,unknown_cluster_count,near_cluster_count,far_cluster_count,far_distance_evidence_cluster_count,image_only_cluster_count,depth_hole_candidate_cluster_count,small_cluster_count,small_cluster_noise_ratio,cluster_fragmentation_index,candidate_count,anchor_supported_count,cue_input_count,cue_accepted_count,cue_rejected_texture_count,stable_track_count,new_track_count,lost_track_count,recovered_track_count,id_switch_estimate_count,mode_transition_count,mode_oscillation_count,depth_step_edge_px,depth_hole_edge_px,gray_edge_px,gray_depth_supported_edge_px,gray_depth_confirmed_edge_px,split_boundary_px,stereo_candidate_cluster_count,stereo_matched_cluster_count,stereo_failed_cluster_count,avg_contour_iou,boundary_jitter_px,unknown_spike,merge_event,split_event,far_stereo_failed_event,far_distance_missing_event,contour_lost_event,frame_time_over_budget
```

Red flags:

- `cluster_coverage_percent` stays low.
- `unknown_percent` spikes.
- `far_cluster_count` drops to zero during a visible far object.
- `stereo_failed_cluster_count` rises continuously.
- `near_cluster_count` jumps sharply.
- `lost_track_count` bursts.
- `total_frame_ms` exceeds the current processing budget.

If a producer cannot populate the full header yet, use the minimum landing set in `docs/EVALUATION_FEATURES.md` and leave missing columns for the next implementation slice.

## cluster_metrics.jsonl

One JSON object per cluster per frame. Use fixed mode names:

```text
PreciseDepth3D
ApproxStereoContour
ImageOnlyContour
BackgroundPlane
FarBackground
DepthHoleCandidate
Unknown
```

Every cluster should expose `mode`, `source`, geometry, contour confidence, applicable spatial evidence, and temporal fields. Use `null` for unavailable distance/depth values whose absence is meaningful; do not fill fake precision.

Near precise example:

```json
{"frame_id":180,"cluster_id":3,"track_id":7,"mode":"PreciseDepth3D","source":"depth_ir_pcl_anchor","pixel_count":18342,"bbox_2d":[100,80,160,220],"center_2d":[180.4,190.7],"depth_min_mm":1280,"depth_mean_mm":1420,"depth_max_mm":1610,"depth_valid_percent":95.08,"centroid_m":[0.12,-0.04,1.42],"min_point_m":[-0.08,-0.20,1.28],"max_point_m":[0.32,0.11,1.61],"anchor_count":42,"confidence":0.91,"stable_frames":18,"misses":0}
```

Far or image-only example:

```json
{"frame_id":180,"cluster_id":12,"track_id":31,"mode":"ApproxStereoContour","source":"ir_left_right_contour","pixel_count":6200,"bbox_2d":[420,110,80,140],"center_2d":[460.1,180.2],"contour_area_px":5980.0,"median_disparity_px":1.42,"disparity_mad_px":0.37,"matched_stereo_points":26,"estimated_distance_mm":5800,"distance_uncertainty_mm":1500,"contour_confidence":0.82,"distance_confidence":0.41,"stable_frames":6,"misses":0}
```

Minimum `cluster_metrics.jsonl` fields are defined in `docs/EVALUATION_FEATURES.md`.

## events.csv

Required header:

```csv
frame_id,event_type,severity,cluster_id,track_id,related_cluster_id,message,value_before,value_after
```

Use this file for failure landmarks such as `far_stereo_failed`, `far_distance_missing`, `track_lost`, `cluster_merge`, `unknown_spike`, `frame_time_spike`, and `depth_hole_spike`. `far_stereo_failed` describes one failed method; only `far_distance_missing` means that neither stereo nor depth-interval rough-distance evidence was available.

Recommended event types:

```text
unknown_spike
cluster_merge
cluster_split
track_created
track_lost
track_recovered
id_switch
mode_transition
mode_oscillation
far_stereo_failed
far_distance_missing
far_mode_downgraded
contour_lost
depth_hole_spike
sensor_drop
frame_time_spike
background_swallow_foreground
foreground_leak_to_background
```

## notes.md

Human observation is required. At minimum, record:

- Test purpose.
- Scene description.
- Expected result.
- Visible failure frames or time ranges.
- Rough ground truth distances when known.
- Any manual intervention, movement, or lighting change.

## Synchronization Message

After pushing a run, send:

```text
repo: https://github.com/<owner>/<repo>
branch: analysis/run-20260703-case01
commit: 8a21c9e
run_id: 20260703_153000_case01
focus: Far cabinet disappears after frame 180; near cup and box merge around frame 240.
```

## Analysis Outputs

Cloud-side or offline review should write these files back into the same run directory when available:

```text
run_score.json
run_summary.md
failure_report.md
```

`run_score.json` follows the score groups in `docs/EVALUATION_FEATURES.md`: pixel clustering, contour quality, spatial quality, temporal stability, far retention, performance, and diagnostic completeness.

## Transitional Export Converter

Until `D455.exe` writes the full standard package directly, use the converter to turn current export prototypes into scoring inputs:

```powershell
python scripts\convert_exports_to_analysis_run.py --run-dir=analysis_runs\<run_id> --cluster-map=analysis_runs\<run_id>\cluster_map_metadata.json --final-segmentation=analysis_runs\<run_id>\final_segmentation_metadata.json --profile-csv=analysis_runs\<run_id>\profile.csv
python scripts\score_run.py --runs=analysis_runs\<run_id> --weights=eval\score_weights.yaml
```

The converter writes the minimum landing set: `frame_metrics.csv`, `cluster_metrics.jsonl`, `events.csv`, and missing manifest/config/notes placeholders. Replace the generated `notes.md` with real scene observations before using the package for cloud review.
