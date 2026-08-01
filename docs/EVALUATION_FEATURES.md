# D455 Engineering Evaluation Feature Set v0.1

This document defines measurable engineering features for D455 segmentation runs. The goal is not whether the output looks good at a glance, but whether the run proves:

- Full-frame pixel clustering.
- Near-field clusters return reliable high-precision 3D information.
- Far-field clusters keep accurate 2D contours and return rough distance when confidence allows.
- Objects outside the D455 high-precision depth range are not dropped.
- Missing depth is not confused with far distance; it may remain `ImageOnlyContour` or `DepthHoleCandidate`.

## Evaluation Unit

Each evaluation uses one `run_id`.

Required inputs:

```text
run_manifest.json
config_snapshot.json
frame_metrics.csv
cluster_metrics.jsonl
events.csv
sample_frames/
```

Optional inputs:

```text
videos/debug_overlay.mp4
videos/recording.mp4
```

Expected analysis outputs:

```text
run_score.json
run_summary.md
failure_report.md
```

## Feature Groups

| Group | Name | Main File |
|---|---|---|
| A | Run Features | `run_manifest.json`, `config_snapshot.json` |
| B | Frame Global Features | `frame_metrics.csv` |
| C | Pixel Clustering Features | `frame_metrics.csv` |
| D | Cluster Common Features | `cluster_metrics.jsonl` |
| E | Near Precise 3D Features | `cluster_metrics.jsonl` |
| F | Far Contour Stereo Features | `cluster_metrics.jsonl` |
| G | Contour Quality Features | `cluster_metrics.jsonl`, optional labels |
| H | Temporal Tracking Features | `frame_metrics.csv`, `cluster_metrics.jsonl` |
| I | Performance / Event Features | `frame_metrics.csv`, `events.csv` |

Recommended total score:

```text
TotalScore =
  20% pixel clustering
+ 20% contour quality
+ 20% spatial information quality
+ 15% temporal stability
+ 10% far-field retention
+ 10% performance
+  5% diagnostic completeness
```

## A. Run Features

Run features are not directly scored, but the run is not reproducible without them.

| ID | Field | Type | Purpose |
|---|---|---|---|
| A001 | `run_id` | string | Unique run id |
| A002 | `git_commit` | string | Code version |
| A003 | `config_hash` | string | Hash of effective `config_snapshot.json` |
| A004 | `scene_tags` | string array | Scene categories for comparison |
| A005 | `camera_profile` | object | Camera model, serial, fps, resolutions |
| A006 | `evaluation_mode` | enum | `live_camera`, `replay_recording`, `offline_dataset` |

## B. Frame Global Features

Frame-level fields are written to `frame_metrics.csv`.

| ID | Field | Meaning |
|---|---|---|
| B001 | `frame_id` | Frame sequence id |
| B002 | `timestamp_ms` | Time from run start |
| B003 | `rgb_valid`, `depth_valid`, `ir_left_valid`, `ir_right_valid` | Sensor input validity flags |
| B004 | `total_pixels` | Frame width * height |
| B005 | `total_cluster_count` | All clusters including background and unknown |
| B006 | `foreground_cluster_count` | `PreciseDepth3D`, `ApproxStereoContour`, `ImageOnlyContour`, `DepthHoleCandidate` |
| B007 | `background_cluster_count` | `BackgroundPlane`, `FarBackground` |
| B008 | `unknown_cluster_count` | Unknown cluster count |

## C. Pixel Clustering Features

Pixel clustering is the first core project goal.

| ID | Field | Formula / Meaning | Direction |
|---|---|---|---|
| C000 | `assignment_coverage_percent` | `100 * assigned_pixels / total_pixels`, including `Unknown` | Should be near 100 for full-frame ownership |
| C001 | `clustered_pixels` | Pixels assigned to non-Unknown clusters | Higher |
| C002 | `unknown_pixels` | Pixels assigned to Unknown | Lower |
| C003 | `cluster_coverage_percent` | `100 * (total_pixels - unknown_pixels) / total_pixels` | Higher |
| C004 | `unknown_percent` | `100 * unknown_pixels / total_pixels` | Lower |
| C005 | `precise_depth3d_pixel_percent` | `100 * precise_depth3d_pixels / total_pixels` | Contextual |
| C006 | `approx_stereo_contour_pixel_percent` | `100 * approx_stereo_contour_pixels / total_pixels` | Contextual |
| C007 | `image_only_contour_pixel_percent` | `100 * image_only_contour_pixels / total_pixels` | Contextual |
| C008 | `background_pixel_percent` | Background plane + far background pixels | Contextual |
| C009 | `depth_hole_candidate_pixel_percent` | Depth holes with visual object evidence | Contextual |
| C010 | `small_cluster_noise_ratio` | Tiny-cluster pixels / total pixels | Lower |
| C011 | `cluster_fragmentation_index` | Small-cluster count / total cluster count when no truth exists | Moderate |

Thresholds:

| Field | Excellent | Usable | Watch | High Risk |
|---|---:|---:|---:|---:|
| `cluster_coverage_percent` | `>= 99%` | `>= 98%` | `95-98%` | `< 95%` |
| `unknown_percent` | `<= 2%` | `<= 5%` | `5-10%` | `> 10%`, severe at `> 20%` |

## D. Cluster Common Features

Every line in `cluster_metrics.jsonl` should include these common fields.

| ID | Field | Type / Formula |
|---|---|---|
| D001 | `cluster_id` | uint64 |
| D002 | `track_id` | uint64 or null |
| D003 | `mode` | fixed spatial mode enum |
| D004 | `source` | evidence source enum |
| D005 | `pixel_count` | cluster pixels |
| D006 | `area_percent` | `100 * pixel_count / total_pixels` |
| D007 | `bbox_2d` | `[x, y, width, height]` |
| D008 | `center_2d` | `[cx, cy]` |
| D009 | `contour_area_px` | OpenCV contour area |
| D010 | `perimeter_px` | contour perimeter |
| D011 | `compactness` | `4 * pi * area / perimeter^2` |
| D012 | `bbox_fill_ratio` | `contour_area_px / bbox_area` |
| D013 | `aspect_ratio` | `bbox_width / max(1, bbox_height)` |
| D014 | `contour_confidence` | 0-1 confidence |

Fixed `mode` values:

```text
PreciseDepth3D
ApproxStereoContour
ImageOnlyContour
BackgroundPlane
FarBackground
DepthHoleCandidate
Unknown
```

Recommended `source` values:

```text
depth_ir_pcl_anchor
depth_ir_boundary
rgb_ir_visual_contour
ir_left_right_contour
rgb_only_contour
indoor_plane_depth_normal
depth_hole_visual_candidate
unknown_fill
```

Recommended contour confidence:

```text
contour_confidence =
  0.30 * edge_alignment_score
+ 0.25 * temporal_iou_score
+ 0.20 * area_stability_score
+ 0.15 * closure_score
+ 0.10 * size_reasonable_score
```

## E. Near Precise 3D Features

Only for `mode = PreciseDepth3D`.

| ID | Field | Meaning |
|---|---|---|
| E001 | `depth_valid_pixel_count` | Valid depth pixels inside cluster |
| E002 | `depth_valid_percent` | `100 * depth_valid_pixel_count / pixel_count` |
| E003-E006 | `depth_min_mm`, `depth_mean_mm`, `depth_median_mm`, `depth_max_mm` | Depth statistics |
| E007-E009 | `depth_std_mm`, `depth_iqr_mm`, `depth_outlier_percent` | Depth noise and outliers |
| E010-E013 | `centroid_3d_m`, `bounds_3d_min_m`, `bounds_3d_max_m`, `size_3d_m` | 3D shape |
| E014 | `point_density` | `1000 * valid_3d_point_count / pixel_count` |
| E015-E016 | `pcl_cluster_id`, `pcl_vote_ratio` | PCL support |
| E017-E019 | `anchor_count`, `anchor_density`, `anchor_support_percent` | Stable anchor support |
| E020 | `near_3d_confidence` | 0-1 near-field 3D confidence |
| E021-E022 | `distance_error_mm`, `relative_distance_error_percent` | Only when truth exists |

Near-depth validity thresholds:

```text
depth_valid_percent >= 90%: excellent
75% - 90%: usable
50% - 75%: watch
< 50%: should not be treated as high-precision 3D
```

Recommended near 3D confidence:

```text
near_3d_confidence =
  0.25 * depth_valid_score
+ 0.20 * depth_outlier_score
+ 0.20 * anchor_score
+ 0.15 * pcl_vote_score
+ 0.10 * temporal_depth_score
+ 0.10 * contour_confidence
```

## F. Far Contour Stereo Features

For `mode = ApproxStereoContour` and `mode = ImageOnlyContour`.

Far-field priority:

```text
contour retained > stable 2D position and size > rough distance > precise depth
```

| ID | Field | Meaning |
|---|---|---|
| F001 | `far_contour_retained` | Far object contour retained, 0/1 |
| F002 | `stereo_candidate` | Stereo matching attempted, 0/1 |
| F003 | `matched_stereo_points` | Matched IR/RGB points |
| F004 | `valid_stereo_point_percent` | Valid matches / sampled contour points |
| F005 | `median_disparity_px` | Median disparity |
| F006 | `disparity_mad_px` | Median absolute deviation |
| F007 | `disparity_stability_score` | `clamp(1 - mad / max(0.5, disparity), 0, 1)` |
| F008 | `epipolar_y_error_px` | Left-right y mismatch |
| F009 | `left_right_bbox_shift_px` | x shift between matched bboxes |
| F010 | `estimated_distance_mm` | `fx * baseline / disparity`, nullable |
| F011 | `distance_uncertainty_mm` | propagated from disparity uncertainty |
| F012 | `distance_confidence` | 0-1 confidence |
| F013 | `far_mode_downgraded_correctly` | Low confidence downgraded to `ImageOnlyContour`, 0/1 |
| F014-F015 | `far_distance_error_mm`, `far_relative_distance_error_percent` | Only when truth exists |

Stereo point thresholds:

```text
matched_stereo_points >= 30: good
12 - 30: usable
< 12: distance estimate unreliable
```

Recommended distance confidence:

```text
distance_confidence =
  0.30 * matched_point_score
+ 0.25 * disparity_stability_score
+ 0.20 * epipolar_score
+ 0.15 * contour_confidence
+ 0.10 * temporal_distance_score
```

## G. Contour Quality Features

Use ground truth when available; otherwise use image/depth evidence and temporal stability.

| ID | Field | With Truth |
|---|---|---|
| G001 | `mask_iou` | Predicted mask IoU |
| G002 | `bbox_iou` | Bounding-box IoU |
| G003 | `center_error_px` | Center distance |
| G004 | `area_error_percent` | Relative area error |
| G005 | `boundary_f_score` | Boundary F-score with 2/4/8 px tolerance |
| G006 | `boundary_hausdorff_p95_px` | Boundary p95 Hausdorff distance |

No-truth fields:

| ID | Field | Meaning |
|---|---|---|
| G007 | `edge_alignment_score` | Contour points near IR/RGB/depth edges |
| G008 | `contour_closure_score` | Closed-contour quality |
| G009 | `contour_jaggedness` | `perimeter_px / sqrt(area_px)` style roughness |
| G010 | `contour_leak_score` | Leakage into background or neighbor objects |
| G011 | `contour_undersegmentation_score` | Multiple objects merged |
| G012 | `contour_oversegmentation_score` | One object split into fragments |

## H. Temporal Tracking Features

| ID | Field | Meaning |
|---|---|---|
| H001 | `track_lifetime_frames` | Track lifetime |
| H002 | `stable_frames` | Consecutive stable hits |
| H003 | `misses` | Consecutive misses |
| H004-H005 | `frame_to_frame_mask_iou`, `frame_to_frame_bbox_iou` | Adjacent-frame consistency |
| H006 | `center_jitter_px` | Center movement from previous frame |
| H007 | `area_jitter_percent` | Area change from previous frame |
| H008 | `depth_jitter_mm` | Near 3D depth jitter |
| H009 | `distance_jitter_mm` | Far rough-distance jitter |
| H010 | `id_switch_count` | Estimated or truth-based ID switches |
| H011-H012 | `track_lost_count`, `track_recovered_count` | Frame-level tracking events |
| H013-H014 | `mode_transition_count`, `mode_oscillation_count` | Spatial-mode switching |
| H015 | `temporal_stability_score` | 0-1 score |

Recommended temporal stability:

```text
temporal_stability_score =
  0.30 * mask_iou_score
+ 0.20 * center_jitter_score
+ 0.20 * area_jitter_score
+ 0.15 * id_consistency_score
+ 0.15 * mode_stability_score
```

## I. Performance And Event Features

| ID | Field | Meaning |
|---|---|---|
| I001 | `total_frame_ms` | Per-frame total time |
| I002 | stage timing columns | capture, depth, boundary, stereo, tracker, visualization, recording |
| I003 | `realtime_fps_estimate` | `1000 / total_frame_ms` |
| I004 | `frame_time_over_budget` | 1 when over 33.3 ms for 30 FPS |
| I005 | `frame_time_spike_count` | Count over budget |
| I006-I012 | event flags | unknown spike, merge, split, stereo failure, contour lost, depth hole spike, sensor drop |

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

## Recommended frame_metrics.csv Header

```csv
frame_id,timestamp_ms,rgb_valid,depth_valid,ir_left_valid,ir_right_valid,total_frame_ms,capture_align_ms,depth_filter_ms,boundary_ms,near_cluster_ms,far_contour_ms,stereo_match_ms,merge_cluster_ms,tracker_ms,visualization_ms,recording_ms,total_pixels,clustered_pixels,unknown_pixels,assignment_coverage_percent,cluster_coverage_percent,unknown_percent,precise_depth3d_pixels,approx_stereo_contour_pixels,image_only_contour_pixels,background_plane_pixels,far_background_pixels,depth_hole_candidate_pixels,precise_depth3d_pixel_percent,approx_stereo_contour_pixel_percent,image_only_contour_pixel_percent,background_pixel_percent,depth_hole_candidate_pixel_percent,reliable_depth_pixels,unreliable_depth_pixels,depth_hole_pixels,total_cluster_count,foreground_cluster_count,background_cluster_count,unknown_cluster_count,near_cluster_count,far_cluster_count,far_distance_evidence_cluster_count,image_only_cluster_count,depth_hole_candidate_cluster_count,small_cluster_count,small_cluster_noise_ratio,cluster_fragmentation_index,candidate_count,anchor_supported_count,cue_input_count,cue_accepted_count,cue_rejected_texture_count,stable_track_count,new_track_count,lost_track_count,recovered_track_count,id_switch_estimate_count,mode_transition_count,mode_oscillation_count,depth_step_edge_px,depth_hole_edge_px,gray_edge_px,gray_depth_supported_edge_px,gray_depth_confirmed_edge_px,split_boundary_px,stereo_candidate_cluster_count,stereo_matched_cluster_count,stereo_failed_cluster_count,avg_contour_iou,boundary_jitter_px,unknown_spike,merge_event,split_event,far_stereo_failed_event,far_distance_missing_event,contour_lost_event,frame_time_over_budget
```

## Minimum Landing Set

If a run cannot populate the full feature set yet, populate these fields first.

Minimum `frame_metrics.csv`:

```csv
frame_id,total_frame_ms,total_pixels,clustered_pixels,unknown_pixels,assignment_coverage_percent,cluster_coverage_percent,unknown_percent,precise_depth3d_pixels,approx_stereo_contour_pixels,image_only_contour_pixels,background_pixels,depth_hole_candidate_pixels,near_cluster_count,far_cluster_count,far_distance_evidence_cluster_count,image_only_cluster_count,background_cluster_count,unknown_cluster_count,total_cluster_count,reliable_depth_pixels,depth_hole_pixels,stable_track_count,new_track_count,lost_track_count,stereo_matched_cluster_count,stereo_failed_cluster_count,unknown_spike,merge_event,split_event,far_stereo_failed_event,far_distance_missing_event,contour_lost_event
```

Minimum `cluster_metrics.jsonl`:

```json
{"frame_id":0,"cluster_id":0,"track_id":0,"mode":"PreciseDepth3D","source":"depth_ir_pcl_anchor","pixel_count":0,"bbox_2d":[0,0,0,0],"center_2d":[0,0],"contour_area_px":0,"contour_confidence":0,"depth_valid_percent":0,"depth_median_mm":0,"depth_iqr_mm":0,"anchor_count":0,"pcl_vote_ratio":0,"median_disparity_px":null,"matched_stereo_points":0,"estimated_distance_mm":null,"distance_uncertainty_mm":null,"distance_confidence":0,"stable_frames":0,"misses":0,"frame_to_frame_mask_iou":0,"center_jitter_px":0}
```

Minimum `events.csv` event types:

```text
unknown_spike
cluster_merge
cluster_split
track_lost
track_recovered
far_stereo_failed
far_distance_missing
contour_lost
frame_time_spike
sensor_drop
```

`far_stereo_failed` is a method diagnostic: color contours existed but direct stereo did not produce a distance. It must not by itself mean that all far-distance evidence is absent. `far_distance_evidence_cluster_count` counts `ApproxStereoContour` clusters carrying either a positive depth interval or stereo distance. `far_distance_missing` is raised only when neither source is available, and scoring penalties use this evidence-level event rather than the stereo-method diagnostic.

## Hard Failure Conditions

The run fails regardless of partial scores when any of these occur:

```text
cluster_coverage_percent_p50 < 95%
unknown_percent_p50 > 15%
total_frame_ms_p95 > 100ms when the target is realtime
depth_valid input is invalid for a long interval
run_manifest.json / config_snapshot.json / frame_metrics.csv is missing
cluster_metrics.jsonl is missing
```

## Score Breakdown

```text
S_pixel = 8*coverage + 4*unknown + 3*fragmentation + 3*small_noise + 2*mode_distribution
S_contour = 20 points from ground truth or no-truth contour evidence
S_spatial = 12*near_3d + 5*far_distance + 3*correct_downgrade
S_temporal = 4*mask_iou + 3*id_consistency + 3*center_jitter + 2*area_jitter + 2*mode_stability + 1*recovery
S_far_retention = 4*contour_retention + 2*image_only_fallback + 2*stereo_match + 2*far_temporal
S_perf = 5*frame_time_p95 + 2*spikes + 2*stage_balance + 1*recording_overhead
S_diag = manifest + config + frame_metrics + cluster_metrics + events/sample_frames
```

## Required Scenario Coverage

| Scene | Purpose | Key Features |
|---|---|---|
| Scene 01 near single object | Validate `PreciseDepth3D` | `depth_valid_percent`, `near_3d_confidence`, `depth_jitter_mm`, `mask_iou`, `center_jitter_px` |
| Scene 02 adjacent near objects | Detect merge/split | `cluster_merge_event`, `cluster_split_event`, `pcl_vote_ratio`, `depth_iqr_mm` |
| Scene 03 border object / arm / person | Validate border foreground protection | `foreground_cluster_count`, `contour_lost_event`, `background_swallow_foreground` |
| Scene 04 far visible object | Validate far contour retention | `far_contour_retained`, `approx_stereo_contour_pixel_percent`, `image_only_contour_pixel_percent` |
| Scene 05 far rough distance | Validate stereo rough distance | `median_disparity_px`, `disparity_mad_px`, `matched_stereo_points`, `distance_confidence` |
| Scene 06 depth holes / reflective / black objects | Retain visual existence when depth fails | `depth_hole_candidate_pixel_percent`, `ImageOnlyContour`, `unknown_spike` |
| Scene 07 wall / floor / ceiling / desktop | Cluster background sensibly | `background_pixel_percent`, `background_cluster_count`, plane stability |
| Scene 08 occlusion and return | Validate tracking stability | `track_lost_count`, `track_recovered_count`, `id_switch_count`, `mode_oscillation_count` |

## run_score.json Output

```json
{
  "run_id": "20260706_153000_case01",
  "pass": false,
  "total_score": 72.4,
  "scores": {
    "pixel_clustering": 16.8,
    "contour_quality": 13.2,
    "spatial_quality": 14.6,
    "temporal_stability": 10.1,
    "far_retention": 7.4,
    "performance": 6.8,
    "diagnostic_completeness": 3.5
  },
  "hard_fail_reasons": [
    "unknown_percent_p50 > 15%"
  ],
  "top_failures": [
    "far contour lost around frame 180",
    "near cluster merge around frame 240"
  ],
  "recommended_next_actions": [
    "increase visual contour retention before depth gating",
    "inspect PCL vote ratio for merged near clusters"
  ]
}
```

## Engineering Principles

1. Far field is not judged primarily by exact depth.
   Priority: contour existence, stable 2D position/size, rough distance, then precision.
2. Near field must expose spatial confidence, not just any depth value.
3. Full-frame clustering must not hide failures behind `Unknown`.
4. Every cluster must expose its distance source through `mode` and `source`.
