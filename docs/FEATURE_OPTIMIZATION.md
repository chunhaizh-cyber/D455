# Feature Collection Performance Optimization

This document defines how D455 should collect evaluation features without turning the feature system into the main real-time bottleneck. It complements `docs/EVALUATION_FEATURES.md`: that file defines what should be measured; this file defines when and how expensive measurements should run.

## Principle

The feature set is the dashboard, not the engine load.

Use four rules:

1. The real-time loop computes only low-cost core features every frame.
2. Expensive features run every N frames or only on events.
3. Image maps are scanned once per frame to batch pixel and cluster statistics.
4. Metrics are written asynchronously; videos and raw frames are dumped only on samples or events.

## Feature Profiles

| Profile | Goal | Per-frame Work | Sampled / Event Work |
|---|---|---|---|
| `light` | Real-time development default | `frame_id`, `total_frame_ms`, mode pixel counts, coverage, unknown, cluster counts, key events | Simplified `cluster_metrics` every 5 frames; dump sample frames only on event |
| `normal` | Acceptance default | Full `frame_metrics`, core `cluster_metrics`, track stats, mode transitions | Heavy contour features and detailed far stereo every 15 frames; event ring buffer dump |
| `debug` | Problem localization | Full `cluster_metrics`, detailed timing, PCL diagnostics, stereo diagnostics | Periodic `cluster_id_map`, `mode_map`, overlay images |
| `full` | Offline research | Complete features and artifacts | No real-time target; suited for grid search and postmortem analysis |

Recommended knobs:

```text
--feature-profile=light|normal|debug|full
--feature-heavy-interval=15
--feature-cluster-metrics-interval=5
--feature-dump-on-event
--feature-event-ring-frames=10
```

The current code may not implement all of these flags yet. Until then, record equivalent values in `config_snapshot.json` so analysis can interpret the run correctly.

## Always-On Core Features

These are cheap enough to compute every frame:

```text
frame_id
timestamp_ms
input validity flags
total_frame_ms
total_pixels
clustered_pixels
unknown_pixels
cluster_coverage_percent
unknown_percent
mode pixel counts
cluster counts
basic bbox / center / pixel_count
stable_track_count
new_track_count
lost_track_count
event flags
```

## Sampled Or Event-Triggered Features

These should not be mandatory per-frame work in real-time profiles:

```text
mask_iou
Hausdorff / boundary_f_score
detailed edge alignment
detailed disparity distribution
full PCL vote diagnostics
cluster_id_map image dump
mode_map image dump
depth16 sample dump
debug overlay video frame dump
```

## Single-Pass Pixel Statistics

For `cluster_id_map` and `mode_map`, do not call `countNonZero` once per mode or per cluster. Scan the map once and accumulate:

```text
modeCounts[mode]
clusterPixelCounts[cluster_id]
bbox min/max
center sum_x / sum_y
unknown_pixels
small_cluster_count
```

This directly supports:

```text
cluster_coverage_percent
unknown_percent
mode pixel distribution
cluster_fragmentation_index
small_cluster_noise_ratio
```

## Near-Field PCL Policy

PCL should not be a mandatory per-frame cost in real-time profiles.

Recommended knobs:

```text
--pcl-mode=off|roi|sampled|full
--pcl-voxel-mm=10
--pcl-run-interval=2
--pcl-run-on-change
--pcl-max-roi-points=7000
```

Run or refresh PCL when:

```text
near_cluster_count changes
cluster_merge_event is suspected
cluster_split_event is suspected
depth_iqr_mm spikes
a new track appears
the PCL cache expires
```

Otherwise reuse the previous PCL association for `pcl_cluster_id` and `pcl_vote_ratio`, and mark whether the value is cached.

## Far-Field Stereo Policy

Far-field distance is rough distance, not high-precision depth. Do not run dense stereo for the whole frame.

Recommended policy:

```text
extract RGB/IR visual contours
sample contour boundary / corner / edge points
match sparse points between left/right IR
estimate median_disparity_px
derive estimated_distance_mm and distance_uncertainty_mm
downgrade to ImageOnlyContour when confidence is low
```

Recommended knobs:

```text
--far-stereo-interval=3
--far-stereo-boundary-samples=64
--far-stereo-sparse-only
```

Between stereo updates, propagate contour/bbox through the tracker and keep the last distance only while `distance_confidence` remains valid.

## RealSense Depth Policy

Depth filtering affects both speed and near 3D confidence.

Recommended profile mapping:

| Profile | Depth Strategy |
|---|---|
| `fast` | decimation enabled, light spatial/temporal filtering, fewer heavy depth statistics |
| `balanced` | spatial + temporal filtering, per-frame valid depth / median / IQR |
| `stable` | stronger temporal behavior and more depth diagnostics; not necessarily real-time |

The run must record the effective depth policy in `config_snapshot.json`.

## Visual Pipeline Policy

OpenCV operations should be organized as reusable stages:

```text
buildVisualBoundaryGraph()
buildDepthBoundaryGraph()
buildModeMapGraph()
buildOverlayGraph()
```

Before considering a larger graph framework migration, first:

```text
avoid repeated cv::Mat clone
reuse cv::Mat buffers
batch morphology and connected-component passes
avoid repeated mask allocation inside per-cluster loops
run expensive overlay generation only when displayed or recorded
```

## Large Model Policy

SAM2 or similar segmentation models may be useful as candidate enhancers, but should not run full-frame every frame in the real-time loop.

Recommended use:

```text
offline failure-frame labeling
periodic far-field candidate enhancement
event-triggered ROI enhancement after unknown_spike or contour_lost
candidate mask generation followed by D455 depth/stereo/tracker validation
```

## Metrics I/O Policy

Do not write high-volume metrics directly from the real-time path.

Recommended design:

```text
main thread:
  compute core metrics
  enqueue FrameMetrics / ClusterMetrics / Events

writer thread:
  batch write CSV / JSONL
  batch flush
  save sample frames only on event or sample interval
```

Queue policy:

```text
core frame_metrics: do not drop
events: do not drop
debug overlays / sample maps: may drop oldest in light/normal profiles
```

## Offline Data Format Policy

Real-time output remains simple:

```text
frame_metrics.csv
cluster_metrics.jsonl
events.csv
```

After the run, convert large metrics to columnar analysis files when needed:

```text
frame_metrics.parquet
cluster_metrics.parquet
```

The conversion must be offline and must not block the camera loop.

## Profiling Policy

The existing CSV timing fields are necessary but not sufficient. Add timeline profiling when the CSV shows a persistent bottleneck.

Trace scopes should map to the evaluation feature system:

```cpp
TRACE_SCOPE("capture_align");
TRACE_SCOPE("depth_filter");
TRACE_SCOPE("boundary_analysis");
TRACE_SCOPE("near_cluster");
TRACE_SCOPE("pcl_refine");
TRACE_SCOPE("far_visual_contour");
TRACE_SCOPE("stereo_match");
TRACE_SCOPE("feature_collect");
TRACE_SCOPE("log_enqueue");
TRACE_SCOPE("video_write");
```

## First Implementation Slice

Implement in this order:

1. Add `feature_profile` and interval fields to config snapshots and run manifests.
2. Compute mode and cluster pixel counts from a single map scan.
3. Write `frame_metrics.csv` every frame, but simplify `cluster_metrics.jsonl` in `light`.
4. Make sample frame dumping event-triggered.
5. Add PCL cache reuse and run-on-change.
6. Move detailed far stereo to sparse contour points and interval updates.
7. Add a background writer thread before increasing metrics volume.

## Reference Technologies

- Temporal reuse: temporal cluster assignment and video object memory ideas.
- RealSense: decimation, spatial, temporal, and hole filling filters.
- PCL: VoxelGrid downsampling, ROI clustering, KdTree Euclidean clustering.
- OpenCV: staged graph-style visual pipelines and buffer reuse.
- SAM2: offline or event-triggered ROI candidate generation.
- spdlog-style async logging: queued metrics and background writes.
- Parquet / Arrow: offline columnar analysis.
- Perfetto / Nsight Systems: timeline-level bottleneck diagnosis.

Reference links:

- RealSense post-processing filters: https://github.com/realsenseai/librealsense/blob/master/doc/post-processing-filters.md
- PCL VoxelGrid downsampling: https://pointclouds.org/documentation/tutorials/voxel_grid.html
- PCL Euclidean cluster extraction: https://pointclouds.org/documentation/tutorials/cluster_extraction.html
- OpenCV G-API: https://docs.opencv.org/4.x/d0/d1e/gapi.html
- SAM2: https://github.com/facebookresearch/sam2
- spdlog asynchronous logging: https://github.com/gabime/spdlog/wiki/Asynchronous-logging
- Parquet file format: https://parquet.apache.org/docs/file-format/
- Perfetto tracing SDK: https://perfetto.dev/docs/instrumentation/tracing-sdk
