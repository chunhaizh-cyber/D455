#!/usr/bin/env python3
import argparse
import csv
import json
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path


MODE_ALIASES = {
    "NearPreciseObject": "PreciseDepth3D",
    "FarContourObject": "ApproxStereoContour",
}

PIXEL_FIELD_BY_MODE = {
    "PreciseDepth3D": "precise_depth3d_pixels",
    "ApproxStereoContour": "approx_stereo_contour_pixels",
    "ImageOnlyContour": "image_only_contour_pixels",
    "BackgroundPlane": "background_plane_pixels",
    "FarBackground": "far_background_pixels",
    "DepthHoleCandidate": "depth_hole_candidate_pixels",
    "Unknown": "unknown_pixels",
}

FRAME_FIELDS = [
    "frame_id",
    "total_frame_ms",
    "total_pixels",
    "clustered_pixels",
    "unknown_pixels",
    "cluster_coverage_percent",
    "unknown_percent",
    "assignment_coverage_percent",
    "precise_depth3d_pixels",
    "approx_stereo_contour_pixels",
    "image_only_contour_pixels",
    "background_plane_pixels",
    "far_background_pixels",
    "background_pixels",
    "depth_hole_candidate_pixels",
    "near_cluster_count",
    "far_cluster_count",
    "image_only_cluster_count",
    "background_cluster_count",
    "unknown_cluster_count",
    "total_cluster_count",
    "reliable_depth_pixels",
    "depth_hole_pixels",
    "stable_track_count",
    "new_track_count",
    "lost_track_count",
    "stereo_matched_cluster_count",
    "stereo_failed_cluster_count",
    "color_contour_refreshed",
    "color_contour_cache_reused",
    "color_contour_refresh_startup",
    "color_contour_refresh_interval",
    "color_contour_refresh_motion",
    "color_contour_refresh_unknown_spike",
    "color_contour_refresh_far_loss",
    "color_contour_motion_delta_percent",
    "color_contour_region_count",
    "color_contour_stereo_valid_count",
    "color_contour_stereo_reuse_count",
    "color_contour_async_submitted",
    "color_contour_async_applied",
    "color_contour_async_dropped",
    "color_contour_async_pending",
    "color_contour_cache_age_frames",
    "color_contour_async_worker_ms",
    "unknown_spike",
    "merge_event",
    "split_event",
    "far_stereo_failed_event",
    "contour_lost_event",
    "frame_time_over_budget",
]

EVENT_FIELDS = [
    "frame_id",
    "event_type",
    "severity",
    "cluster_id",
    "track_id",
    "related_cluster_id",
    "message",
    "value_before",
    "value_after",
]


def normalize_mode(mode):
    return MODE_ALIASES.get(str(mode or "Unknown"), str(mode or "Unknown"))


def as_float(value, default=0.0):
    if value in (None, ""):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def as_int(value, default=0):
    if value in (None, ""):
        return default
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def nullable_positive_int(value):
    parsed = as_int(value, 0)
    return parsed if parsed > 0 else None


def read_json(path):
    if not path or not path.exists():
        return {}
    with path.open("r", encoding="utf-8-sig") as f:
        return json.load(f)


def read_csv_rows(path):
    if not path or not path.exists():
        return []
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    return rows


def read_last_csv_row(path):
    rows = read_csv_rows(path)
    return rows[-1] if rows else {}


def row_frame_id(row):
    return as_int(row.get("frame_index") or row.get("frame_id"), -1)


def find_metadata(run_dir, stem):
    candidates = [run_dir / f"{stem}_metadata.json"]
    candidates.extend(sorted(run_dir.glob(f"{stem}*_metadata.json")))
    for path in candidates:
        if path.exists():
            return path
    return None


def frame_id_from_path(path):
    match = re.search(r"_frame_(\d+)_metadata\.json$", path.name)
    return int(match.group(1)) if match else None


def frame_id_from_metadata(path, meta):
    if "frame_id" in meta:
        return as_int(meta.get("frame_id"))
    return frame_id_from_path(path)


def metadata_series_from_explicit(path):
    if not path:
        return []
    path = Path(path)
    stem = path.name
    if stem.endswith("_metadata.json"):
        stem = stem[:-len("_metadata.json")]
    frame_matches = sorted(path.parent.glob(f"{stem}_frame_*_metadata.json"))
    if frame_matches:
        return frame_matches
    return [path] if path.exists() else []


def find_metadata_series(run_dir, explicit_path, stem):
    explicit_series = metadata_series_from_explicit(explicit_path)
    if explicit_series:
        return explicit_series
    frame_matches = sorted(run_dir.glob(f"{stem}_frame_*_metadata.json"))
    if frame_matches:
        return frame_matches
    single = find_metadata(run_dir, stem)
    return [single] if single else []


def load_metadata_series(run_dir, explicit_path, stem):
    series = []
    for path in find_metadata_series(run_dir, explicit_path, stem):
        meta = read_json(path)
        if not meta:
            continue
        series.append({
            "frame_id": frame_id_from_metadata(path, meta),
            "path": path,
            "meta": meta,
        })
    return sorted(series, key=lambda item: (-1 if item["frame_id"] is None else item["frame_id"], str(item["path"])))


def find_optional_csv(run_dir, names):
    for name in names:
        path = run_dir / name
        if path.exists():
            return path
    for pattern in names:
        matches = sorted(run_dir.glob(pattern))
        if matches:
            return matches[-1]
    return None


def git_value(args):
    try:
        return subprocess.check_output(args, text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return ""


def image_size_from_metadata(cluster_meta, final_meta):
    size = cluster_meta.get("image_size") or final_meta.get("image_size") or [0, 0]
    if not isinstance(size, list) or len(size) < 2:
        return 0, 0
    return as_int(size[0]), as_int(size[1])


def bbox_area(cluster):
    bbox = cluster.get("bbox_2d") or [0, 0, 0, 0]
    if not isinstance(bbox, list) or len(bbox) < 4:
        return 0
    return max(0, as_int(bbox[2])) * max(0, as_int(bbox[3]))


def contour_area(cluster):
    if "contour_area_px" in cluster:
        return as_float(cluster.get("contour_area_px"))
    return float(bbox_area(cluster))


def synthesize_clusters_from_final(final_meta):
    clusters = []
    next_id = 1
    for material in final_meta.get("depth_materials", []):
        clusters.append({
            "id": next_id,
            "mode": "PreciseDepth3D",
            "source": "depth_ir_pcl_anchor",
            "pixel_count": material.get("pixel_count", 0),
            "bbox_2d": material.get("bbox_2d", [0, 0, 0, 0]),
            "center_2d": material.get("center_2d", [0, 0]),
            "depth_min_mm": material.get("depth_min_mm", 0),
            "depth_mean_mm": material.get("depth_mean_mm", 0),
            "depth_max_mm": material.get("depth_max_mm", 0),
            "confidence": 0.75,
        })
        next_id += 1
    for region in final_meta.get("color_regions", []):
        has_distance = as_int(region.get("estimated_distance_mm"), 0) > 0
        clusters.append({
            "id": next_id,
            "mode": "ApproxStereoContour" if has_distance else "ImageOnlyContour",
            "source": "ir_left_right_contour" if has_distance else "rgb_ir_visual_contour",
            "pixel_count": region.get("pixel_count", 0),
            "bbox_2d": region.get("bbox_2d", [0, 0, 0, 0]),
            "center_2d": region.get("center_2d", [0, 0]),
            "estimated_distance_mm": region.get("estimated_distance_mm"),
            "distance_uncertainty_mm": region.get("distance_uncertainty_mm"),
            "median_disparity_px": region.get("median_disparity_px"),
            "matched_stereo_points": region.get("matched_stereo_points", 0),
            "contour_confidence": region.get("contour_confidence", 0.0),
            "distance_confidence": region.get("distance_confidence", 0.0),
        })
        next_id += 1
    return clusters


def load_clusters(cluster_meta, final_meta, total_pixels):
    clusters = list(cluster_meta.get("clusters") or [])
    if not clusters:
        clusters = synthesize_clusters_from_final(final_meta)
    for index, cluster in enumerate(clusters, start=1):
        cluster["id"] = as_int(cluster.get("cluster_id", cluster.get("id", index)), index)
        cluster["mode"] = normalize_mode(cluster.get("mode"))
    assigned = sum(as_int(c.get("pixel_count")) for c in clusters)
    if total_pixels > 0 and assigned < total_pixels and not any(c.get("mode") == "Unknown" for c in clusters):
        clusters.append({
            "id": max([as_int(c.get("id")) for c in clusters] or [0]) + 1,
            "mode": "Unknown",
            "source": "unknown_fill",
            "pixel_count": total_pixels - assigned,
            "bbox_2d": [0, 0, 0, 0],
            "center_2d": [0, 0],
            "confidence": 0.0,
        })
    return clusters


def summarize_frame(run_dir, cluster_meta, final_meta, profile_row, clusters, args, frame_id_override=None):
    width, height = image_size_from_metadata(cluster_meta, final_meta)
    total_pixels = max(0, width * height)
    mode_pixels = {field: 0 for field in PIXEL_FIELD_BY_MODE.values()}
    mode_counts = {mode: 0 for mode in PIXEL_FIELD_BY_MODE}
    for cluster in clusters:
        mode = normalize_mode(cluster.get("mode"))
        pixels = as_int(cluster.get("pixel_count"))
        mode_pixels[PIXEL_FIELD_BY_MODE.get(mode, "unknown_pixels")] += pixels
        mode_counts[mode] = mode_counts.get(mode, 0) + 1

    unknown_pixels = mode_pixels["unknown_pixels"]
    if total_pixels <= 0:
        total_pixels = sum(mode_pixels.values())
    clustered_pixels = max(0, total_pixels - unknown_pixels)
    assigned_pixels = sum(mode_pixels.values())
    assignment_coverage = (
        as_float(cluster_meta.get("assignment_coverage_percent"), None)
        if cluster_meta.get("assignment_coverage_percent") is not None
        else (100.0 * assigned_pixels / total_pixels if total_pixels else 0.0)
    )
    cluster_coverage = (
        as_float(cluster_meta.get("cluster_coverage_percent"), None)
        if cluster_meta.get("cluster_coverage_percent") is not None
        else (100.0 * clustered_pixels / total_pixels if total_pixels else 0.0)
    )
    unknown_percent = (
        as_float(cluster_meta.get("unknown_percent"), None)
        if cluster_meta.get("unknown_percent") is not None
        else (100.0 * unknown_pixels / total_pixels if total_pixels else 0.0)
    )

    stereo_valid = as_int(final_meta.get("stereo_distance_valid_count"))
    color_region_count = as_int(final_meta.get("color_region_count"))
    stereo_failed = max(0, color_region_count - stereo_valid)
    total_ms = as_float(
        profile_row.get("total_ms") or
        profile_row.get("processing_ms") or
        profile_row.get("total_frame_ms"),
        0.0,
    )
    frame_id = frame_id_override if frame_id_override is not None else row_frame_id(profile_row)
    if frame_id < 0:
        frame_id = 0

    row = {field: 0 for field in FRAME_FIELDS}
    row.update({
        "frame_id": frame_id,
        "total_frame_ms": round(total_ms, 3),
        "total_pixels": total_pixels,
        "clustered_pixels": clustered_pixels,
        "unknown_pixels": unknown_pixels,
        "cluster_coverage_percent": round(cluster_coverage, 3),
        "unknown_percent": round(unknown_percent, 3),
        "assignment_coverage_percent": round(assignment_coverage, 3),
        "precise_depth3d_pixels": mode_pixels["precise_depth3d_pixels"],
        "approx_stereo_contour_pixels": mode_pixels["approx_stereo_contour_pixels"],
        "image_only_contour_pixels": mode_pixels["image_only_contour_pixels"],
        "background_plane_pixels": mode_pixels["background_plane_pixels"],
        "far_background_pixels": mode_pixels["far_background_pixels"],
        "background_pixels": mode_pixels["background_plane_pixels"] + mode_pixels["far_background_pixels"],
        "depth_hole_candidate_pixels": mode_pixels["depth_hole_candidate_pixels"],
        "near_cluster_count": mode_counts.get("PreciseDepth3D", 0),
        "far_cluster_count": mode_counts.get("ApproxStereoContour", 0),
        "image_only_cluster_count": mode_counts.get("ImageOnlyContour", 0),
        "background_cluster_count": mode_counts.get("BackgroundPlane", 0) + mode_counts.get("FarBackground", 0),
        "unknown_cluster_count": mode_counts.get("Unknown", 0),
        "total_cluster_count": len(clusters),
        "reliable_depth_pixels": mode_pixels["precise_depth3d_pixels"],
        "stable_track_count": as_int(profile_row.get("stable_count")),
        "stereo_matched_cluster_count": stereo_valid,
        "stereo_failed_cluster_count": stereo_failed,
        "color_contour_refreshed": as_int(profile_row.get("color_contour_refreshed")),
        "color_contour_cache_reused": as_int(profile_row.get("color_contour_cache_reused")),
        "color_contour_refresh_startup": as_int(profile_row.get("color_contour_refresh_startup")),
        "color_contour_refresh_interval": as_int(profile_row.get("color_contour_refresh_interval")),
        "color_contour_refresh_motion": as_int(profile_row.get("color_contour_refresh_motion")),
        "color_contour_refresh_unknown_spike": as_int(profile_row.get("color_contour_refresh_unknown_spike")),
        "color_contour_refresh_far_loss": as_int(profile_row.get("color_contour_refresh_far_loss")),
        "color_contour_motion_delta_percent": round(
            as_float(profile_row.get("color_contour_motion_delta_percent")),
            3,
        ),
        "color_contour_region_count": as_int(profile_row.get("color_contour_region_count"), color_region_count),
        "color_contour_stereo_valid_count": as_int(
            profile_row.get("color_contour_stereo_valid_count"),
            stereo_valid,
        ),
        "color_contour_stereo_reuse_count": as_int(profile_row.get("color_contour_stereo_reuse_count")),
        "color_contour_async_submitted": as_int(profile_row.get("color_contour_async_submitted")),
        "color_contour_async_applied": as_int(profile_row.get("color_contour_async_applied")),
        "color_contour_async_dropped": as_int(profile_row.get("color_contour_async_dropped")),
        "color_contour_async_pending": as_int(profile_row.get("color_contour_async_pending")),
        "color_contour_cache_age_frames": as_int(profile_row.get("color_contour_cache_age_frames")),
        "color_contour_async_worker_ms": round(
            as_float(profile_row.get("color_contour_async_worker_ms")),
            3,
        ),
        "unknown_spike": int(unknown_percent > args.unknown_spike_percent),
        "far_stereo_failed_event": int(color_region_count > 0 and stereo_valid <= 0),
        "frame_time_over_budget": int(total_ms > args.frame_budget_ms) if total_ms else 0,
    })
    return row


def cluster_metric_line(frame_id, total_pixels, cluster):
    mode = normalize_mode(cluster.get("mode"))
    pixel_count = as_int(cluster.get("pixel_count"))
    confidence = as_float(cluster.get("confidence", cluster.get("contour_confidence")), 0.0)
    depth_mean = nullable_positive_int(cluster.get("depth_mean_mm"))
    estimated_distance = nullable_positive_int(cluster.get("estimated_distance_mm"))
    matched = as_int(cluster.get("matched_stereo_points"))
    return {
        "frame_id": frame_id,
        "cluster_id": as_int(cluster.get("cluster_id", cluster.get("id"))),
        "track_id": cluster.get("track_id"),
        "mode": mode,
        "source": cluster.get("source", ""),
        "pixel_count": pixel_count,
        "area_percent": round(100.0 * pixel_count / total_pixels, 3) if total_pixels else 0.0,
        "bbox_2d": cluster.get("bbox_2d", [0, 0, 0, 0]),
        "center_2d": cluster.get("center_2d", [0, 0]),
        "contour_area_px": contour_area(cluster),
        "contour_confidence": confidence,
        "depth_min_mm": nullable_positive_int(cluster.get("depth_min_mm")),
        "depth_mean_mm": depth_mean,
        "depth_max_mm": nullable_positive_int(cluster.get("depth_max_mm")),
        "depth_valid_percent": 100.0 if mode == "PreciseDepth3D" and depth_mean else None,
        "median_disparity_px": (
            as_float(cluster.get("median_disparity_px"))
            if cluster.get("median_disparity_px") not in (None, "")
            else None
        ),
        "matched_stereo_points": matched,
        "estimated_distance_mm": estimated_distance,
        "distance_uncertainty_mm": nullable_positive_int(cluster.get("distance_uncertainty_mm")),
        "distance_confidence": as_float(cluster.get("distance_confidence")),
        "confidence": confidence,
        "stable_frames": as_int(cluster.get("stable_frames")),
        "misses": as_int(cluster.get("misses")),
    }


def write_frame_metrics(run_dir, rows):
    if isinstance(rows, dict):
        rows = [rows]
    with (run_dir / "frame_metrics.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FRAME_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def write_cluster_metrics(run_dir, frame_clusters):
    with (run_dir / "cluster_metrics.jsonl").open("w", encoding="utf-8") as f:
        for frame_id, total_pixels, clusters in frame_clusters:
            for cluster in clusters:
                f.write(json.dumps(
                    cluster_metric_line(frame_id, total_pixels, cluster),
                    ensure_ascii=False,
                    separators=(",", ":"),
                ) + "\n")


def collect_events(frame_row):
    events = []
    frame_id = frame_row["frame_id"]
    if frame_row["unknown_spike"]:
        events.append({
            "frame_id": frame_id,
            "event_type": "unknown_spike",
            "severity": "warning",
            "cluster_id": "",
            "track_id": "",
            "related_cluster_id": "",
            "message": "unknown_percent exceeded converter threshold",
            "value_before": "",
            "value_after": frame_row["unknown_percent"],
        })
    if frame_row["far_stereo_failed_event"]:
        events.append({
            "frame_id": frame_id,
            "event_type": "far_stereo_failed",
            "severity": "warning",
            "cluster_id": "",
            "track_id": "",
            "related_cluster_id": "",
            "message": "final color regions had no valid stereo distance",
            "value_before": "",
            "value_after": frame_row["stereo_failed_cluster_count"],
        })
    if frame_row.get("color_contour_refreshed"):
        reasons = []
        for key, label in [
            ("color_contour_refresh_startup", "startup"),
            ("color_contour_refresh_interval", "interval"),
            ("color_contour_refresh_motion", "motion"),
            ("color_contour_refresh_unknown_spike", "unknown_spike"),
            ("color_contour_refresh_far_loss", "far_loss"),
        ]:
            if frame_row.get(key):
                reasons.append(label)
        events.append({
            "frame_id": frame_id,
            "event_type": "color_contour_refresh",
            "severity": "info",
            "cluster_id": "",
            "track_id": "",
            "related_cluster_id": "",
            "message": "color contour cache refreshed: " + ("+".join(reasons) if reasons else "unspecified"),
            "value_before": frame_row.get("color_contour_motion_delta_percent", ""),
            "value_after": frame_row.get("color_contour_region_count", ""),
        })
    if frame_row["frame_time_over_budget"]:
        events.append({
            "frame_id": frame_id,
            "event_type": "frame_time_spike",
            "severity": "warning",
            "cluster_id": "",
            "track_id": "",
            "related_cluster_id": "",
            "message": "frame time exceeded converter budget",
            "value_before": "",
            "value_after": frame_row["total_frame_ms"],
        })
    return events


def write_events(run_dir, frame_rows):
    if isinstance(frame_rows, dict):
        frame_rows = [frame_rows]
    events = []
    for frame_row in frame_rows:
        events.extend(collect_events(frame_row))
    with (run_dir / "events.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=EVENT_FIELDS)
        writer.writeheader()
        writer.writerows(events)


def load_candidate_config(path):
    if not path:
        return {}
    return read_json(Path(path))


def manifest_is_template_or_incomplete(manifest, args):
    if not manifest:
        return True
    if manifest.get("run_id") == "YYYYMMDD_HHMMSS_case01":
        return True
    if manifest.get("evaluation_mode") == "live_camera" and not manifest.get("converted_from"):
        return True
    if args.candidate_id and not manifest.get("candidate_id"):
        return True
    if args.case_id and not manifest.get("case_id"):
        return True
    if not manifest.get("converted_from"):
        return True
    return False


def config_is_template_or_incomplete(config, candidate_config):
    if not config:
        return True
    if config.get("source") == "convert_exports_to_analysis_run.py":
        return False
    if candidate_config and not config.get("candidate_config"):
        return True
    if not config.get("inputs"):
        return True
    template_markers = {"feature_profile", "feature_heavy_interval", "notes"}
    return any(key in config for key in template_markers)


def build_manifest(run_dir, args, candidate_config, inputs, existing=None):
    manifest = dict(existing or {})
    manifest.update({
        "run_id": args.run_id or manifest.get("run_id") or run_dir.name,
        "candidate_id": args.candidate_id or manifest.get("candidate_id") or candidate_config.get("candidate_id"),
        "case_id": args.case_id or manifest.get("case_id"),
        "purpose": args.purpose or manifest.get("purpose"),
        "git_commit": git_value(["git", "rev-parse", "--short", "HEAD"]),
        "branch": git_value(["git", "branch", "--show-current"]),
        "evaluation_mode": "converted_export",
        "command_line": args.command_line or manifest.get("command_line", ""),
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "converted_from": {name: stringify_input(path) for name, path in inputs.items() if path},
    })
    return manifest


def stringify_input(value):
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value]
    return str(value)


def build_config_snapshot(candidate_config, inputs, existing=None):
    config = {
        "source": "convert_exports_to_analysis_run.py",
        "candidate_config": candidate_config,
        "inputs": {name: stringify_input(path) for name, path in inputs.items() if path},
    }
    if existing:
        config["previous_config_snapshot"] = existing
    return config


def write_manifest_and_config(run_dir, args, candidate_config, inputs):
    manifest_path = run_dir / "run_manifest.json"
    existing_manifest = read_json(manifest_path)
    if args.update_manifest or manifest_is_template_or_incomplete(existing_manifest, args):
        manifest = build_manifest(run_dir, args, candidate_config, inputs, existing_manifest)
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    config_path = run_dir / "config_snapshot.json"
    existing_config = read_json(config_path)
    if args.overwrite_config_snapshot or config_is_template_or_incomplete(existing_config, candidate_config):
        config = build_config_snapshot(candidate_config, inputs, existing_config)
        config_path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    notes_path = run_dir / "notes.md"
    if not notes_path.exists():
        notes_path.write_text(
            f"# {args.run_id or run_dir.name}\n\n"
            "Generated from D455 export metadata. Add scene notes, expected result, "
            "visible failure frames, and rough ground truth distances before cloud review.\n",
            encoding="utf-8",
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--cluster-map", default="")
    parser.add_argument("--final-segmentation", default="")
    parser.add_argument("--profile-csv", default="")
    parser.add_argument("--acceptance-csv", default="")
    parser.add_argument("--candidate-config", default="")
    parser.add_argument("--run-id", default="")
    parser.add_argument("--candidate-id", default="")
    parser.add_argument("--case-id", default="")
    parser.add_argument("--purpose", default="Converted D455 export smoke analysis run.")
    parser.add_argument("--command-line", default="")
    parser.add_argument("--update-manifest", action="store_true")
    parser.add_argument("--overwrite-config-snapshot", action="store_true")
    parser.add_argument("--ignore-first-n-frames", type=int, default=0)
    parser.add_argument("--unknown-spike-percent", type=float, default=15.0)
    parser.add_argument("--frame-budget-ms", type=float, default=33.3)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)

    profile_path = Path(args.profile_csv) if args.profile_csv else find_optional_csv(run_dir, ["profile.csv", "profile_*.csv"])
    acceptance_path = Path(args.acceptance_csv) if args.acceptance_csv else find_optional_csv(run_dir, ["acceptance.csv", "acceptance_*.csv"])
    timing_path = profile_path or acceptance_path

    cluster_series = load_metadata_series(run_dir, args.cluster_map, "cluster_map")
    final_series = load_metadata_series(run_dir, args.final_segmentation, "final_segmentation")
    if not cluster_series and not final_series:
        raise SystemExit("No cluster_map or final_segmentation metadata found.")

    profile_rows = read_csv_rows(timing_path)
    profile_by_frame = {
        row_frame_id(row): row for row in profile_rows if row_frame_id(row) >= 0
    }
    fallback_profile_row = profile_rows[-1] if profile_rows else {}
    final_by_frame = {
        item["frame_id"]: item for item in final_series if item["frame_id"] is not None
    }
    single_final = final_series[-1] if final_series else None

    source_series = cluster_series or final_series
    frame_rows = []
    frame_clusters = []
    for item in source_series:
        frame_id = item["frame_id"]
        if frame_id is not None and frame_id < args.ignore_first_n_frames:
            continue
        cluster_item = item if cluster_series else None
        final_item = final_by_frame.get(frame_id) if frame_id is not None else None
        if final_item is None:
            final_item = single_final
        cluster_meta = cluster_item["meta"] if cluster_item else {}
        final_meta = final_item["meta"] if final_item else {}
        profile_row = profile_by_frame.get(frame_id, fallback_profile_row)
        width, height = image_size_from_metadata(cluster_meta, final_meta)
        total_pixels = max(0, width * height)
        clusters = load_clusters(cluster_meta, final_meta, total_pixels)
        frame_row = summarize_frame(
            run_dir,
            cluster_meta,
            final_meta,
            profile_row,
            clusters,
            args,
            frame_id_override=frame_id)
        frame_rows.append(frame_row)
        frame_clusters.append((frame_row["frame_id"], frame_row["total_pixels"], clusters))
    if not frame_rows:
        raise SystemExit("No frame metrics remained after applying --ignore-first-n-frames.")

    candidate_config = load_candidate_config(args.candidate_config)
    inputs = {
        "cluster_map_metadata": [item["path"] for item in cluster_series],
        "final_segmentation_metadata": [item["path"] for item in final_series],
        "timing_csv": timing_path,
        "candidate_config": Path(args.candidate_config) if args.candidate_config else None,
    }
    write_manifest_and_config(run_dir, args, candidate_config, inputs)
    write_frame_metrics(run_dir, frame_rows)
    write_cluster_metrics(run_dir, frame_clusters)
    write_events(run_dir, frame_rows)
    print(f"converted {run_dir}: {len(frame_rows)} frame rows")


if __name__ == "__main__":
    main()
