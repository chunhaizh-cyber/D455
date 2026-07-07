#!/usr/bin/env python3
import argparse
import csv
import html
import json
import shutil
from pathlib import Path


def read_json(path):
    with Path(path).open("r", encoding="utf-8-sig") as f:
        return json.load(f)


def read_run_score_metrics(run_dir):
    path = Path(run_dir) / "run_score.json"
    if not path.exists():
        return {}
    try:
        score = read_json(path)
    except (OSError, json.JSONDecodeError):
        return {}
    metrics = score.get("metrics")
    return metrics if isinstance(metrics, dict) else {}


def frame_id_from_path(path):
    name = Path(path).name
    marker = "_frame_"
    if marker not in name:
        return None
    suffix = name.split(marker, 1)[1].split("_metadata", 1)[0]
    try:
        return int(suffix)
    except ValueError:
        return None


def metadata_by_frame(run_dir):
    result = {}
    for path in sorted(Path(run_dir).glob("final_segmentation_frame_*_metadata.json")):
        meta = read_json(path)
        frame_id = meta.get("frame_id", frame_id_from_path(path))
        if frame_id is not None:
            result[int(frame_id)] = {"path": path, "meta": meta}
    return result


def bbox_iou(a, b):
    ax1, ay1, aw, ah = [float(x) for x in a]
    bx1, by1, bw, bh = [float(x) for x in b]
    ax2, ay2 = ax1 + aw, ay1 + ah
    bx2, by2 = bx1 + bw, by1 + bh
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    union = max(1.0, aw * ah + bw * bh - inter)
    return inter / union


def center_distance(a, b):
    ax, ay = float(a[0]) + float(a[2]) * 0.5, float(a[1]) + float(a[3]) * 0.5
    bx, by = float(b[0]) + float(b[2]) * 0.5, float(b[1]) + float(b[3]) * 0.5
    return ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5


def is_matching_region(region, candidates, iou_min, center_max):
    bbox = region.get("bbox_2d", [0, 0, 0, 0])
    for candidate in candidates:
        candidate_bbox = candidate.get("bbox_2d", [0, 0, 0, 0])
        if bbox_iou(bbox, candidate_bbox) >= iou_min:
            return True
        if center_distance(bbox, candidate_bbox) <= center_max:
            return True
    return False


def overlay_path_for(metadata_path):
    return str(metadata_path).replace("_metadata.json", "_overlay.png")


def dataset_color_path(case_dir, frame_id):
    return Path(case_dir) / "frames" / f"{int(frame_id):06d}_color.png"


def copy_sample_image(source, dest):
    source = Path(source)
    if not source.exists():
        return ""
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, dest)
    return dest.as_posix()


def relative_link(path, base_dir):
    if not path:
        return ""
    path = Path(path)
    try:
        return path.relative_to(base_dir).as_posix()
    except ValueError:
        return path.as_posix()


def attach_sample_frames(rows, out, max_frames):
    if max_frames <= 0:
        return []
    sample_dir = out / "sample_frames"
    if sample_dir.exists():
        for path in sample_dir.glob("*.png"):
            path.unlink()
    sampled = []
    seen_frames = set()
    sample_rows = sorted(
        rows,
        key=lambda row: (
            0 if row.get("review_priority") == "high_target_risk" else 1,
            int(row.get("frame_id") or 0),
            -int(row.get("pixel_count") or 0),
        ),
    )
    for row in sample_rows:
        frame_id = int(row["frame_id"])
        if frame_id in seen_frames:
            continue
        seen_frames.add(frame_id)
        if len(sampled) >= max_frames:
            break
        prefix = f"frame_{frame_id:06d}_region_{row['baseline_region_id']}"
        for key, suffix in [
            ("source_color", "source_color.png"),
            ("baseline_overlay", "baseline_overlay.png"),
            ("filtered_overlay", "filtered_overlay.png"),
        ]:
            copied = copy_sample_image(row[key], sample_dir / f"{prefix}_{suffix}")
            if copied:
                row[f"sample_{key}"] = relative_link(copied, out)
        sampled.append(frame_id)
    return sampled


def g3_status(stereo_drop_count, no_stereo_drop_count, missing_count, contour_lost_events, runtime_drop_pixels, warning_pixels):
    if stereo_drop_count > 0:
        return "red"
    if (
        missing_count == 0 and
        stereo_drop_count == 0 and
        contour_lost_events == 0 and
        runtime_drop_pixels <= warning_pixels
    ):
        return "pass"
    if no_stereo_drop_count > 0 or runtime_drop_pixels > warning_pixels or contour_lost_events > 0:
        return "warning"
    return "warning" if missing_count > 0 else "pass"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-run", required=True, help="Run without drop filtering, such as candidate_0034.")
    parser.add_argument("--filtered-run", required=True, help="Run with drop filtering, such as candidate_0035.")
    parser.add_argument("--case-dir", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--match-iou-min", type=float, default=0.05)
    parser.add_argument("--match-center-max-px", type=float, default=120.0)
    parser.add_argument("--max-sample-frames", type=int, default=1)
    parser.add_argument("--runtime-drop-pixels-warning", type=int, default=20000)
    parser.add_argument("--no-copy-sample-frames", action="store_true")
    args = parser.parse_args()

    baseline = metadata_by_frame(args.baseline_run)
    filtered = metadata_by_frame(args.filtered_run)
    filtered_metrics = read_run_score_metrics(args.filtered_run)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rows = []
    for frame_id in sorted(set(baseline) & set(filtered)):
        base_item = baseline[frame_id]
        filtered_item = filtered[frame_id]
        base_regions = base_item["meta"].get("color_regions", [])
        filtered_regions = filtered_item["meta"].get("color_regions", [])
        for region in base_regions:
            if is_matching_region(region, filtered_regions, args.match_iou_min, args.match_center_max_px):
                continue
            bbox = region.get("bbox_2d", [0, 0, 0, 0])
            estimated_distance_mm = int(region.get("estimated_distance_mm") or 0)
            matched_stereo_points = int(region.get("matched_stereo_points") or 0)
            stereo_evidence = estimated_distance_mm > 0 or matched_stereo_points > 0
            rows.append({
                "frame_id": frame_id,
                "baseline_region_id": region.get("id", ""),
                "pixel_count": int(region.get("pixel_count") or 0),
                "bbox_x": bbox[0],
                "bbox_y": bbox[1],
                "bbox_w": bbox[2],
                "bbox_h": bbox[3],
                "center_x": region.get("center_2d", ["", ""])[0],
                "center_y": region.get("center_2d", ["", ""])[1],
                "estimated_distance_mm": estimated_distance_mm,
                "matched_stereo_points": matched_stereo_points,
                "stereo_evidence": int(stereo_evidence),
                "review_priority": "high_target_risk" if stereo_evidence else "fragment_review",
                "source_color": str(dataset_color_path(args.case_dir, frame_id)),
                "baseline_overlay": overlay_path_for(base_item["path"]),
                "filtered_overlay": overlay_path_for(filtered_item["path"]),
            })

    csv_path = out / "g3_drop_fragments.csv"
    fieldnames = [
        "frame_id",
        "baseline_region_id",
        "pixel_count",
        "bbox_x",
        "bbox_y",
        "bbox_w",
        "bbox_h",
        "center_x",
        "center_y",
        "estimated_distance_mm",
        "matched_stereo_points",
        "stereo_evidence",
        "review_priority",
        "source_color",
        "baseline_overlay",
        "filtered_overlay",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    total_pixels = sum(int(row["pixel_count"]) for row in rows)
    stereo_rows = [row for row in rows if int(row["stereo_evidence"]) > 0]
    no_stereo_rows = [row for row in rows if int(row["stereo_evidence"]) <= 0]
    unique_frames = sorted({int(row["frame_id"]) for row in rows})
    runtime_drop_count = int(filtered_metrics.get("color_contour_refresh_roi_stereo_dropped_count") or 0)
    runtime_drop_pixels = int(filtered_metrics.get("color_contour_refresh_roi_stereo_dropped_pixels") or 0)
    contour_lost_events = int(filtered_metrics.get("contour_lost_event_count") or 0)
    sampled_frames = []
    if rows and not args.no_copy_sample_frames:
        sampled_frames = attach_sample_frames(rows, out, args.max_sample_frames)
    status = g3_status(
        len(stereo_rows),
        len(no_stereo_rows),
        len(rows),
        contour_lost_events,
        runtime_drop_pixels,
        args.runtime_drop_pixels_warning,
    )
    warning_reasons = []
    red_reasons = []
    if stereo_rows:
        red_reasons.append("dropped_stereo_region_count > 0")
    if no_stereo_rows:
        warning_reasons.append("dropped_no_stereo_region_count > 0")
    if runtime_drop_pixels > args.runtime_drop_pixels_warning:
        warning_reasons.append(f"runtime_roi_stereo_dropped_pixels > {args.runtime_drop_pixels_warning}")
    if contour_lost_events > 0:
        warning_reasons.append("contour_lost_event_count > 0")
    summary = {
        "g3_status": status,
        "g3_red_reasons": red_reasons,
        "g3_warning_reasons": warning_reasons,
        "runtime_roi_stereo_dropped_count": runtime_drop_count,
        "runtime_roi_stereo_dropped_pixels": runtime_drop_pixels,
        "runtime_drop_pixels_warning": args.runtime_drop_pixels_warning,
        "contour_lost_event_count": contour_lost_events,
        "missing_baseline_region_count": len(rows),
        "missing_baseline_region_pixels": total_pixels,
        "dropped_fragment_count": len(rows),
        "dropped_fragment_pixels": total_pixels,
        "dropped_stereo_region_count": len(stereo_rows),
        "dropped_stereo_region_pixels": sum(int(row["pixel_count"]) for row in stereo_rows),
        "dropped_no_stereo_region_count": len(no_stereo_rows),
        "dropped_no_stereo_region_pixels": sum(int(row["pixel_count"]) for row in no_stereo_rows),
        "frames_with_drops": unique_frames,
        "frame_count_with_drops": len(unique_frames),
        "baseline_run": str(args.baseline_run),
        "filtered_run": str(args.filtered_run),
        "case_dir": str(args.case_dir),
        "sample_frames_dir": str(out / "sample_frames") if sampled_frames else "",
        "sampled_frames": sampled_frames,
        "review_status": "target_risk_needs_visual_review" if stereo_rows else ("needs_visual_review" if rows else "no_dropped_fragments"),
        "acceptance_note": (
            "g3_status is an automatic gate: red blocks promotion, warning requires visual review, "
            "pass means no missing baseline region, no stereo-bearing drop, and no contour-lost event. "
            "Inspect source/baseline/filtered overlays before promoting any drop policy."
        ),
    }
    (out / "g3_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    md_lines = [
        "# G3 ROI Drop Visual Review",
        "",
        f"- baseline_run: `{args.baseline_run}`",
        f"- filtered_run: `{args.filtered_run}`",
        f"- g3_status: `{status}`",
        f"- g3_red_reasons: {', '.join(red_reasons) if red_reasons else 'none'}",
        f"- g3_warning_reasons: {', '.join(warning_reasons) if warning_reasons else 'none'}",
        f"- runtime_roi_stereo_dropped_count: {runtime_drop_count}",
        f"- runtime_roi_stereo_dropped_pixels: {runtime_drop_pixels}",
        f"- contour_lost_event_count: {contour_lost_events}",
        f"- missing_baseline_region_count: {len(rows)}",
        f"- missing_baseline_region_pixels: {total_pixels}",
        f"- dropped_stereo_region_count: {len(stereo_rows)}",
        f"- dropped_no_stereo_region_count: {len(no_stereo_rows)}",
        f"- frames_with_drops: {', '.join(str(x) for x in unique_frames) if unique_frames else 'none'}",
        f"- review_status: {summary['review_status']}",
        "",
        "G3 asks whether `--color-contour-refresh-roi-drop-stereo-failed` removes only unsafe fragments, not real occlusion/reappear target contours. Stereo-bearing dropped regions are a hard red signal; no-stereo drops and oversized runtime drops are warnings that require visual review.",
        "",
        "| frame | region | pixels | bbox | stereo | priority | source | baseline | filtered |",
        "|---:|---:|---:|---|---:|---|---|---|---|",
    ]
    for row in rows:
        bbox = f"{row['bbox_x']},{row['bbox_y']},{row['bbox_w']},{row['bbox_h']}"
        source_link = row.get("sample_source_color") or row["source_color"]
        baseline_link = row.get("sample_baseline_overlay") or row["baseline_overlay"]
        filtered_link = row.get("sample_filtered_overlay") or row["filtered_overlay"]
        md_lines.append(
            f"| {row['frame_id']} | {row['baseline_region_id']} | {row['pixel_count']} | {bbox} | "
            f"{row['stereo_evidence']} | {row['review_priority']} | "
            f"[source]({source_link}) | [baseline]({baseline_link}) | "
            f"[filtered]({filtered_link}) |"
        )
    (out / "g3_drop_review.md").write_text("\n".join(md_lines) + "\n", encoding="utf-8")

    html_rows = []
    for row in rows:
        bbox = f"{row['bbox_x']},{row['bbox_y']},{row['bbox_w']},{row['bbox_h']}"
        source_link = row.get("sample_source_color") or row["source_color"]
        baseline_link = row.get("sample_baseline_overlay") or row["baseline_overlay"]
        filtered_link = row.get("sample_filtered_overlay") or row["filtered_overlay"]
        html_rows.append(
            "<tr>"
            f"<td>{html.escape(str(row['frame_id']))}</td>"
            f"<td>{html.escape(str(row['baseline_region_id']))}</td>"
            f"<td>{html.escape(str(row['pixel_count']))}</td>"
            f"<td>{html.escape(bbox)}</td>"
            f"<td>{html.escape(str(row['stereo_evidence']))}</td>"
            f"<td>{html.escape(str(row['review_priority']))}</td>"
            f"<td><img src=\"{html.escape(source_link)}\"></td>"
            f"<td><img src=\"{html.escape(baseline_link)}\"></td>"
            f"<td><img src=\"{html.escape(filtered_link)}\"></td>"
            "</tr>"
        )
    html_doc = f"""<!doctype html>
<meta charset="utf-8">
<title>G3 ROI Drop Visual Review</title>
<style>
body {{ font-family: Segoe UI, Arial, sans-serif; margin: 20px; }}
table {{ border-collapse: collapse; width: 100%; }}
th, td {{ border: 1px solid #ccc; padding: 6px; vertical-align: top; }}
img {{ max-width: 260px; height: auto; display: block; }}
.note {{ max-width: 980px; }}
</style>
<h1>G3 ROI Drop Visual Review</h1>
<div class="note">
<p>G3 status: <strong>{html.escape(status)}</strong>. Red blocks promotion; warning requires visual review; pass means no missing baseline region, no stereo-bearing drop, and no contour-lost event.</p>
<p>Runtime ROI stereo drops: {runtime_drop_count}; runtime pixels: {runtime_drop_pixels}; contour-lost events: {contour_lost_events}; missing baseline regions: {len(rows)}; missing pixels: {total_pixels}; stereo-bearing missing regions: {len(stereo_rows)}; no-stereo missing regions: {len(no_stereo_rows)}; frames: {', '.join(str(x) for x in unique_frames) if unique_frames else 'none'}.</p>
</div>
<table>
<tr><th>Frame</th><th>Region</th><th>Pixels</th><th>BBox</th><th>Stereo</th><th>Priority</th><th>Source color</th><th>Baseline overlay</th><th>Filtered overlay</th></tr>
{''.join(html_rows)}
</table>
"""
    (out / "g3_drop_review.html").write_text(html_doc, encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
