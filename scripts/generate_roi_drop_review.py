#!/usr/bin/env python3
import argparse
import csv
import html
import json
from pathlib import Path


def read_json(path):
    with Path(path).open("r", encoding="utf-8-sig") as f:
        return json.load(f)


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-run", required=True, help="Run without drop filtering, such as candidate_0034.")
    parser.add_argument("--filtered-run", required=True, help="Run with drop filtering, such as candidate_0035.")
    parser.add_argument("--case-dir", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--match-iou-min", type=float, default=0.05)
    parser.add_argument("--match-center-max-px", type=float, default=120.0)
    args = parser.parse_args()

    baseline = metadata_by_frame(args.baseline_run)
    filtered = metadata_by_frame(args.filtered_run)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rows = []
    for frame_id in sorted(set(baseline) & set(filtered)):
        base_item = baseline[frame_id]
        filtered_item = filtered[frame_id]
        base_regions = base_item["meta"].get("color_regions", [])
        filtered_regions = filtered_item["meta"].get("color_regions", [])
        for region in base_regions:
            if int(region.get("estimated_distance_mm") or 0) > 0:
                continue
            if is_matching_region(region, filtered_regions, args.match_iou_min, args.match_center_max_px):
                continue
            bbox = region.get("bbox_2d", [0, 0, 0, 0])
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
                "estimated_distance_mm": region.get("estimated_distance_mm", 0),
                "matched_stereo_points": region.get("matched_stereo_points", 0),
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
        "source_color",
        "baseline_overlay",
        "filtered_overlay",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    total_pixels = sum(int(row["pixel_count"]) for row in rows)
    unique_frames = sorted({int(row["frame_id"]) for row in rows})
    summary = {
        "dropped_fragment_count": len(rows),
        "dropped_fragment_pixels": total_pixels,
        "frames_with_drops": unique_frames,
        "frame_count_with_drops": len(unique_frames),
        "baseline_run": str(args.baseline_run),
        "filtered_run": str(args.filtered_run),
        "case_dir": str(args.case_dir),
        "review_status": "needs_visual_review" if rows else "no_dropped_fragments",
        "acceptance_note": (
            "G3 is not automatically pass/fail. Inspect source/baseline/filtered overlays "
            "to decide whether dropped fragments are only revealed background/no-stereo fragments."
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
        f"- dropped_fragment_count: {len(rows)}",
        f"- dropped_fragment_pixels: {total_pixels}",
        f"- frames_with_drops: {', '.join(str(x) for x in unique_frames) if unique_frames else 'none'}",
        "- review_status: needs_visual_review" if rows else "- review_status: no_dropped_fragments",
        "",
        "G3 asks whether `--color-contour-refresh-roi-drop-stereo-failed` removes only no-stereo fragments, not real occlusion/reappear target contours.",
        "",
        "| frame | region | pixels | bbox | source | baseline | filtered |",
        "|---:|---:|---:|---|---|---|---|",
    ]
    for row in rows:
        bbox = f"{row['bbox_x']},{row['bbox_y']},{row['bbox_w']},{row['bbox_h']}"
        md_lines.append(
            f"| {row['frame_id']} | {row['baseline_region_id']} | {row['pixel_count']} | {bbox} | "
            f"[source]({row['source_color']}) | [baseline]({row['baseline_overlay']}) | "
            f"[filtered]({row['filtered_overlay']}) |"
        )
    (out / "g3_drop_review.md").write_text("\n".join(md_lines) + "\n", encoding="utf-8")

    html_rows = []
    for row in rows:
        bbox = f"{row['bbox_x']},{row['bbox_y']},{row['bbox_w']},{row['bbox_h']}"
        html_rows.append(
            "<tr>"
            f"<td>{html.escape(str(row['frame_id']))}</td>"
            f"<td>{html.escape(str(row['baseline_region_id']))}</td>"
            f"<td>{html.escape(str(row['pixel_count']))}</td>"
            f"<td>{html.escape(bbox)}</td>"
            f"<td><img src=\"{html.escape(row['source_color'])}\"></td>"
            f"<td><img src=\"{html.escape(row['baseline_overlay'])}\"></td>"
            f"<td><img src=\"{html.escape(row['filtered_overlay'])}\"></td>"
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
<p>Review whether dropped ROI fragments are only no-stereo background fragments. This report is an evidence package, not an automatic pass/fail decision.</p>
<p>Dropped fragments: {len(rows)}; pixels: {total_pixels}; frames: {', '.join(str(x) for x in unique_frames) if unique_frames else 'none'}.</p>
</div>
<table>
<tr><th>Frame</th><th>Region</th><th>Pixels</th><th>BBox</th><th>Source color</th><th>Baseline overlay</th><th>Filtered overlay</th></tr>
{''.join(html_rows)}
</table>
"""
    (out / "g3_drop_review.html").write_text(html_doc, encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
