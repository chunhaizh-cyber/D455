#!/usr/bin/env python3
"""Validate centered 8x8 contour files and recompute temporal similarity."""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


MAGIC = b"B8X8CNT\0"
VERSION = 1
HEADER = struct.Struct("<8sIIQQIIiiIIIIIIIQI")


@dataclass(frozen=True)
class PackedContour:
    path: Path
    frame_id: int
    contour_id: int
    source_width: int
    source_height: int
    bbox_x: int
    bbox_y: int
    bbox_width: int
    bbox_height: int
    canvas_width: int
    canvas_height: int
    blocks_x: int
    blocks_y: int
    block_count: int
    foreground_pixels: int
    flags: int
    content_points: frozenset[tuple[int, int]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate .b8x8 files and recompute centered contour similarity."
    )
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--cpp-csv", type=Path)
    return parser.parse_args()


def round_up_to_eight(value: int) -> int:
    return max(8, ((max(1, value) + 7) // 8) * 8)


def read_contour(path: Path) -> PackedContour:
    data = path.read_bytes()
    if len(data) < HEADER.size:
        raise ValueError(f"{path}: file is smaller than the {HEADER.size}-byte header")
    fields = HEADER.unpack_from(data)
    (
        magic,
        version,
        header_bytes,
        frame_id,
        contour_id,
        source_width,
        source_height,
        bbox_x,
        bbox_y,
        bbox_width,
        bbox_height,
        canvas_width,
        canvas_height,
        blocks_x,
        blocks_y,
        block_count,
        foreground_pixels,
        flags,
    ) = fields

    if magic != MAGIC:
        raise ValueError(f"{path}: invalid magic {magic!r}")
    if version != VERSION:
        raise ValueError(f"{path}: unsupported version {version}")
    if header_bytes != HEADER.size:
        raise ValueError(f"{path}: header size {header_bytes} != {HEADER.size}")
    if source_width <= 0 or source_height <= 0:
        raise ValueError(f"{path}: invalid source dimensions")
    if bbox_width <= 0 or bbox_height <= 0:
        raise ValueError(f"{path}: empty contour bbox")
    if bbox_x < 0 or bbox_y < 0:
        raise ValueError(f"{path}: negative source bbox origin")
    if bbox_x + bbox_width > source_width or bbox_y + bbox_height > source_height:
        raise ValueError(f"{path}: source bbox exceeds source dimensions")
    if canvas_width % 8 or canvas_height % 8:
        raise ValueError(f"{path}: canvas dimensions are not multiples of 8")
    if canvas_width != round_up_to_eight(bbox_width):
        raise ValueError(f"{path}: canvas width is not the minimum padded width")
    if canvas_height != round_up_to_eight(bbox_height):
        raise ValueError(f"{path}: canvas height is not the minimum padded height")
    if blocks_x != canvas_width // 8 or blocks_y != canvas_height // 8:
        raise ValueError(f"{path}: block grid does not match canvas dimensions")
    if block_count != blocks_x * blocks_y:
        raise ValueError(f"{path}: block count does not match block grid")
    expected_bytes = header_bytes + block_count * 8
    if len(data) != expected_bytes:
        raise ValueError(f"{path}: file size {len(data)} != expected {expected_bytes}")
    if not flags & 1:
        raise ValueError(f"{path}: centered-contour flag is missing")

    words = struct.unpack_from(f"<{block_count}Q", data, header_bytes)
    offset_x = (canvas_width - bbox_width) // 2
    offset_y = (canvas_height - bbox_height) // 2
    content_points: set[tuple[int, int]] = set()
    packed_foreground = 0
    for block_index, packed in enumerate(words):
        block_x = block_index % blocks_x
        block_y = block_index // blocks_x
        packed_foreground += packed.bit_count()
        remaining = packed
        while remaining:
            lowest = remaining & -remaining
            bit = lowest.bit_length() - 1
            canvas_x = block_x * 8 + bit % 8
            canvas_y = block_y * 8 + bit // 8
            content_x = canvas_x - offset_x
            content_y = canvas_y - offset_y
            if not (0 <= content_x < bbox_width and 0 <= content_y < bbox_height):
                raise ValueError(f"{path}: nonzero padding bit at ({canvas_x}, {canvas_y})")
            content_points.add((content_x, content_y))
            remaining ^= lowest
    if packed_foreground != foreground_pixels:
        raise ValueError(
            f"{path}: foreground header {foreground_pixels} != popcount {packed_foreground}"
        )

    return PackedContour(
        path=path,
        frame_id=frame_id,
        contour_id=contour_id,
        source_width=source_width,
        source_height=source_height,
        bbox_x=bbox_x,
        bbox_y=bbox_y,
        bbox_width=bbox_width,
        bbox_height=bbox_height,
        canvas_width=canvas_width,
        canvas_height=canvas_height,
        blocks_x=blocks_x,
        blocks_y=blocks_y,
        block_count=block_count,
        foreground_pixels=foreground_pixels,
        flags=flags,
        content_points=frozenset(content_points),
    )


def compare_contours(first: PackedContour, second: PackedContour) -> dict[str, float | int]:
    width = round_up_to_eight(max(first.bbox_width, second.bbox_width))
    height = round_up_to_eight(max(first.bbox_height, second.bbox_height))
    first_dx = (width - first.bbox_width) // 2
    first_dy = (height - first.bbox_height) // 2
    second_dx = (width - second.bbox_width) // 2
    second_dy = (height - second.bbox_height) // 2
    first_centered = {(x + first_dx, y + first_dy) for x, y in first.content_points}
    second_centered = {(x + second_dx, y + second_dy) for x, y in second.content_points}
    intersection = len(first_centered & second_centered)
    union = len(first_centered | second_centered)
    changed = len(first_centered ^ second_centered)
    compare_pixels = width * height
    return {
        "compare_pixels": compare_pixels,
        "changed_pixels": changed,
        "similarity_percent": 100.0 * (compare_pixels - changed) / compare_pixels,
        "intersection_pixels": intersection,
        "union_pixels": union,
        "iou_percent": 100.0 * intersection / union if union else 100.0,
    }


def percentile(values: Iterable[float], percent: float) -> float | None:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def stats(values: Iterable[float]) -> dict[str, float | None]:
    materialized = [float(value) for value in values]
    return {
        "min": min(materialized) if materialized else None,
        "p50": percentile(materialized, 50.0),
        "p95": percentile(materialized, 95.0),
        "max": max(materialized) if materialized else None,
    }


def metric_columns(prefix: str, comparison: dict[str, float | int] | None) -> dict[str, object]:
    if comparison is None:
        return {
            f"{prefix}_available": 0,
            f"{prefix}_compare_pixels": "",
            f"{prefix}_changed_pixels": "",
            f"{prefix}_similarity_percent": "",
            f"{prefix}_intersection_pixels": "",
            f"{prefix}_union_pixels": "",
            f"{prefix}_iou_percent": "",
        }
    return {
        f"{prefix}_available": 1,
        f"{prefix}_compare_pixels": comparison["compare_pixels"],
        f"{prefix}_changed_pixels": comparison["changed_pixels"],
        f"{prefix}_similarity_percent": f'{comparison["similarity_percent"]:.6f}',
        f"{prefix}_intersection_pixels": comparison["intersection_pixels"],
        f"{prefix}_union_pixels": comparison["union_pixels"],
        f"{prefix}_iou_percent": f'{comparison["iou_percent"]:.6f}',
    }


def compare_cpp_csv(cpp_csv: Path, rows: list[dict[str, object]]) -> list[str]:
    if not cpp_csv.exists():
        return [f"missing C++ CSV: {cpp_csv}"]
    with cpp_csv.open("r", encoding="utf-8-sig", newline="") as stream:
        cpp_rows = {
            (int(row["frame_id"]), int(row["contour_id"])): row
            for row in csv.DictReader(stream)
        }
    errors: list[str] = []
    integer_suffixes = (
        "compare_pixels",
        "changed_pixels",
        "intersection_pixels",
        "union_pixels",
    )
    float_suffixes = ("similarity_percent", "iou_percent")
    for row in rows:
        frame_id = int(row["frame_id"])
        contour_id = int(row["contour_id"])
        cpp = cpp_rows.get((frame_id, contour_id))
        if cpp is None:
            errors.append(f"frame {frame_id} contour {contour_id}: missing C++ CSV row")
            continue
        for prefix in ("previous", "reference"):
            available_key = f"{prefix}_available"
            if int(cpp[available_key]) != int(row[available_key]):
                errors.append(f"frame {frame_id} contour {contour_id}: {available_key} mismatch")
                continue
            if int(row[available_key]) == 0:
                continue
            for suffix in integer_suffixes:
                key = f"{prefix}_{suffix}"
                if int(cpp[key]) != int(row[key]):
                    errors.append(f"frame {frame_id} contour {contour_id}: {key} mismatch")
            for suffix in float_suffixes:
                key = f"{prefix}_{suffix}"
                if abs(float(cpp[key]) - float(row[key])) > 1e-5:
                    errors.append(f"frame {frame_id} contour {contour_id}: {key} mismatch")
    expected_keys = {(int(row["frame_id"]), int(row["contour_id"])) for row in rows}
    extra = sorted(set(cpp_rows) - expected_keys)
    if extra:
        errors.append(f"C++ CSV has extra frame/contour rows: {extra[:10]}")
    return errors


def format_value(value: float | None, digits: int = 4) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def write_markdown(path: Path, summary: dict[str, object]) -> None:
    previous = summary["previous_frame"]
    reference = summary["reference_frame"]
    worst = summary.get("worst_previous_iou_frame")
    lines = [
        "# 二值轮廓居中压缩静态相似度分析",
        "",
        f"- 校验状态：`{summary['validation_status']}`",
        f"- 采样帧：{summary['sampled_frame_count']}",
        f"- 有效压缩轮廓：{summary['valid_contour_count']}",
        f"- 稳定 track 数：{summary['track_count']}",
        f"- 帧范围：{summary['first_frame_id']} - {summary['last_frame_id']}",
        f"- 压缩载荷：{summary['total_payload_bytes']} bytes",
        f"- 含文件头总大小：{summary['total_file_bytes']} bytes",
        f"- 相对居中 8-bit 二值图压缩比：{summary['centered_8bit_to_payload_ratio']:.3f}:1",
        "",
        "## 相邻帧",
        "",
        f"- Hamming 一致率 min/p50/p95：{format_value(previous['similarity_percent']['min'])}% / "
        f"{format_value(previous['similarity_percent']['p50'])}% / "
        f"{format_value(previous['similarity_percent']['p95'])}%",
        f"- 前景 IoU min/p50/p95：{format_value(previous['iou_percent']['min'])}% / "
        f"{format_value(previous['iou_percent']['p50'])}% / "
        f"{format_value(previous['iou_percent']['p95'])}%",
        f"- 完全相同帧对比例：{previous['exact_match_percent']:.4f}%",
        "",
        "## 首帧参考",
        "",
        f"- Hamming 一致率 min/p50/p95：{format_value(reference['similarity_percent']['min'])}% / "
        f"{format_value(reference['similarity_percent']['p50'])}% / "
        f"{format_value(reference['similarity_percent']['p95'])}%",
        f"- 前景 IoU min/p50/p95：{format_value(reference['iou_percent']['min'])}% / "
        f"{format_value(reference['iou_percent']['p50'])}% / "
        f"{format_value(reference['iou_percent']['p95'])}%",
        "",
        "## 最差相邻帧",
        "",
        (
            "- 无可比较帧。"
            if worst is None
            else f"- frame={worst['frame_id']}，contour_id={worst['contour_id']}，"
            f"previous_iou={worst['previous_iou_percent']:.4f}%，"
            f"changed_pixels={worst['previous_changed_pixels']}。"
        ),
        "",
        "说明：Hamming 一致率包含居中画布背景；前景 IoU 只衡量轮廓前景重叠，判断形状稳定性时应优先看 IoU。",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    input_dir = args.input_dir.resolve()
    output_dir = (args.output_dir or args.input_dir).resolve()
    cpp_csv = (args.cpp_csv or (input_dir / "binary_contour_similarity.csv")).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    files = sorted((input_dir / "frames").glob("*.b8x8"))
    if not files:
        raise SystemExit(f"No .b8x8 files found under {input_dir / 'frames'}")
    contours = sorted(
        (read_contour(path) for path in files),
        key=lambda item: (item.frame_id, item.contour_id),
    )
    keys = {(item.frame_id, item.contour_id) for item in contours}
    if len(keys) != len(contours):
        raise SystemExit("Duplicate frame_id/contour_id found in .b8x8 files")

    rows: list[dict[str, object]] = []
    references: dict[int, PackedContour] = {}
    previous_by_id: dict[int, PackedContour] = {}
    previous_comparisons: list[dict[str, float | int]] = []
    reference_comparisons: list[dict[str, float | int]] = []
    for contour in contours:
        previous = previous_by_id.get(contour.contour_id)
        reference = references.setdefault(contour.contour_id, contour)
        previous_comparison = compare_contours(previous, contour) if previous else None
        reference_comparison = compare_contours(reference, contour)
        if previous_comparison is not None:
            previous_comparisons.append(previous_comparison)
        reference_comparisons.append(reference_comparison)
        row: dict[str, object] = {
            "frame_id": contour.frame_id,
            "contour_id": contour.contour_id,
            "bbox_width": contour.bbox_width,
            "bbox_height": contour.bbox_height,
            "canvas_width": contour.canvas_width,
            "canvas_height": contour.canvas_height,
            "block_count": contour.block_count,
            "foreground_pixels": contour.foreground_pixels,
            "packed_file": contour.path.relative_to(input_dir).as_posix(),
            "previous_frame_id": previous.frame_id if previous else 0,
            "previous_gap_frames": contour.frame_id - previous.frame_id if previous else 0,
        }
        row.update(metric_columns("previous", previous_comparison))
        row["reference_frame_id"] = reference.frame_id
        row.update(metric_columns("reference", reference_comparison))
        rows.append(row)
        previous_by_id[contour.contour_id] = contour

    csv_path = output_dir / "binary_contour_similarity_recomputed.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    cpp_errors = compare_cpp_csv(cpp_csv, rows)
    frame_csv = input_dir / "binary_contour_frames.csv"
    frame_rows: list[dict[str, str]] = []
    if frame_csv.exists():
        with frame_csv.open("r", encoding="utf-8-sig", newline="") as stream:
            frame_rows = list(csv.DictReader(stream))
    else:
        cpp_errors.append(f"missing frame CSV: {frame_csv}")
    files_per_frame: dict[int, int] = {}
    for contour in contours:
        files_per_frame[contour.frame_id] = files_per_frame.get(contour.frame_id, 0) + 1
    for frame_row in frame_rows:
        frame_id = int(frame_row["frame_id"])
        if int(frame_row["contour_count"]) != files_per_frame.get(frame_id, 0):
            cpp_errors.append(f"frame {frame_id}: contour_count does not match packed files")
    total_payload = sum(item.block_count * 8 for item in contours)
    total_files = sum(item.path.stat().st_size for item in contours)
    centered_8bit_bytes = sum(item.canvas_width * item.canvas_height for item in contours)
    exact_pairs = sum(int(item["changed_pixels"] == 0) for item in previous_comparisons)
    worst_row = None
    comparable_rows = [row for row in rows if int(row["previous_available"]) == 1]
    if comparable_rows:
        worst = min(comparable_rows, key=lambda row: float(row["previous_iou_percent"]))
        worst_row = {
            "frame_id": int(worst["frame_id"]),
            "contour_id": int(worst["contour_id"]),
            "previous_iou_percent": float(worst["previous_iou_percent"]),
            "previous_changed_pixels": int(worst["previous_changed_pixels"]),
        }

    per_track: dict[str, object] = {}
    for contour_id in sorted(references):
        track_rows = [row for row in rows if int(row["contour_id"]) == contour_id]
        track_previous = [row for row in track_rows if int(row["previous_available"]) == 1]
        per_track[str(contour_id)] = {
            "frame_count": len(track_rows),
            "first_frame_id": int(track_rows[0]["frame_id"]),
            "last_frame_id": int(track_rows[-1]["frame_id"]),
            "previous_iou_percent": stats(float(row["previous_iou_percent"]) for row in track_previous),
            "previous_similarity_percent": stats(
                float(row["previous_similarity_percent"]) for row in track_previous
            ),
            "max_observation_gap_frames": max(
                (int(row["previous_gap_frames"]) for row in track_previous),
                default=0,
            ),
        }

    summary: dict[str, object] = {
        "format": "d455_centered_binary_contour_stability_v1",
        "validation_status": "pass" if not cpp_errors else "fail",
        "validation_errors": cpp_errors,
        "sampled_frame_count": len(frame_rows),
        "frames_with_contours": len(files_per_frame),
        "valid_contour_count": len(contours),
        "track_count": len(references),
        "first_frame_id": min(item.frame_id for item in contours),
        "last_frame_id": max(item.frame_id for item in contours),
        "total_payload_bytes": total_payload,
        "total_file_bytes": total_files,
        "centered_8bit_bytes": centered_8bit_bytes,
        "centered_8bit_to_payload_ratio": centered_8bit_bytes / total_payload,
        "previous_frame": {
            "pair_count": len(previous_comparisons),
            "changed_pixels": stats(item["changed_pixels"] for item in previous_comparisons),
            "similarity_percent": stats(item["similarity_percent"] for item in previous_comparisons),
            "iou_percent": stats(item["iou_percent"] for item in previous_comparisons),
            "exact_match_percent": 100.0 * exact_pairs / len(previous_comparisons)
            if previous_comparisons
            else 0.0,
        },
        "reference_frame": {
            "reference_mode": "first observation per contour_id",
            "changed_pixels": stats(item["changed_pixels"] for item in reference_comparisons),
            "similarity_percent": stats(item["similarity_percent"] for item in reference_comparisons),
            "iou_percent": stats(item["iou_percent"] for item in reference_comparisons),
        },
        "bbox_width": stats(item.bbox_width for item in contours),
        "bbox_height": stats(item.bbox_height for item in contours),
        "foreground_pixels": stats(item.foreground_pixels for item in contours),
        "block_count": stats(item.block_count for item in contours),
        "worst_previous_iou_frame": worst_row,
        "contour_count_per_frame": stats(int(row["contour_count"]) for row in frame_rows),
        "per_track": per_track,
    }
    summary_path = output_dir / "binary_contour_stability_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_markdown(output_dir / "binary_contour_stability_report.md", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["validation_status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
