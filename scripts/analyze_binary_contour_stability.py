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
from typing import Callable, Iterable


MAGIC = b"B8X8CNT\0"
VERSION = 1
HEADER = struct.Struct("<8sIIQQIIiiIIIIIIIQI")
TARGET_SIMILARITY_PERCENT = 99.0


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
    parser.add_argument(
        "--require-target-pass",
        action="store_true",
        help="Return exit code 2 when the strict continuous-frame 99%% target fails.",
    )
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
    first_center_x = first.bbox_x + (first.bbox_width - 1) * 0.5
    first_center_y = first.bbox_y + (first.bbox_height - 1) * 0.5
    second_center_x = second.bbox_x + (second.bbox_width - 1) * 0.5
    second_center_y = second.bbox_y + (second.bbox_height - 1) * 0.5
    center_shift_pixels = math.hypot(
        first_center_x - second_center_x,
        first_center_y - second_center_y,
    )
    source_diagonal = math.hypot(
        max(first.source_width, second.source_width),
        max(first.source_height, second.source_height),
    )
    return {
        "compare_pixels": compare_pixels,
        "changed_pixels": changed,
        "similarity_percent": 100.0 * (compare_pixels - changed) / compare_pixels,
        "intersection_pixels": intersection,
        "union_pixels": union,
        "iou_percent": 100.0 * intersection / union if union else 100.0,
        "center_shift_pixels": center_shift_pixels,
        "position_similarity_percent": (
            100.0 * max(0.0, 1.0 - center_shift_pixels / source_diagonal)
            if source_diagonal > 0.0
            else 0.0
        ),
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
        "p05": percentile(materialized, 5.0),
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
            f"{prefix}_center_shift_pixels": "",
            f"{prefix}_position_similarity_percent": "",
        }
    return {
        f"{prefix}_available": 1,
        f"{prefix}_compare_pixels": comparison["compare_pixels"],
        f"{prefix}_changed_pixels": comparison["changed_pixels"],
        f"{prefix}_similarity_percent": f'{comparison["similarity_percent"]:.6f}',
        f"{prefix}_intersection_pixels": comparison["intersection_pixels"],
        f"{prefix}_union_pixels": comparison["union_pixels"],
        f"{prefix}_iou_percent": f'{comparison["iou_percent"]:.6f}',
        f"{prefix}_center_shift_pixels": f'{comparison["center_shift_pixels"]:.6f}',
        f"{prefix}_position_similarity_percent": (
            f'{comparison["position_similarity_percent"]:.6f}'
        ),
    }


def read_cpp_csv(cpp_csv: Path) -> dict[tuple[int, int], dict[str, str]]:
    if not cpp_csv.exists():
        return {}
    with cpp_csv.open("r", encoding="utf-8-sig", newline="") as stream:
        return {
            (int(row["frame_id"]), int(row["contour_id"])): row
            for row in csv.DictReader(stream)
        }


def compare_cpp_csv(cpp_csv: Path, rows: list[dict[str, object]]) -> list[str]:
    if not cpp_csv.exists():
        return [f"missing C++ CSV: {cpp_csv}"]
    cpp_rows = read_cpp_csv(cpp_csv)
    errors: list[str] = []
    integer_suffixes = (
        "compare_pixels",
        "changed_pixels",
        "intersection_pixels",
        "union_pixels",
    )
    float_suffixes = (
        "similarity_percent",
        "iou_percent",
        "center_shift_pixels",
        "position_similarity_percent",
    )
    base_integer_keys = (
        "source_width",
        "source_height",
        "bbox_x",
        "bbox_y",
        "bbox_width",
        "bbox_height",
        "foreground_pixels",
        "mean_depth_mm",
        "observed_depth_min_mm",
        "observed_depth_max_mm",
        "previous_frame_id",
        "previous_gap_frames",
        "reference_frame_id",
    )
    base_float_keys = (
        "bbox_area_percent",
        "foreground_area_percent",
        "center_radius_percent",
    )
    for row in rows:
        frame_id = int(row["frame_id"])
        contour_id = int(row["contour_id"])
        cpp = cpp_rows.get((frame_id, contour_id))
        if cpp is None:
            errors.append(f"frame {frame_id} contour {contour_id}: missing C++ CSV row")
            continue
        for key in base_integer_keys:
            if int(float(cpp[key])) != int(row[key]):
                errors.append(f"frame {frame_id} contour {contour_id}: {key} mismatch")
        for key in base_float_keys:
            if abs(float(cpp[key]) - float(row[key])) > 1e-5:
                errors.append(f"frame {frame_id} contour {contour_id}: {key} mismatch")
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


def target_metrics(rows: list[dict[str, object]]) -> dict[str, object]:
    shape_values = [float(row["previous_iou_percent"]) for row in rows]
    position_values = [float(row["previous_position_similarity_percent"]) for row in rows]
    shape_pass_count = sum(value > TARGET_SIMILARITY_PERCENT for value in shape_values)
    position_pass_count = sum(value > TARGET_SIMILARITY_PERCENT for value in position_values)
    joint_pass_count = sum(
        shape > TARGET_SIMILARITY_PERCENT and position > TARGET_SIMILARITY_PERCENT
        for shape, position in zip(shape_values, position_values)
    )
    pair_count = len(rows)

    def pass_percent(count: int) -> float:
        return 100.0 * count / pair_count if pair_count else 0.0

    return {
        "pair_count": pair_count,
        "threshold_percent": TARGET_SIMILARITY_PERCENT,
        "comparison_operator": ">",
        "shape_iou_percent": stats(shape_values),
        "position_similarity_percent": stats(position_values),
        "center_shift_pixels": stats(
            float(row["previous_center_shift_pixels"]) for row in rows
        ),
        "shape_pass_count": shape_pass_count,
        "shape_pass_percent": pass_percent(shape_pass_count),
        "position_pass_count": position_pass_count,
        "position_pass_percent": pass_percent(position_pass_count),
        "joint_pass_count": joint_pass_count,
        "joint_pass_percent": pass_percent(joint_pass_count),
        "status": (
            "no_data"
            if pair_count == 0
            else "pass"
            if joint_pass_count == pair_count
            else "fail"
        ),
    }


def relation_buckets(
    rows: list[dict[str, object]],
    labels: tuple[str, ...],
    selector: Callable[[dict[str, object]], str],
) -> dict[str, dict[str, object]]:
    bucket_rows: dict[str, list[dict[str, object]]] = {label: [] for label in labels}
    for row in rows:
        bucket_rows[selector(row)].append(row)
    return {label: target_metrics(bucket_rows[label]) for label in labels}


def average_ranks(values: list[float]) -> list[float]:
    ranked = [0.0] * len(values)
    ordered = sorted(range(len(values)), key=values.__getitem__)
    start = 0
    while start < len(ordered):
        end = start + 1
        while end < len(ordered) and values[ordered[end]] == values[ordered[start]]:
            end += 1
        average_rank = 0.5 * (start + end - 1)
        for ordered_index in range(start, end):
            ranked[ordered[ordered_index]] = average_rank
        start = end
    return ranked


def pearson(values_x: list[float], values_y: list[float]) -> float | None:
    if len(values_x) < 2 or len(values_x) != len(values_y):
        return None
    mean_x = sum(values_x) / len(values_x)
    mean_y = sum(values_y) / len(values_y)
    centered_x = [value - mean_x for value in values_x]
    centered_y = [value - mean_y for value in values_y]
    denominator = math.sqrt(
        sum(value * value for value in centered_x)
        * sum(value * value for value in centered_y)
    )
    if denominator == 0.0:
        return None
    return sum(x * y for x, y in zip(centered_x, centered_y)) / denominator


def correlation_metrics(
    rows: list[dict[str, object]],
    feature: Callable[[dict[str, object]], float | None],
) -> dict[str, object]:
    samples: list[tuple[float, float, float]] = []
    for row in rows:
        feature_value = feature(row)
        if feature_value is None or not math.isfinite(feature_value):
            continue
        samples.append(
            (
                feature_value,
                float(row["previous_iou_percent"]),
                float(row["previous_position_similarity_percent"]),
            )
        )
    feature_values = [sample[0] for sample in samples]
    shape_values = [sample[1] for sample in samples]
    position_values = [sample[2] for sample in samples]
    feature_ranks = average_ranks(feature_values)
    return {
        "pair_count": len(samples),
        "shape_iou_percent": {
            "pearson": pearson(feature_values, shape_values),
            "spearman": pearson(feature_ranks, average_ranks(shape_values)),
        },
        "position_similarity_percent": {
            "pearson": pearson(feature_values, position_values),
            "spearman": pearson(feature_ranks, average_ranks(position_values)),
        },
    }


def write_bucket_table(
    lines: list[str],
    title: str,
    buckets: dict[str, dict[str, object]],
) -> None:
    lines.extend(
        [
            f"### {title}",
            "",
            "| 分桶 | 连续帧对 | 轮廓 IoU min / p05 / p50 | 位置相似度 min / p05 / p50 | 轮廓 >99% | 位置 >99% | 同时 >99% |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for label, result in buckets.items():
        shape = result["shape_iou_percent"]
        position = result["position_similarity_percent"]
        lines.append(
            f"| {label} | {result['pair_count']} | "
            f"{format_value(shape['min'])}% / {format_value(shape['p05'])}% / "
            f"{format_value(shape['p50'])}% | "
            f"{format_value(position['min'])}% / {format_value(position['p05'])}% / "
            f"{format_value(position['p50'])}% | "
            f"{result['shape_pass_percent']:.4f}% | "
            f"{result['position_pass_percent']:.4f}% | "
            f"{result['joint_pass_percent']:.4f}% |"
        )
    lines.append("")


def write_markdown(path: Path, summary: dict[str, object]) -> None:
    previous = summary["previous_frame"]
    reference = summary["reference_frame"]
    target = summary["continuous_frame_target"]
    worst = summary.get("worst_previous_iou_frame")
    lines = [
        "# 静止存在连续帧轮廓与位置 99% 目标分析",
        "",
        f"- 校验状态：`{summary['validation_status']}`",
        f"- 99% 双门禁：`{target['status']}`",
        f"- 硬门禁对象：同一 observation_id 且 previous_gap_frames == 1，共 {target['pair_count']} 对",
        f"- 轮廓 IoU 严格 >99%：{target['shape_pass_count']}/{target['pair_count']} "
        f"({target['shape_pass_percent']:.4f}%)",
        f"- 位置相似度严格 >99%：{target['position_pass_count']}/{target['pair_count']} "
        f"({target['position_pass_percent']:.4f}%)",
        f"- 两项同时严格 >99%：{target['joint_pass_count']}/{target['pair_count']} "
        f"({target['joint_pass_percent']:.4f}%)",
        f"- 采样帧：{summary['sampled_frame_count']}",
        f"- 有效压缩轮廓：{summary['valid_contour_count']}",
        f"- 稳定 track 数：{summary['track_count']}",
        f"- 帧范围：{summary['first_frame_id']} - {summary['last_frame_id']}",
        f"- 压缩载荷：{summary['total_payload_bytes']} bytes",
        f"- 含文件头总大小：{summary['total_file_bytes']} bytes",
        f"- 相对居中 8-bit 二值图压缩比：{summary['centered_8bit_to_payload_ratio']:.3f}:1",
        "",
        "## 连续帧硬门禁分布",
        "",
        f"- 轮廓 IoU min/p05/p50/p95：{format_value(target['shape_iou_percent']['min'])}% / "
        f"{format_value(target['shape_iou_percent']['p05'])}% / "
        f"{format_value(target['shape_iou_percent']['p50'])}% / "
        f"{format_value(target['shape_iou_percent']['p95'])}%",
        f"- 位置相似度 min/p05/p50/p95："
        f"{format_value(target['position_similarity_percent']['min'])}% / "
        f"{format_value(target['position_similarity_percent']['p05'])}% / "
        f"{format_value(target['position_similarity_percent']['p50'])}% / "
        f"{format_value(target['position_similarity_percent']['p95'])}%",
        f"- bbox 中心位移 min/p05/p50/p95：{format_value(target['center_shift_pixels']['min'])} / "
        f"{format_value(target['center_shift_pixels']['p05'])} / "
        f"{format_value(target['center_shift_pixels']['p50'])} / "
        f"{format_value(target['center_shift_pixels']['p95'])} px",
        f"- 所有同 ID 相邻记录（含非连续）Hamming 完全相同比例："
        f"{previous['exact_match_percent']:.4f}%",
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
        "## 大小、距离、画面位置关系",
        "",
    ]
    write_bucket_table(lines, "按当前轮廓前景面积占全画面比例", summary["relation_buckets"]["size"])
    write_bucket_table(lines, "按当前轮廓平均深度", summary["relation_buckets"]["distance"])
    write_bucket_table(lines, "按当前轮廓中心距画面中心的径向比例", summary["relation_buckets"]["center_radius"])
    lines.extend(
        [
            "### 相关系数",
            "",
            "| 特征 | 有效帧对 | 轮廓 IoU Pearson / Spearman | 位置相似度 Pearson / Spearman |",
            "|---|---:|---:|---:|",
        ]
    )
    correlation_labels = {
        "log10_foreground_pixels": "log10(前景像素数)",
        "mean_depth_mm": "平均深度 mm（仅 >0）",
        "center_radius_percent": "中心径向比例 %",
    }
    for key, label in correlation_labels.items():
        result = summary["correlations"][key]
        shape = result["shape_iou_percent"]
        position = result["position_similarity_percent"]
        lines.append(
            f"| {label} | {result['pair_count']} | "
            f"{format_value(shape['pearson'])} / {format_value(shape['spearman'])} | "
            f"{format_value(position['pearson'])} / {format_value(position['spearman'])} |"
        )
    lines.extend(
        [
            "",
            "说明：轮廓相似度使用居中后的前景 IoU；位置相似度为 "
            "100 * (1 - bbox 中心位移 / 原图对角线)。640x480 的对角线为 800px，"
            "因此严格 >99% 等价于中心位移严格 <8px。两项不可互相补偿。",
            "",
            "分桶使用当前帧特征；相关系数只描述本次固定回放中的关系，不证明因果。"
            "Hamming 一致率包含居中画布背景，不作为 99% 轮廓硬门禁。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    input_dir = args.input_dir.resolve()
    output_dir = (args.output_dir or args.input_dir).resolve()
    cpp_csv = (args.cpp_csv or (input_dir / "binary_contour_similarity.csv")).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    cpp_rows = read_cpp_csv(cpp_csv)

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
        cpp_row = cpp_rows.get((contour.frame_id, contour.contour_id), {})

        def cpp_int(key: str) -> int:
            value = cpp_row.get(key, "")
            return int(float(value)) if value not in (None, "") else 0

        previous = previous_by_id.get(contour.contour_id)
        reference = references.setdefault(contour.contour_id, contour)
        previous_comparison = compare_contours(previous, contour) if previous else None
        reference_comparison = compare_contours(reference, contour)
        if previous_comparison is not None:
            previous_comparisons.append(previous_comparison)
        reference_comparisons.append(reference_comparison)
        source_pixels = max(1, contour.source_width * contour.source_height)
        bbox_area_percent = 100.0 * contour.bbox_width * contour.bbox_height / source_pixels
        foreground_area_percent = 100.0 * contour.foreground_pixels / source_pixels
        frame_center_x = (contour.source_width - 1) * 0.5
        frame_center_y = (contour.source_height - 1) * 0.5
        contour_center_x = contour.bbox_x + (contour.bbox_width - 1) * 0.5
        contour_center_y = contour.bbox_y + (contour.bbox_height - 1) * 0.5
        half_diagonal = 0.5 * math.hypot(contour.source_width, contour.source_height)
        center_radius_percent = (
            100.0
            * math.hypot(contour_center_x - frame_center_x, contour_center_y - frame_center_y)
            / half_diagonal
            if half_diagonal > 0.0
            else 0.0
        )
        row: dict[str, object] = {
            "frame_id": contour.frame_id,
            "contour_id": contour.contour_id,
            "source_width": contour.source_width,
            "source_height": contour.source_height,
            "bbox_x": contour.bbox_x,
            "bbox_y": contour.bbox_y,
            "bbox_width": contour.bbox_width,
            "bbox_height": contour.bbox_height,
            "canvas_width": contour.canvas_width,
            "canvas_height": contour.canvas_height,
            "block_count": contour.block_count,
            "foreground_pixels": contour.foreground_pixels,
            "bbox_area_percent": f"{bbox_area_percent:.6f}",
            "foreground_area_percent": f"{foreground_area_percent:.6f}",
            "mean_depth_mm": cpp_int("mean_depth_mm"),
            "observed_depth_min_mm": cpp_int("observed_depth_min_mm"),
            "observed_depth_max_mm": cpp_int("observed_depth_max_mm"),
            "center_radius_percent": f"{center_radius_percent:.6f}",
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
    continuous_rows = [row for row in comparable_rows if int(row["previous_gap_frames"]) == 1]
    if continuous_rows:
        worst = min(continuous_rows, key=lambda row: float(row["previous_iou_percent"]))
        worst_row = {
            "frame_id": int(worst["frame_id"]),
            "contour_id": int(worst["contour_id"]),
            "previous_iou_percent": float(worst["previous_iou_percent"]),
            "previous_position_similarity_percent": float(
                worst["previous_position_similarity_percent"]
            ),
            "previous_center_shift_pixels": float(worst["previous_center_shift_pixels"]),
            "previous_changed_pixels": int(worst["previous_changed_pixels"]),
        }

    per_track: dict[str, object] = {}
    for contour_id in sorted(references):
        track_rows = [row for row in rows if int(row["contour_id"]) == contour_id]
        track_previous = [row for row in track_rows if int(row["previous_available"]) == 1]
        track_continuous = [row for row in track_previous if int(row["previous_gap_frames"]) == 1]
        per_track[str(contour_id)] = {
            "frame_count": len(track_rows),
            "first_frame_id": int(track_rows[0]["frame_id"]),
            "last_frame_id": int(track_rows[-1]["frame_id"]),
            "previous_iou_percent": stats(float(row["previous_iou_percent"]) for row in track_previous),
            "previous_similarity_percent": stats(
                float(row["previous_similarity_percent"]) for row in track_previous
            ),
            "previous_position_similarity_percent": stats(
                float(row["previous_position_similarity_percent"]) for row in track_previous
            ),
            "continuous_frame_target": target_metrics(track_continuous),
            "max_observation_gap_frames": max(
                (int(row["previous_gap_frames"]) for row in track_previous),
                default=0,
            ),
        }

    size_buckets = relation_buckets(
        continuous_rows,
        ("<1%", "1%-5%", ">=5%"),
        lambda row: (
            "<1%"
            if float(row["foreground_area_percent"]) < 1.0
            else "1%-5%"
            if float(row["foreground_area_percent"]) < 5.0
            else ">=5%"
        ),
    )
    distance_buckets = relation_buckets(
        continuous_rows,
        ("<=1500mm", "1500-2500mm", ">2500mm", "unknown"),
        lambda row: (
            "unknown"
            if float(row["mean_depth_mm"]) <= 0.0
            else "<=1500mm"
            if float(row["mean_depth_mm"]) <= 1500.0
            else "1500-2500mm"
            if float(row["mean_depth_mm"]) <= 2500.0
            else ">2500mm"
        ),
    )
    center_buckets = relation_buckets(
        continuous_rows,
        ("<=33%", "33%-66%", ">66%"),
        lambda row: (
            "<=33%"
            if float(row["center_radius_percent"]) <= 33.0
            else "33%-66%"
            if float(row["center_radius_percent"]) <= 66.0
            else ">66%"
        ),
    )
    correlations = {
        "log10_foreground_pixels": correlation_metrics(
            continuous_rows,
            lambda row: math.log10(float(row["foreground_pixels"]))
            if float(row["foreground_pixels"]) > 0.0
            else None,
        ),
        "mean_depth_mm": correlation_metrics(
            continuous_rows,
            lambda row: float(row["mean_depth_mm"])
            if float(row["mean_depth_mm"]) > 0.0
            else None,
        ),
        "center_radius_percent": correlation_metrics(
            continuous_rows,
            lambda row: float(row["center_radius_percent"]),
        ),
    }

    summary: dict[str, object] = {
        "format": "d455_centered_binary_contour_stability_v2",
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
            "position_similarity_percent": stats(
                item["position_similarity_percent"] for item in previous_comparisons
            ),
            "center_shift_pixels": stats(
                item["center_shift_pixels"] for item in previous_comparisons
            ),
            "exact_match_percent": 100.0 * exact_pairs / len(previous_comparisons)
            if previous_comparisons
            else 0.0,
        },
        "reference_frame": {
            "reference_mode": "first observation per contour_id",
            "changed_pixels": stats(item["changed_pixels"] for item in reference_comparisons),
            "similarity_percent": stats(item["similarity_percent"] for item in reference_comparisons),
            "iou_percent": stats(item["iou_percent"] for item in reference_comparisons),
            "position_similarity_percent": stats(
                item["position_similarity_percent"] for item in reference_comparisons
            ),
        },
        "continuous_frame_target": target_metrics(continuous_rows),
        "relation_buckets": {
            "size": size_buckets,
            "distance": distance_buckets,
            "center_radius": center_buckets,
        },
        "correlations": correlations,
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
    if summary["validation_status"] != "pass":
        return 1
    if args.require_target_pass and summary["continuous_frame_target"]["status"] != "pass":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
