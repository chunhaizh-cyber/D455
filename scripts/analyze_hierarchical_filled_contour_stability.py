#!/usr/bin/env python3
"""Compare fully filled contours through a square 8x8 image pyramid."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import struct
from collections import Counter
from pathlib import Path
from typing import Iterable

import numpy as np
from PIL import Image, ImageDraw


MAGIC = b"B8X8CNT\0"
VERSION = 1
HEADER = struct.Struct("<8sIIQQIIiiIIIIIIIQI")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fill contour holes and compare a power-of-two square image pyramid."
    )
    parser.add_argument("--input-dir", type=Path, action="append", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--contour-id", type=int, default=1)
    parser.add_argument("--min-side", type=int, default=8)
    return parser.parse_args()


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


def stats(values: Iterable[float]) -> dict[str, float | int | None]:
    materialized = [float(value) for value in values]
    return {
        "min": min(materialized) if materialized else None,
        "p05": percentile(materialized, 5.0),
        "p50": percentile(materialized, 50.0),
        "p95": percentile(materialized, 95.0),
        "max": max(materialized) if materialized else None,
        "gt99_count": sum(value > 99.0 for value in materialized),
        "gt99_percent": (
            100.0 * sum(value > 99.0 for value in materialized) / len(materialized)
            if materialized
            else 0.0
        ),
    }


def next_power_of_two(value: int) -> int:
    result = 1
    while result < value:
        result *= 2
    return result


def read_rows(input_dir: Path, contour_id: int) -> list[dict[str, str]]:
    csv_path = input_dir / "binary_contour_similarity.csv"
    with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = [
            row
            for row in csv.DictReader(stream)
            if int(row["contour_id"]) == contour_id
        ]
    if not rows:
        raise ValueError(f"{input_dir}: contour_id={contour_id} has no rows")
    return rows


def decode_content_mask(input_dir: Path, row: dict[str, str]) -> np.ndarray:
    path = input_dir / row["packed_file"]
    data = path.read_bytes()
    if len(data) < HEADER.size:
        raise ValueError(f"{path}: truncated header")
    fields = HEADER.unpack_from(data)
    (
        magic,
        version,
        header_bytes,
        _frame_id,
        _contour_id,
        _source_width,
        _source_height,
        _bbox_x,
        _bbox_y,
        bbox_width,
        bbox_height,
        canvas_width,
        canvas_height,
        blocks_x,
        blocks_y,
        block_count,
        foreground_pixels,
        _flags,
    ) = fields
    if magic != MAGIC or version != VERSION or header_bytes != HEADER.size:
        raise ValueError(f"{path}: unsupported .b8x8 header")
    expected_bytes = header_bytes + block_count * 8
    if len(data) != expected_bytes:
        raise ValueError(f"{path}: file size does not match block count")
    words = np.frombuffer(data, dtype="<u8", offset=header_bytes, count=block_count)
    bits = (
        np.unpackbits(words.view(np.uint8), bitorder="little")
        .reshape(blocks_y, blocks_x, 8, 8)
        .transpose(0, 2, 1, 3)
        .reshape(canvas_height, canvas_width)
    )
    if int(bits.sum()) != int(foreground_pixels):
        raise ValueError(f"{path}: packed foreground count mismatch")
    offset_x = (canvas_width - bbox_width) // 2
    offset_y = (canvas_height - bbox_height) // 2
    return bits[
        offset_y : offset_y + bbox_height,
        offset_x : offset_x + bbox_width,
    ].astype(np.uint8)


def fill_enclosed_holes(mask: np.ndarray) -> np.ndarray:
    padded = np.pad(mask * 255, 1).astype(np.uint8)
    image = Image.fromarray(padded, "L").copy()
    ImageDraw.floodfill(image, (0, 0), 128, thresh=0)
    flooded = np.asarray(image)
    holes = flooded[1:-1, 1:-1] == 0
    result = mask.astype(bool) | holes
    rows, columns = np.where(result)
    if not len(rows):
        raise ValueError("filled contour is empty")
    return result[rows.min() : rows.max() + 1, columns.min() : columns.max() + 1]


def center_in_square(mask: np.ndarray, side: int) -> np.ndarray:
    height, width = mask.shape
    if width > side or height > side:
        raise ValueError(f"contour {width}x{height} exceeds square side {side}")
    result = np.zeros((side, side), dtype=np.float32)
    offset_x = (side - width) // 2
    offset_y = (side - height) // 2
    result[offset_y : offset_y + height, offset_x : offset_x + width] = mask
    return result


def build_pyramid(square: np.ndarray, min_side: int) -> dict[int, np.ndarray]:
    levels: dict[int, np.ndarray] = {}
    current = square
    while current.shape[0] >= min_side:
        side = current.shape[0]
        levels[side] = current
        if side == min_side:
            break
        next_side = side // 2
        current = current.reshape(next_side, 2, next_side, 2).mean(axis=(1, 3))
    return levels


def binary_digest(occupancy: np.ndarray) -> tuple[str, np.ndarray, int]:
    binary = occupancy >= 0.5
    payload = np.packbits(binary.reshape(-1), bitorder="little").tobytes()
    return hashlib.sha256(payload).hexdigest(), binary, len(payload)


def occupancy_digest(occupancy: np.ndarray) -> tuple[str, int]:
    quantized = np.rint(occupancy * 255.0).astype(np.uint8)
    payload = quantized.tobytes()
    return hashlib.sha256(payload).hexdigest(), len(payload)


def compare_level(
    previous: np.ndarray,
    current: np.ndarray,
) -> tuple[float, float]:
    previous_binary = previous >= 0.5
    current_binary = current >= 0.5
    intersection = int((previous_binary & current_binary).sum())
    union = int((previous_binary | current_binary).sum())
    binary_iou = 100.0 * intersection / union if union else 100.0
    occupancy_similarity = 100.0 * (1.0 - float(np.abs(previous - current).mean()))
    return binary_iou, occupancy_similarity


def first_repeat(
    seen: dict[str, dict[str, object]],
    digest: str,
    current: dict[str, object],
) -> dict[str, object] | None:
    first = seen.get(digest)
    if first is None:
        seen[digest] = current
        return None
    return {"first": first, "repeat": current}


def format_value(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.4f}"


def write_report(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# 全填充轮廓正方形分级压缩稳定性测试",
        "",
        f"- 输入段数：{summary['input_run_count']}",
        f"- 轮廓 ID：{summary['contour_id']}",
        f"- 总帧数：{summary['frame_count']}",
        f"- 连续帧对：{summary['pair_count']}",
        f"- 基础正方形：{summary['base_side']}x{summary['base_side']}",
        f"- 分级：{', '.join(str(side) for side in summary['levels'])}",
        "",
        "所有封闭内部孔洞在压缩前均填为前景；轮廓原图位置被居中消除。",
        "",
        "| 边长 | 二值IoU p05 / p50 / p95 | 二值IoU >99% | 占用相似度 p50 |",
        "|---:|---:|---:|---:|",
    ]
    for side in summary["levels"]:
        result = summary["level_metrics"][str(side)]
        binary = result["binary_iou_percent"]
        occupancy = result["occupancy_similarity_percent"]
        lines.append(
            f"| {side} | {format_value(binary['p05'])}% / "
            f"{format_value(binary['p50'])}% / {format_value(binary['p95'])}% | "
            f"{binary['gt99_percent']:.4f}% | "
            f"{format_value(occupancy['p50'])}% |"
        )
    lines.extend(
        [
            "",
            "| 边长 | 二值载荷 | 相对512级压缩 | 二值不同值 | 重复帧 | 相邻精确重复 | 最大重复次数 | 占用不同值 |",
            "|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for side in summary["levels"]:
        result = summary["level_metrics"][str(side)]
        lines.append(
            f"| {side} | {result['binary_payload_bytes']} bytes | "
            f"{result['binary_compression_ratio_vs_base']:.0f}:1 | "
            f"{result['binary_unique_value_count']} | "
            f"{result['binary_repeated_frame_count']} | "
            f"{result['binary_exact_adjacent_pair_percent']:.4f}% | "
            f"{result['binary_max_repeat_frequency']} | "
            f"{result['occupancy_unique_value_count']} |"
        )
    lines.extend(
        [
            "",
            "二值值使用每格占用率 >=50% 后的黑白图并按位压缩；占用值使用0-255量化占用率。",
            "二值重复说明该分辨率下得到完全相同的粗轮廓，不表示原始高分辨率轮廓完全相同。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    if args.min_side < 8 or args.min_side & (args.min_side - 1):
        raise SystemExit("--min-side must be a power of two and at least 8")
    input_dirs = [path.resolve() for path in args.input_dir]
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    rows_by_run = [read_rows(path, args.contour_id) for path in input_dirs]
    max_extent = max(
        max(int(row["bbox_width"]), int(row["bbox_height"]))
        for rows in rows_by_run
        for row in rows
    )
    base_side = next_power_of_two(max(max_extent, args.min_side))
    levels: list[int] = []
    side = base_side
    while side >= args.min_side:
        levels.append(side)
        side //= 2

    comparisons: dict[int, list[dict[str, object]]] = {side: [] for side in levels}
    binary_counts: dict[int, Counter[str]] = {side: Counter() for side in levels}
    occupancy_counts: dict[int, Counter[str]] = {side: Counter() for side in levels}
    binary_seen: dict[int, dict[str, dict[str, object]]] = {side: {} for side in levels}
    occupancy_seen: dict[int, dict[str, dict[str, object]]] = {side: {} for side in levels}
    binary_first_repeat: dict[int, dict[str, object] | None] = {
        side: None for side in levels
    }
    occupancy_first_repeat: dict[int, dict[str, object] | None] = {
        side: None for side in levels
    }
    hash_rows: list[dict[str, object]] = []
    pair_rows: list[dict[str, object]] = []
    frame_count = 0

    for input_dir, rows in zip(input_dirs, rows_by_run):
        previous_frame_id = 0
        previous_pyramid: dict[int, np.ndarray] | None = None
        previous_binary_hashes: dict[int, str] = {}
        for row in rows:
            frame_id = int(row["frame_id"])
            filled = fill_enclosed_holes(decode_content_mask(input_dir, row))
            pyramid = build_pyramid(center_in_square(filled, base_side), args.min_side)
            frame_ref = {"run": input_dir.name, "frame_id": frame_id}
            for level_side in levels:
                occupancy = pyramid[level_side]
                binary_hash, binary, binary_bytes = binary_digest(occupancy)
                occupancy_hash, occupancy_bytes = occupancy_digest(occupancy)
                binary_counts[level_side][binary_hash] += 1
                occupancy_counts[level_side][occupancy_hash] += 1
                binary_repeat = first_repeat(
                    binary_seen[level_side], binary_hash, frame_ref
                )
                occupancy_repeat = first_repeat(
                    occupancy_seen[level_side], occupancy_hash, frame_ref
                )
                if binary_first_repeat[level_side] is None and binary_repeat is not None:
                    binary_first_repeat[level_side] = binary_repeat
                if occupancy_first_repeat[level_side] is None and occupancy_repeat is not None:
                    occupancy_first_repeat[level_side] = occupancy_repeat
                hash_rows.append(
                    {
                        "run": input_dir.name,
                        "frame_id": frame_id,
                        "contour_id": args.contour_id,
                        "side": level_side,
                        "binary_sha256": binary_hash,
                        "occupancy_sha256": occupancy_hash,
                        "binary_payload_bytes": binary_bytes,
                        "occupancy_payload_bytes": occupancy_bytes,
                        "binary_foreground_cells": int(binary.sum()),
                        "occupancy_sum": f"{float(occupancy.sum()):.6f}",
                    }
                )
                if previous_pyramid is not None and frame_id - previous_frame_id == 1:
                    binary_iou, occupancy_similarity = compare_level(
                        previous_pyramid[level_side], occupancy
                    )
                    comparison = {
                        "run": input_dir.name,
                        "previous_frame_id": previous_frame_id,
                        "frame_id": frame_id,
                        "side": level_side,
                        "binary_iou_percent": binary_iou,
                        "occupancy_similarity_percent": occupancy_similarity,
                        "binary_exact_match": int(
                            binary_hash == previous_binary_hashes[level_side]
                        ),
                    }
                    comparisons[level_side].append(comparison)
                    pair_rows.append(comparison)
            previous_frame_id = frame_id
            previous_pyramid = pyramid
            previous_binary_hashes = {
                level_side: binary_digest(pyramid[level_side])[0]
                for level_side in levels
            }
            frame_count += 1

    level_metrics: dict[str, object] = {}
    for level_side in levels:
        rows = comparisons[level_side]
        binary_counter = binary_counts[level_side]
        occupancy_counter = occupancy_counts[level_side]
        level_metrics[str(level_side)] = {
            "pair_count": len(rows),
            "binary_iou_percent": stats(
                float(row["binary_iou_percent"]) for row in rows
            ),
            "occupancy_similarity_percent": stats(
                float(row["occupancy_similarity_percent"]) for row in rows
            ),
            "binary_exact_adjacent_pair_count": sum(
                int(row["binary_exact_match"]) for row in rows
            ),
            "binary_exact_adjacent_pair_percent": (
                100.0
                * sum(int(row["binary_exact_match"]) for row in rows)
                / len(rows)
                if rows
                else 0.0
            ),
            "binary_payload_bytes": level_side * level_side // 8,
            "occupancy_payload_bytes": level_side * level_side,
            "binary_compression_ratio_vs_base": (
                base_side * base_side / (level_side * level_side)
            ),
            "binary_unique_value_count": len(binary_counter),
            "binary_repeated_value_count": sum(
                count > 1 for count in binary_counter.values()
            ),
            "binary_repeated_frame_count": frame_count - len(binary_counter),
            "binary_max_repeat_frequency": max(binary_counter.values()),
            "binary_first_repeat": binary_first_repeat[level_side],
            "occupancy_unique_value_count": len(occupancy_counter),
            "occupancy_repeated_value_count": sum(
                count > 1 for count in occupancy_counter.values()
            ),
            "occupancy_repeated_frame_count": frame_count - len(occupancy_counter),
            "occupancy_max_repeat_frequency": max(occupancy_counter.values()),
            "occupancy_first_repeat": occupancy_first_repeat[level_side],
        }

    summary: dict[str, object] = {
        "format": "d455_hierarchical_filled_contour_stability_v1",
        "definition": {
            "interior": "all enclosed holes filled before normalization",
            "square": "centered next-power-of-two square",
            "pyramid": "2x2 area pooling down to min_side",
            "binary": "occupancy >= 0.5, row-major bit-packed",
            "occupancy": "0-255 rounded occupancy per cell",
        },
        "input_run_count": len(input_dirs),
        "input_runs": [path.name for path in input_dirs],
        "contour_id": args.contour_id,
        "frame_count": frame_count,
        "pair_count": len(comparisons[levels[0]]),
        "base_side": base_side,
        "min_side": args.min_side,
        "levels": levels,
        "level_metrics": level_metrics,
    }

    with (output_dir / "hierarchical_feature_hashes.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(hash_rows[0].keys()))
        writer.writeheader()
        writer.writerows(hash_rows)
    with (output_dir / "hierarchical_pair_metrics.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(pair_rows[0].keys()))
        writer.writeheader()
        writer.writerows(pair_rows)
    (output_dir / "hierarchical_filled_contour_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    write_report(output_dir / "hierarchical_filled_contour_report.md", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
