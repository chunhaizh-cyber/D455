#!/usr/bin/env python3
"""Compare every contour track through the filled square image pyramid."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable

import numpy as np

import analyze_hierarchical_filled_contour_stability as hierarchy


BUCKET_SHARED_LONG = "shared_long"
BUCKET_SINGLE_RUN_LONG = "single_run_long"
BUCKET_MEDIUM_SHORT = "medium_short"
BUCKET_ALL = "all_contours"
BUCKET_ORDER = [
    BUCKET_SHARED_LONG,
    BUCKET_SINGLE_RUN_LONG,
    BUCKET_MEDIUM_SHORT,
    BUCKET_ALL,
]
BUCKET_LABELS = {
    BUCKET_SHARED_LONG: "两段长期持续",
    BUCKET_SINGLE_RUN_LONG: "单段长期持续",
    BUCKET_MEDIUM_SHORT: "中短期/接替轮廓",
    BUCKET_ALL: "全部轮廓",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fill holes and compare all contour tracks through a power-of-two "
            "square image pyramid."
        )
    )
    parser.add_argument("--input-dir", type=Path, action="append", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--min-side", type=int, default=8)
    parser.add_argument(
        "--long-track-percent",
        type=float,
        default=90.0,
        help="Minimum percent of a run covered by a long track (default: 90).",
    )
    return parser.parse_args()


def read_all_rows(input_dir: Path) -> tuple[list[dict[str, str]], int]:
    csv_path = input_dir / "binary_contour_similarity.csv"
    with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"{csv_path}: no contour rows")
    rows.sort(key=lambda row: (int(row["contour_id"]), int(row["frame_id"])))
    return rows, len({int(row["frame_id"]) for row in rows})


def make_levels(base_side: int, min_side: int) -> list[int]:
    levels: list[int] = []
    side = base_side
    while side >= min_side:
        levels.append(side)
        side //= 2
    return levels


def is_consecutive(row: dict[str, str], previous_frame_id: int | None) -> bool:
    if previous_frame_id is None:
        return False
    return (
        row.get("previous_available") == "1"
        and int(row.get("previous_gap_frames") or 0) == 1
        and int(row["frame_id"]) == previous_frame_id + 1
    )


def add_feature(
    accumulator: dict[str, object],
    contour_id: int,
    binary_hash: str,
    occupancy_hash: str,
) -> None:
    accumulator["frame_count"] += 1
    accumulator["binary_counts"][(contour_id, binary_hash)] += 1
    accumulator["occupancy_counts"][(contour_id, occupancy_hash)] += 1
    accumulator["shape_only_binary_counts"][binary_hash] += 1


def new_accumulator() -> dict[str, object]:
    return {
        "frame_count": 0,
        "pairs": [],
        "binary_counts": Counter(),
        "occupancy_counts": Counter(),
        "shape_only_binary_counts": Counter(),
        "track_keys": set(),
        "contour_ids": set(),
    }


def metric_summary(accumulator: dict[str, object]) -> dict[str, object]:
    pairs = accumulator["pairs"]
    frame_count = int(accumulator["frame_count"])
    binary_counts = accumulator["binary_counts"]
    occupancy_counts = accumulator["occupancy_counts"]
    exact_count = sum(int(row["binary_exact_match"]) for row in pairs)
    return {
        "track_count": len(accumulator["track_keys"]),
        "contour_count": len(accumulator["contour_ids"]),
        "frame_count": frame_count,
        "pair_count": len(pairs),
        "binary_iou_percent": hierarchy.stats(
            float(row["binary_iou_percent"]) for row in pairs
        ),
        "occupancy_similarity_percent": hierarchy.stats(
            float(row["occupancy_similarity_percent"]) for row in pairs
        ),
        "binary_exact_adjacent_pair_count": exact_count,
        "binary_exact_adjacent_pair_percent": (
            100.0 * exact_count / len(pairs) if pairs else 0.0
        ),
        "identity_scoped_binary_unique_value_count": len(binary_counts),
        "identity_scoped_binary_repeated_frame_count": (
            frame_count - len(binary_counts)
        ),
        "identity_scoped_binary_max_repeat_frequency": (
            max(binary_counts.values()) if binary_counts else 0
        ),
        "shape_only_binary_unique_value_count": len(
            accumulator["shape_only_binary_counts"]
        ),
        "identity_scoped_occupancy_unique_value_count": len(occupancy_counts),
        "identity_scoped_occupancy_repeated_frame_count": (
            frame_count - len(occupancy_counts)
        ),
    }


def row_median(rows: Iterable[dict[str, str]], field: str) -> float | None:
    return hierarchy.percentile(
        (float(row[field]) for row in rows if row.get(field) not in (None, "")),
        50.0,
    )


def source_summary(rows: list[dict[str, str]]) -> dict[str, float | None]:
    return {
        "bbox_width_p50": row_median(rows, "bbox_width"),
        "bbox_height_p50": row_median(rows, "bbox_height"),
        "bbox_area_percent_p50": row_median(rows, "bbox_area_percent"),
        "foreground_area_percent_p50": row_median(
            rows, "foreground_area_percent"
        ),
        "mean_depth_mm_p50": row_median(rows, "mean_depth_mm"),
        "center_radius_percent_p50": row_median(rows, "center_radius_percent"),
    }


def weighted_correlation(
    rows: list[dict[str, object]], x_field: str, y_field: str
) -> float | None:
    usable = [
        row
        for row in rows
        if row.get(x_field) is not None
        and row.get(y_field) is not None
        and int(row["pair_count"]) > 0
    ]
    if len(usable) < 2:
        return None
    weights = np.asarray([float(row["pair_count"]) for row in usable])
    x_values = np.asarray([float(row[x_field]) for row in usable])
    y_values = np.asarray([float(row[y_field]) for row in usable])
    x_centered = x_values - np.average(x_values, weights=weights)
    y_centered = y_values - np.average(y_values, weights=weights)
    denominator = math.sqrt(
        float(np.sum(weights * x_centered * x_centered))
        * float(np.sum(weights * y_centered * y_centered))
    )
    if denominator == 0.0:
        return None
    return float(np.sum(weights * x_centered * y_centered) / denominator)


def flatten_metric_row(
    prefix: dict[str, object], summary: dict[str, object]
) -> dict[str, object]:
    binary = summary["binary_iou_percent"]
    occupancy = summary["occupancy_similarity_percent"]
    return {
        **prefix,
        "track_count": summary["track_count"],
        "contour_count": summary["contour_count"],
        "frame_count": summary["frame_count"],
        "pair_count": summary["pair_count"],
        "binary_iou_min": binary["min"],
        "binary_iou_p05": binary["p05"],
        "binary_iou_p50": binary["p50"],
        "binary_iou_p95": binary["p95"],
        "binary_iou_max": binary["max"],
        "binary_iou_gt99_percent": binary["gt99_percent"],
        "occupancy_similarity_p05": occupancy["p05"],
        "occupancy_similarity_p50": occupancy["p50"],
        "occupancy_similarity_p95": occupancy["p95"],
        "binary_exact_adjacent_pair_count": summary[
            "binary_exact_adjacent_pair_count"
        ],
        "binary_exact_adjacent_pair_percent": summary[
            "binary_exact_adjacent_pair_percent"
        ],
        "identity_scoped_binary_unique_value_count": summary[
            "identity_scoped_binary_unique_value_count"
        ],
        "identity_scoped_binary_repeated_frame_count": summary[
            "identity_scoped_binary_repeated_frame_count"
        ],
        "identity_scoped_binary_max_repeat_frequency": summary[
            "identity_scoped_binary_max_repeat_frequency"
        ],
        "shape_only_binary_unique_value_count": summary[
            "shape_only_binary_unique_value_count"
        ],
        "identity_scoped_occupancy_unique_value_count": summary[
            "identity_scoped_occupancy_unique_value_count"
        ],
        "identity_scoped_occupancy_repeated_frame_count": summary[
            "identity_scoped_occupancy_repeated_frame_count"
        ],
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise ValueError(f"{path}: refusing to write an empty CSV")
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def format_value(value: float | int | None) -> str:
    return "n/a" if value is None else f"{float(value):.4f}"


def write_report(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# 全轮廓全填充分级压缩稳定性比较",
        "",
        f"- 输入段数：{summary['input_run_count']}",
        f"- 不同轮廓 ID：{summary['contour_count']}",
        f"- 段内 track：{summary['track_count']}",
        f"- 轮廓帧记录：{summary['frame_count']}",
        f"- 长 track 门槛：每段帧数的 {summary['long_track_percent']:.1f}%",
        "",
        "只比较同一回放段、同一轮廓 ID 且 previous_gap_frames=1 的连续帧；不做跨回放相邻比较。",
        "",
        "## 持续性分桶",
        "",
        "| 分桶 | 轮廓ID数 | track数 | 帧记录 | 连续帧对(8级) |",
        "|---|---:|---:|---:|---:|",
    ]
    for bucket in BUCKET_ORDER[:-1]:
        item = summary["bucket_overview"][bucket]
        lines.append(
            f"| {BUCKET_LABELS[bucket]} | {item['contour_count']} | "
            f"{item['track_count']} | {item['frame_count']} | {item['pair_count']} |"
        )
    lines.extend(
        [
            "",
            "## 全部轮廓分级结果",
            "",
            "| 边长 | 轮廓ID | 帧记录 | 连续帧对 | IoU p05 / p50 / p95 | >99% | 相邻精确重复 | 身份内不同值 |",
            "|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    all_levels = summary["level_metrics"][BUCKET_ALL]
    for side_text in sorted(all_levels, key=int, reverse=True):
        item = all_levels[side_text]
        binary = item["binary_iou_percent"]
        lines.append(
            f"| {side_text} | {item['contour_count']} | {item['frame_count']} | "
            f"{item['pair_count']} | {format_value(binary['p05'])}% / "
            f"{format_value(binary['p50'])}% / {format_value(binary['p95'])}% | "
            f"{binary['gt99_percent']:.4f}% | "
            f"{item['binary_exact_adjacent_pair_percent']:.4f}% | "
            f"{item['identity_scoped_binary_unique_value_count']} |"
        )
    lines.extend(
        [
            "",
            "## 8级分桶对比",
            "",
            "| 分桶 | 连续帧对 | IoU p05 / p50 / p95 | >99% | 相邻精确重复 | 身份内不同值 |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for bucket in BUCKET_ORDER:
        item = summary["level_metrics"][bucket].get("8")
        if item is None:
            continue
        binary = item["binary_iou_percent"]
        lines.append(
            f"| {BUCKET_LABELS[bucket]} | {item['pair_count']} | "
            f"{format_value(binary['p05'])}% / {format_value(binary['p50'])}% / "
            f"{format_value(binary['p95'])}% | {binary['gt99_percent']:.4f}% | "
            f"{item['binary_exact_adjacent_pair_percent']:.4f}% | "
            f"{item['identity_scoped_binary_unique_value_count']} |"
        )
    lines.extend(
        [
            "",
            "## 相似度与画面属性的加权相关",
            "",
            "| 边长 | 指标 | 面积占比 | 距离 | 距画面中心 |",
            "|---:|---|---:|---:|---:|",
        ]
    )
    for side_text in ("32", "16", "8"):
        relation = summary["relation_metrics"].get(side_text)
        if relation is None:
            continue
        for metric_key, metric_label in (
            ("binary_iou_p50", "逐track IoU p50"),
            ("binary_exact_adjacent_pair_percent", "相邻精确重复率"),
        ):
            values = relation[metric_key]
            lines.append(
                f"| {side_text} | {metric_label} | "
                f"{format_value(values['bbox_area_percent_p50'])} | "
                f"{format_value(values['mean_depth_mm_p50'])} | "
                f"{format_value(values['center_radius_percent_p50'])} |"
            )
    lines.extend(
        [
            "",
            "身份内不同值以 `(contour_id, digest)` 计数；不同轮廓即使压缩位图相同，也不视为同一存在。",
            "短 track 与长期 track 不等权：汇总统计按实际连续帧对数加权，并保留逐 track CSV 供复核。",
            "相关系数按每个 track 的连续帧对数加权，只描述本次静态场景中的共变关系，不证明面积、距离或画面位置是因果。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    if args.min_side < 8 or args.min_side & (args.min_side - 1):
        raise SystemExit("--min-side must be a power of two and at least 8")
    if not 0.0 < args.long_track_percent <= 100.0:
        raise SystemExit("--long-track-percent must be in (0, 100]")

    input_dirs = [path.resolve() for path in args.input_dir]
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    run_rows: dict[str, list[dict[str, str]]] = {}
    run_frame_counts: dict[str, int] = {}
    tracks: dict[tuple[str, int], list[dict[str, str]]] = defaultdict(list)
    for input_dir in input_dirs:
        rows, run_frame_count = read_all_rows(input_dir)
        run_rows[input_dir.name] = rows
        run_frame_counts[input_dir.name] = run_frame_count
        for row in rows:
            tracks[(input_dir.name, int(row["contour_id"]))].append(row)

    contour_ids = sorted({contour_id for _, contour_id in tracks})
    long_runs_by_contour: dict[int, list[str]] = {}
    bucket_by_contour: dict[int, str] = {}
    for contour_id in contour_ids:
        long_runs = []
        for input_dir in input_dirs:
            run_name = input_dir.name
            count = len(tracks.get((run_name, contour_id), []))
            threshold = math.ceil(
                run_frame_counts[run_name] * args.long_track_percent / 100.0
            )
            if count >= threshold:
                long_runs.append(run_name)
        long_runs_by_contour[contour_id] = long_runs
        if len(long_runs) == len(input_dirs):
            bucket_by_contour[contour_id] = BUCKET_SHARED_LONG
        elif long_runs:
            bucket_by_contour[contour_id] = BUCKET_SINGLE_RUN_LONG
        else:
            bucket_by_contour[contour_id] = BUCKET_MEDIUM_SHORT

    base_side_by_contour: dict[int, int] = {}
    for contour_id in contour_ids:
        max_extent = max(
            max(int(row["bbox_width"]), int(row["bbox_height"]))
            for (run_name, track_contour_id), rows in tracks.items()
            if track_contour_id == contour_id
            for row in rows
        )
        base_side_by_contour[contour_id] = hierarchy.next_power_of_two(
            max(max_extent, args.min_side)
        )

    aggregate: dict[tuple[str, int], dict[str, object]] = defaultdict(
        new_accumulator
    )
    contour_aggregate: dict[tuple[int, int], dict[str, object]] = defaultdict(
        new_accumulator
    )
    track_metric_rows: list[dict[str, object]] = []
    contour_metric_rows: list[dict[str, object]] = []
    pair_rows: list[dict[str, object]] = []

    input_by_name = {path.name: path for path in input_dirs}
    for run_name, contour_id in sorted(tracks):
        rows = tracks[(run_name, contour_id)]
        input_dir = input_by_name[run_name]
        bucket = bucket_by_contour[contour_id]
        base_side = base_side_by_contour[contour_id]
        levels = make_levels(base_side, args.min_side)
        track_accumulators = {side: new_accumulator() for side in levels}
        previous_frame_id: int | None = None
        previous_pyramid: dict[int, np.ndarray] | None = None
        previous_hashes: dict[int, str] = {}

        for row in rows:
            frame_id = int(row["frame_id"])
            filled = hierarchy.fill_enclosed_holes(
                hierarchy.decode_content_mask(input_dir, row)
            )
            pyramid = hierarchy.build_pyramid(
                hierarchy.center_in_square(filled, base_side), args.min_side
            )
            consecutive = is_consecutive(row, previous_frame_id)
            for side in levels:
                occupancy = pyramid[side]
                binary_hash, _binary, _binary_bytes = hierarchy.binary_digest(
                    occupancy
                )
                occupancy_hash, _occupancy_bytes = hierarchy.occupancy_digest(
                    occupancy
                )
                for accumulator in (
                    track_accumulators[side],
                    contour_aggregate[(contour_id, side)],
                    aggregate[(bucket, side)],
                    aggregate[(BUCKET_ALL, side)],
                ):
                    add_feature(
                        accumulator, contour_id, binary_hash, occupancy_hash
                    )
                    accumulator["track_keys"].add((run_name, contour_id))
                    accumulator["contour_ids"].add(contour_id)

                if consecutive and previous_pyramid is not None:
                    binary_iou, occupancy_similarity = hierarchy.compare_level(
                        previous_pyramid[side], occupancy
                    )
                    comparison = {
                        "run": run_name,
                        "contour_id": contour_id,
                        "persistence_bucket": bucket,
                        "base_side": base_side,
                        "side": side,
                        "previous_frame_id": previous_frame_id,
                        "frame_id": frame_id,
                        "binary_iou_percent": binary_iou,
                        "occupancy_similarity_percent": occupancy_similarity,
                        "binary_exact_match": int(
                            binary_hash == previous_hashes[side]
                        ),
                    }
                    pair_rows.append(comparison)
                    for accumulator in (
                        track_accumulators[side],
                        contour_aggregate[(contour_id, side)],
                        aggregate[(bucket, side)],
                        aggregate[(BUCKET_ALL, side)],
                    ):
                        accumulator["pairs"].append(comparison)

            previous_frame_id = frame_id
            previous_pyramid = pyramid
            previous_hashes = {
                side: hierarchy.binary_digest(pyramid[side])[0] for side in levels
            }

        for side in levels:
            track_metric_rows.append(
                flatten_metric_row(
                    {
                        "run": run_name,
                        "contour_id": contour_id,
                        "persistence_bucket": bucket,
                        "long_in_run": int(run_name in long_runs_by_contour[contour_id]),
                        "track_frame_percent": (
                            100.0 * len(rows) / run_frame_counts[run_name]
                        ),
                        **source_summary(rows),
                        "base_side": base_side,
                        "side": side,
                    },
                    metric_summary(track_accumulators[side]),
                )
            )

    for contour_id in contour_ids:
        base_side = base_side_by_contour[contour_id]
        contour_rows = [
            row
            for (run_name, track_contour_id), rows in tracks.items()
            if track_contour_id == contour_id
            for row in rows
        ]
        for side in make_levels(base_side, args.min_side):
            contour_metric_rows.append(
                flatten_metric_row(
                    {
                        "contour_id": contour_id,
                        "persistence_bucket": bucket_by_contour[contour_id],
                        "long_run_count": len(long_runs_by_contour[contour_id]),
                        "run_count": sum(
                            (input_dir.name, contour_id) in tracks
                            for input_dir in input_dirs
                        ),
                        **source_summary(contour_rows),
                        "base_side": base_side,
                        "side": side,
                    },
                    metric_summary(contour_aggregate[(contour_id, side)]),
                )
            )

    level_metric_rows: list[dict[str, object]] = []
    level_metrics: dict[str, dict[str, object]] = {
        bucket: {} for bucket in BUCKET_ORDER
    }
    for bucket in BUCKET_ORDER:
        sides = sorted(
            (side for candidate_bucket, side in aggregate if candidate_bucket == bucket),
            reverse=True,
        )
        for side in sides:
            item = metric_summary(aggregate[(bucket, side)])
            level_metrics[bucket][str(side)] = item
            level_metric_rows.append(
                flatten_metric_row(
                    {
                        "persistence_bucket": bucket,
                        "side": side,
                    },
                    item,
                )
            )

    bucket_overview: dict[str, dict[str, int]] = {}
    for bucket in BUCKET_ORDER[:-1]:
        item = level_metrics[bucket].get(str(args.min_side), {})
        bucket_overview[bucket] = {
            "contour_count": sum(
                value == bucket for value in bucket_by_contour.values()
            ),
            "track_count": int(item.get("track_count", 0)),
            "frame_count": int(item.get("frame_count", 0)),
            "pair_count": int(item.get("pair_count", 0)),
        }

    relation_metrics: dict[str, dict[str, dict[str, float | None]]] = {}
    relation_features = [
        "bbox_area_percent_p50",
        "foreground_area_percent_p50",
        "mean_depth_mm_p50",
        "center_radius_percent_p50",
    ]
    relation_targets = [
        "binary_iou_p50",
        "binary_exact_adjacent_pair_percent",
    ]
    for side in sorted({int(row["side"]) for row in track_metric_rows}, reverse=True):
        side_rows = [row for row in track_metric_rows if int(row["side"]) == side]
        relation_metrics[str(side)] = {
            target: {
                feature: weighted_correlation(side_rows, feature, target)
                for feature in relation_features
            }
            for target in relation_targets
        }

    summary: dict[str, object] = {
        "format": "d455_all_hierarchical_filled_contours_v1",
        "definition": {
            "interior": "all enclosed holes filled before normalization",
            "square": "each contour_id uses its own next-power-of-two square",
            "comparison": (
                "same run and contour_id with previous_gap_frames == 1 only"
            ),
            "repeat_identity": "(contour_id, digest), never digest alone",
            "aggregation": "weighted by actual frame and adjacent-pair counts",
        },
        "input_run_count": len(input_dirs),
        "input_runs": [path.name for path in input_dirs],
        "run_frame_counts": run_frame_counts,
        "long_track_percent": args.long_track_percent,
        "min_side": args.min_side,
        "contour_count": len(contour_ids),
        "track_count": len(tracks),
        "frame_count": sum(len(rows) for rows in tracks.values()),
        "pair_metric_row_count": len(pair_rows),
        "bucket_labels": BUCKET_LABELS,
        "bucket_by_contour": {
            str(contour_id): bucket_by_contour[contour_id]
            for contour_id in contour_ids
        },
        "long_runs_by_contour": {
            str(contour_id): long_runs_by_contour[contour_id]
            for contour_id in contour_ids
        },
        "base_side_by_contour": {
            str(contour_id): base_side_by_contour[contour_id]
            for contour_id in contour_ids
        },
        "bucket_overview": bucket_overview,
        "level_metrics": level_metrics,
        "relation_metrics": relation_metrics,
    }

    write_csv(output_dir / "all_contours_per_track.csv", track_metric_rows)
    write_csv(output_dir / "all_contours_per_contour.csv", contour_metric_rows)
    write_csv(output_dir / "all_contours_level_metrics.csv", level_metric_rows)
    write_csv(output_dir / "all_contours_pair_metrics.csv", pair_rows)
    (output_dir / "all_contours_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    write_report(output_dir / "all_contours_report.md", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
