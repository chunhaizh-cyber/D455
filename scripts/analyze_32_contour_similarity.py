#!/usr/bin/env python3
"""Measure 32x32 contour IoU separation for same and different track IDs."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare same-ID and different-ID compressed contours and search "
            "descriptive IoU thresholds."
        )
    )
    parser.add_argument("--feature-cache", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--side", type=int, default=32)
    parser.add_argument("--long-track-percent", type=float, default=90.0)
    parser.add_argument("--threshold-step", type=float, default=0.1)
    return parser.parse_args()


def percentile(values: np.ndarray, percent: float) -> float | None:
    if not len(values):
        return None
    return float(np.percentile(values, percent))


def distribution(values: np.ndarray) -> dict[str, float | int | None]:
    return {
        "count": int(len(values)),
        "min": float(values.min()) if len(values) else None,
        "p01": percentile(values, 1.0),
        "p05": percentile(values, 5.0),
        "p50": percentile(values, 50.0),
        "p95": percentile(values, 95.0),
        "p99": percentile(values, 99.0),
        "max": float(values.max()) if len(values) else None,
    }


def unpack_features(packed: np.ndarray, side: int) -> np.ndarray:
    return np.unpackbits(packed, axis=1, bitorder="little")[
        :, : side * side
    ].astype(bool)


def pair_iou(
    features: np.ndarray,
    left_indices: np.ndarray,
    right_indices: np.ndarray,
) -> np.ndarray:
    scores = np.empty(len(left_indices), dtype=np.float64)
    batch_size = 4096
    for start in range(0, len(left_indices), batch_size):
        stop = min(start + batch_size, len(left_indices))
        left = features[left_indices[start:stop]]
        right = features[right_indices[start:stop]]
        intersection = np.logical_and(left, right).sum(axis=1)
        union = np.logical_or(left, right).sum(axis=1)
        scores[start:stop] = np.divide(
            intersection,
            union,
            out=np.ones_like(intersection, dtype=np.float64),
            where=union != 0,
        )
    return scores * 100.0


def within_run_pairs(
    run_names: np.ndarray,
    frame_ids: np.ndarray,
    contour_ids: np.ndarray,
    run_name: str,
    allowed_ids: set[int] | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    mask = run_names == run_name
    if allowed_ids is not None:
        mask &= np.isin(contour_ids, np.asarray(sorted(allowed_ids)))
    indices = np.flatnonzero(mask)

    positive_left: list[int] = []
    positive_right: list[int] = []
    by_id: dict[int, list[int]] = defaultdict(list)
    for index in indices:
        by_id[int(contour_ids[index])].append(int(index))
    for track_indices in by_id.values():
        ordered = sorted(track_indices, key=lambda index: int(frame_ids[index]))
        for left, right in zip(ordered, ordered[1:]):
            if int(frame_ids[right]) == int(frame_ids[left]) + 1:
                positive_left.append(left)
                positive_right.append(right)

    negative_left: list[int] = []
    negative_right: list[int] = []
    by_frame: dict[int, list[int]] = defaultdict(list)
    for index in indices:
        by_frame[int(frame_ids[index])].append(int(index))
    for frame_indices in by_frame.values():
        for left, right in itertools.combinations(frame_indices, 2):
            if contour_ids[left] != contour_ids[right]:
                negative_left.append(left)
                negative_right.append(right)

    return (
        np.asarray(positive_left, dtype=np.int64),
        np.asarray(positive_right, dtype=np.int64),
        np.asarray(negative_left, dtype=np.int64),
        np.asarray(negative_right, dtype=np.int64),
    )


def cross_run_pairs(
    run_names: np.ndarray,
    frame_ids: np.ndarray,
    contour_ids: np.ndarray,
    source_run: str,
    target_run: str,
    shared_ids: set[int],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    source_lookup = {
        (int(frame_ids[index]), int(contour_ids[index])): int(index)
        for index in np.flatnonzero(
            (run_names == source_run)
            & np.isin(contour_ids, np.asarray(sorted(shared_ids)))
        )
    }
    target_lookup = {
        (int(frame_ids[index]), int(contour_ids[index])): int(index)
        for index in np.flatnonzero(
            (run_names == target_run)
            & np.isin(contour_ids, np.asarray(sorted(shared_ids)))
        )
    }
    common_frames = sorted(
        {frame for frame, _ in source_lookup} & {frame for frame, _ in target_lookup}
    )
    positive_left: list[int] = []
    positive_right: list[int] = []
    negative_left: list[int] = []
    negative_right: list[int] = []
    for frame_id in common_frames:
        source_ids = [
            contour_id
            for contour_id in sorted(shared_ids)
            if (frame_id, contour_id) in source_lookup
        ]
        target_ids = [
            contour_id
            for contour_id in sorted(shared_ids)
            if (frame_id, contour_id) in target_lookup
        ]
        for contour_id in set(source_ids) & set(target_ids):
            positive_left.append(source_lookup[(frame_id, contour_id)])
            positive_right.append(target_lookup[(frame_id, contour_id)])
        for source_id in source_ids:
            for target_id in target_ids:
                if source_id == target_id:
                    continue
                negative_left.append(source_lookup[(frame_id, source_id)])
                negative_right.append(target_lookup[(frame_id, target_id)])
    return (
        np.asarray(positive_left, dtype=np.int64),
        np.asarray(positive_right, dtype=np.int64),
        np.asarray(negative_left, dtype=np.int64),
        np.asarray(negative_right, dtype=np.int64),
    )


def auc_score(positive: np.ndarray, negative: np.ndarray) -> float:
    scores = np.concatenate([positive, negative])
    labels = np.concatenate(
        [np.ones(len(positive), dtype=bool), np.zeros(len(negative), dtype=bool)]
    )
    order = np.argsort(scores, kind="mergesort")
    ordered_scores = scores[order]
    ranks = np.empty(len(scores), dtype=np.float64)
    start = 0
    while start < len(scores):
        stop = start + 1
        while stop < len(scores) and ordered_scores[stop] == ordered_scores[start]:
            stop += 1
        average_rank = (start + 1 + stop) / 2.0
        ranks[order[start:stop]] = average_rank
        start = stop
    positive_rank_sum = float(ranks[labels].sum())
    numerator = positive_rank_sum - len(positive) * (len(positive) + 1) / 2.0
    return numerator / (len(positive) * len(negative))


def threshold_metrics(
    positive: np.ndarray, negative: np.ndarray, threshold: float
) -> dict[str, float | int]:
    true_positive = int((positive >= threshold).sum())
    false_negative = len(positive) - true_positive
    false_positive = int((negative >= threshold).sum())
    true_negative = len(negative) - false_positive
    true_positive_percent = 100.0 * true_positive / len(positive)
    false_positive_percent = 100.0 * false_positive / len(negative)
    true_negative_percent = 100.0 - false_positive_percent
    balanced_accuracy = (true_positive_percent + true_negative_percent) / 2.0
    precision = (
        100.0 * true_positive / (true_positive + false_positive)
        if true_positive + false_positive
        else 0.0
    )
    return {
        "threshold_percent": threshold,
        "true_positive_count": true_positive,
        "false_negative_count": false_negative,
        "false_positive_count": false_positive,
        "true_negative_count": true_negative,
        "true_positive_percent": true_positive_percent,
        "false_negative_percent": 100.0 - true_positive_percent,
        "false_positive_percent": false_positive_percent,
        "true_negative_percent": true_negative_percent,
        "balanced_accuracy_percent": balanced_accuracy,
        "precision_percent": precision,
    }


def choose_best(rows: list[dict[str, float | int]]) -> dict[str, float | int]:
    return max(
        rows,
        key=lambda row: (
            float(row["balanced_accuracy_percent"]),
            -float(row["false_positive_percent"]),
            float(row["threshold_percent"]),
        ),
    )


def choose_fpr_limit(
    rows: list[dict[str, float | int]], limit_percent: float
) -> dict[str, float | int] | None:
    eligible = [
        row for row in rows if float(row["false_positive_percent"]) <= limit_percent
    ]
    if not eligible:
        return None
    return max(
        eligible,
        key=lambda row: (
            float(row["true_positive_percent"]),
            -float(row["false_positive_percent"]),
            float(row["threshold_percent"]),
        ),
    )


def make_threshold_rows(
    scope: str,
    positive: np.ndarray,
    negative: np.ndarray,
    step: float,
) -> list[dict[str, float | int | str]]:
    count = int(round(100.0 / step))
    thresholds = [round(index * step, 10) for index in range(count + 1)]
    return [
        {"scope": scope, **threshold_metrics(positive, negative, threshold)}
        for threshold in thresholds
    ]


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise ValueError(f"{path}: refusing to write empty CSV")
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def pair_rows(
    scope: str,
    label: str,
    left: np.ndarray,
    right: np.ndarray,
    scores: np.ndarray,
    run_names: np.ndarray,
    frame_ids: np.ndarray,
    contour_ids: np.ndarray,
) -> list[dict[str, object]]:
    return [
        {
            "scope": scope,
            "label": label,
            "run_a": str(run_names[left_index]),
            "frame_id_a": int(frame_ids[left_index]),
            "contour_id_a": int(contour_ids[left_index]),
            "run_b": str(run_names[right_index]),
            "frame_id_b": int(frame_ids[right_index]),
            "contour_id_b": int(contour_ids[right_index]),
            "iou_percent": float(score),
        }
        for left_index, right_index, score in zip(left, right, scores)
    ]


def group_pair_metrics(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    groups: dict[tuple[object, ...], list[float]] = defaultdict(list)
    metadata: dict[tuple[object, ...], dict[str, object]] = {}
    for row in rows:
        scope = str(row["scope"])
        label = str(row["label"])
        contour_id_a = int(row["contour_id_a"])
        contour_id_b = int(row["contour_id_b"])
        if label == "same_id":
            key = (
                scope,
                label,
                str(row["run_a"]),
                str(row["run_b"]),
                contour_id_a,
                contour_id_b,
            )
        elif row["run_a"] == row["run_b"]:
            key = (
                scope,
                label,
                str(row["run_a"]),
                str(row["run_b"]),
                min(contour_id_a, contour_id_b),
                max(contour_id_a, contour_id_b),
            )
        else:
            key = (
                scope,
                label,
                str(row["run_a"]),
                str(row["run_b"]),
                contour_id_a,
                contour_id_b,
            )
        groups[key].append(float(row["iou_percent"]))
        metadata[key] = {
            "scope": key[0],
            "label": key[1],
            "run_a": key[2],
            "run_b": key[3],
            "contour_id_a": key[4],
            "contour_id_b": key[5],
        }
    result = []
    for key, values in groups.items():
        item = distribution(np.asarray(values, dtype=np.float64))
        result.append({**metadata[key], **item})
    return sorted(
        result,
        key=lambda row: (
            str(row["scope"]),
            str(row["label"]),
            str(row["run_a"]),
            int(row["contour_id_a"]),
            int(row["contour_id_b"]),
        ),
    )


def format_value(value: float | int | None) -> str:
    return "n/a" if value is None else f"{float(value):.4f}"


def write_report(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# 32x32轮廓直接相似度与阈值测试",
        "",
        f"- 特征记录：{summary['frame_count']}",
        f"- 共同长期ID：{', '.join(str(value) for value in summary['shared_long_ids'])}",
        "- 正样本：同ID连续帧或两段中同长期ID",
        "- 负样本：同帧不同ID",
        "",
        "| 范围 | 正样本 | 负样本 | 正样本IoU p05 / p50 / p95 | 负样本IoU p05 / p50 / p95 | AUC |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for item in summary["scope_metrics"]:
        positive = item["positive_distribution"]
        negative = item["negative_distribution"]
        lines.append(
            f"| {item['scope']} | {positive['count']} | {negative['count']} | "
            f"{format_value(positive['p05'])}% / {format_value(positive['p50'])}% / "
            f"{format_value(positive['p95'])}% | {format_value(negative['p05'])}% / "
            f"{format_value(negative['p50'])}% / {format_value(negative['p95'])}% | "
            f"{item['auc']:.6f} |"
        )
    lines.extend(
        [
            "",
            "## 自动阈值",
            "",
            "| 范围 | 最佳平衡阈值 | TPR | FPR | 平衡准确率 | FPR<=1%阈值 | 对应TPR |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for item in summary["scope_metrics"]:
        best = item["best_balanced_threshold"]
        fpr = item["fpr_le_1_threshold"]
        lines.append(
            f"| {item['scope']} | {best['threshold_percent']:.1f}% | "
            f"{best['true_positive_percent']:.4f}% | "
            f"{best['false_positive_percent']:.4f}% | "
            f"{best['balanced_accuracy_percent']:.4f}% | "
            f"{format_value(fpr['threshold_percent'] if fpr else None)}% | "
            f"{format_value(fpr['true_positive_percent'] if fpr else None)}% |"
        )
    transfer = summary["threshold_transfer"]
    lines.extend(
        [
            "",
            "## 阈值迁移",
            "",
            f"阈值只在 `{transfer['selection_scope']}` 上选择，再固定应用到其他范围。",
            "",
            "| 范围 | 固定阈值 | TPR | FPR | 平衡准确率 |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for row in transfer["best_balanced_transfer"]:
        lines.append(
            f"| {row['scope']} | {row['threshold_percent']:.1f}% | "
            f"{row['true_positive_percent']:.4f}% | "
            f"{row['false_positive_percent']:.4f}% | "
            f"{row['balanced_accuracy_percent']:.4f}% |"
        )
    lines.extend(
        [
            "",
            "## 主要重叠来源",
            "",
            "| 类型 | 回放 | ID | p05 | p50 | p95 |",
            "|---|---|---|---:|---:|---:|",
        ]
    )
    for row in summary["risk_groups"]["lowest_same_id_p05"][:5]:
        lines.append(
            f"| 同ID低相似 | {row['run_a']} | {row['contour_id_a']} | "
            f"{format_value(row['p05'])}% | {format_value(row['p50'])}% | "
            f"{format_value(row['p95'])}% |"
        )
    for row in summary["risk_groups"]["highest_different_id_p95"][:5]:
        lines.append(
            f"| 不同ID高相似 | {row['run_a']} | "
            f"{row['contour_id_a']}/{row['contour_id_b']} | "
            f"{format_value(row['p05'])}% | {format_value(row['p50'])}% | "
            f"{format_value(row['p95'])}% |"
        )
    lines.extend(
        [
            "",
            "不同ID是观察标签，不是人工现实存在真值；接替ID可能属于同一现实对象。",
            "自动阈值来自当前静态闭集数据，只是描述性候选，不能直接写入生产配置。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    if args.side < 8 or args.side & (args.side - 1):
        raise SystemExit("--side must be a power of two and at least 8")
    if not 0.0 < args.long_track_percent <= 100.0:
        raise SystemExit("--long-track-percent must be in (0, 100]")
    if not 0.0 < args.threshold_step <= 10.0:
        raise SystemExit("--threshold-step must be in (0, 10]")

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    with np.load(args.feature_cache.resolve()) as cache:
        run_names = cache["run_names"].astype(str)
        frame_ids = cache["frame_ids"].astype(np.int64)
        contour_ids = cache["contour_ids"].astype(np.int64)
        packed = cache[f"side_{args.side}"].copy()
    features = unpack_features(packed, args.side)
    runs = sorted(set(run_names.tolist()))
    if len(runs) != 2:
        raise ValueError("cross-run comparison currently requires exactly two runs")

    long_ids_by_run: dict[str, set[int]] = {}
    for run_name in runs:
        run_mask = run_names == run_name
        run_frame_count = len(set(frame_ids[run_mask].tolist()))
        threshold = math.ceil(run_frame_count * args.long_track_percent / 100.0)
        counts = Counter(contour_ids[run_mask].tolist())
        long_ids_by_run[run_name] = {
            int(contour_id) for contour_id, count in counts.items() if count >= threshold
        }
    shared_long_ids = set.intersection(*long_ids_by_run.values())

    datasets: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    atomic_pairs: list[dict[str, object]] = []
    all_positive_parts = []
    all_negative_parts = []
    long_positive_parts = []
    long_negative_parts = []
    first_long_scope = ""

    for run_name in runs:
        for bucket, allowed_ids in (
            ("all", None),
            ("long", long_ids_by_run[run_name]),
        ):
            scope = f"within_{bucket}:{run_name}"
            left_p, right_p, left_n, right_n = within_run_pairs(
                run_names, frame_ids, contour_ids, run_name, allowed_ids
            )
            positive = pair_iou(features, left_p, right_p)
            negative = pair_iou(features, left_n, right_n)
            datasets[scope] = (positive, negative)
            atomic_pairs.extend(
                pair_rows(
                    scope,
                    "same_id",
                    left_p,
                    right_p,
                    positive,
                    run_names,
                    frame_ids,
                    contour_ids,
                )
            )
            atomic_pairs.extend(
                pair_rows(
                    scope,
                    "different_id",
                    left_n,
                    right_n,
                    negative,
                    run_names,
                    frame_ids,
                    contour_ids,
                )
            )
            if bucket == "all":
                all_positive_parts.append(positive)
                all_negative_parts.append(negative)
            else:
                long_positive_parts.append(positive)
                long_negative_parts.append(negative)
                if not first_long_scope:
                    first_long_scope = scope

    datasets["within_all:combined"] = (
        np.concatenate(all_positive_parts),
        np.concatenate(all_negative_parts),
    )
    datasets["within_long:combined"] = (
        np.concatenate(long_positive_parts),
        np.concatenate(long_negative_parts),
    )
    left_p, right_p, left_n, right_n = cross_run_pairs(
        run_names,
        frame_ids,
        contour_ids,
        runs[0],
        runs[1],
        shared_long_ids,
    )
    cross_positive = pair_iou(features, left_p, right_p)
    cross_negative = pair_iou(features, left_n, right_n)
    cross_scope = "cross_shared_long:aligned_frame"
    datasets[cross_scope] = (cross_positive, cross_negative)
    atomic_pairs.extend(
        pair_rows(
            cross_scope,
            "same_id",
            left_p,
            right_p,
            cross_positive,
            run_names,
            frame_ids,
            contour_ids,
        )
    )
    atomic_pairs.extend(
        pair_rows(
            cross_scope,
            "different_id",
            left_n,
            right_n,
            cross_negative,
            run_names,
            frame_ids,
            contour_ids,
        )
    )

    scope_metrics: list[dict[str, object]] = []
    threshold_rows: list[dict[str, object]] = []
    thresholds_by_scope: dict[str, list[dict[str, float | int | str]]] = {}
    for scope, (positive, negative) in datasets.items():
        rows = make_threshold_rows(scope, positive, negative, args.threshold_step)
        thresholds_by_scope[scope] = rows
        threshold_rows.extend(rows)
        positive_distribution = distribution(positive)
        negative_distribution = distribution(negative)
        scope_metrics.append(
            {
                "scope": scope,
                "positive_distribution": positive_distribution,
                "negative_distribution": negative_distribution,
                "p05_minus_negative_p95": (
                    float(positive_distribution["p05"])
                    - float(negative_distribution["p95"])
                ),
                "auc": auc_score(positive, negative),
                "best_balanced_threshold": choose_best(rows),
                "fpr_le_1_threshold": choose_fpr_limit(rows, 1.0),
                "fixed_thresholds": {
                    str(value): threshold_metrics(positive, negative, value)
                    for value in (70.0, 80.0, 90.0)
                },
            }
        )

    selected_best = choose_best(thresholds_by_scope[first_long_scope])
    selected_fpr = choose_fpr_limit(thresholds_by_scope[first_long_scope], 1.0)
    best_transfer = [
        {
            "scope": scope,
            **threshold_metrics(
                positive, negative, float(selected_best["threshold_percent"])
            ),
        }
        for scope, (positive, negative) in datasets.items()
    ]
    fpr_transfer = (
        [
            {
                "scope": scope,
                **threshold_metrics(
                    positive, negative, float(selected_fpr["threshold_percent"])
                ),
            }
            for scope, (positive, negative) in datasets.items()
        ]
        if selected_fpr is not None
        else []
    )
    grouped_metrics = group_pair_metrics(atomic_pairs)
    long_grouped = [
        row for row in grouped_metrics if str(row["scope"]).startswith("within_long:")
    ]
    lowest_same_id_p05 = sorted(
        (row for row in long_grouped if row["label"] == "same_id"),
        key=lambda row: float(row["p05"]),
    )
    highest_different_id_p95 = sorted(
        (row for row in long_grouped if row["label"] == "different_id"),
        key=lambda row: float(row["p95"]),
        reverse=True,
    )

    summary: dict[str, object] = {
        "format": "d455_32_contour_similarity_v1",
        "definition": {
            "feature": f"{args.side}x{args.side} centered filled binary contour",
            "similarity": "foreground IoU percent",
            "positive_within_run": "same contour_id with consecutive frame ids",
            "negative_within_run": "different contour_id in the same frame",
            "positive_cross_run": "same shared-long contour_id at aligned frame id",
            "negative_cross_run": "different shared-long contour_id at aligned frame id",
            "decision": "same candidate when IoU >= threshold",
        },
        "feature_cache": str(args.feature_cache.resolve()),
        "frame_count": len(frame_ids),
        "side": args.side,
        "runs": runs,
        "long_track_percent": args.long_track_percent,
        "long_ids_by_run": {
            run_name: sorted(values) for run_name, values in long_ids_by_run.items()
        },
        "shared_long_ids": sorted(shared_long_ids),
        "scope_metrics": scope_metrics,
        "risk_groups": {
            "lowest_same_id_p05": lowest_same_id_p05[:10],
            "highest_different_id_p95": highest_different_id_p95[:10],
        },
        "threshold_transfer": {
            "selection_scope": first_long_scope,
            "selected_best_balanced_threshold": selected_best,
            "selected_fpr_le_1_threshold": selected_fpr,
            "best_balanced_transfer": best_transfer,
            "fpr_le_1_transfer": fpr_transfer,
        },
    }

    write_csv(output_dir / "similarity_pairs.csv", atomic_pairs)
    write_csv(output_dir / "pair_group_metrics.csv", grouped_metrics)
    write_csv(output_dir / "threshold_sweep.csv", threshold_rows)
    write_csv(output_dir / "threshold_transfer.csv", best_transfer)
    (output_dir / "similarity_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    write_report(output_dir / "similarity_report.md", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
