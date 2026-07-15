#!/usr/bin/env python3
"""Test whether compressed filled-contour features distinguish contour IDs."""

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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build packed binary contour features, measure cross-ID collisions, "
            "and run temporal/cross-run ID recognition."
        )
    )
    parser.add_argument("--input-dir", type=Path, action="append", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--side", type=int, action="append", default=None, help="Default: 8,16,32"
    )
    parser.add_argument("--min-track-frames", type=int, default=60)
    parser.add_argument("--train-percent", type=float, default=50.0)
    parser.add_argument("--cross-run-min-percent", type=float, default=90.0)
    parser.add_argument(
        "--reuse-feature-cache",
        action="store_true",
        help="Reuse compressed_feature_index.csv and compressed_binary_features.npz.",
    )
    return parser.parse_args()


def read_rows(input_dir: Path) -> list[dict[str, str]]:
    path = input_dir / "binary_contour_similarity.csv"
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"{path}: no contour rows")
    rows.sort(key=lambda row: (int(row["contour_id"]), int(row["frame_id"])))
    return rows


def percentile(values: Iterable[float], percent: float) -> float | None:
    return hierarchy.percentile(values, percent)


def make_base_sides(
    rows_by_run: dict[str, list[dict[str, str]]], minimum_side: int
) -> dict[int, int]:
    extents: dict[int, int] = defaultdict(int)
    for rows in rows_by_run.values():
        for row in rows:
            contour_id = int(row["contour_id"])
            extents[contour_id] = max(
                extents[contour_id],
                int(row["bbox_width"]),
                int(row["bbox_height"]),
            )
    return {
        contour_id: hierarchy.next_power_of_two(max(extent, minimum_side))
        for contour_id, extent in extents.items()
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise ValueError(f"{path}: refusing to write empty CSV")
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def unpack_features(packed: np.ndarray, side: int) -> np.ndarray:
    return np.unpackbits(packed, axis=1, bitorder="little")[:, : side * side].astype(
        bool
    )


def collision_metrics(
    run_names: np.ndarray,
    contour_ids: np.ndarray,
    packed: np.ndarray,
    side: int,
) -> list[dict[str, object]]:
    results: list[dict[str, object]] = []
    for run_name in sorted(set(run_names.tolist())):
        indices = np.flatnonzero(run_names == run_name)
        ids_by_value: dict[bytes, set[int]] = defaultdict(set)
        count_by_value: Counter[bytes] = Counter()
        for index in indices:
            value = packed[index].tobytes()
            ids_by_value[value].add(int(contour_ids[index]))
            count_by_value[value] += 1
        collision_values = {
            value: ids for value, ids in ids_by_value.items() if len(ids) > 1
        }
        collision_frame_count = sum(
            count_by_value[value] for value in collision_values
        )
        results.append(
            {
                "scope": "within_run",
                "run": run_name,
                "side": side,
                "frame_count": len(indices),
                "contour_count": len(set(contour_ids[indices].tolist())),
                "unique_value_count": len(ids_by_value),
                "cross_id_collision_value_count": len(collision_values),
                "cross_id_collision_frame_count": collision_frame_count,
                "cross_id_collision_frame_percent": (
                    100.0 * collision_frame_count / len(indices) if len(indices) else 0.0
                ),
                "unambiguous_frame_percent": (
                    100.0 * (len(indices) - collision_frame_count) / len(indices)
                    if len(indices)
                    else 0.0
                ),
                "max_contour_ids_per_value": max(
                    (len(ids) for ids in ids_by_value.values()), default=0
                ),
            }
        )
    return results


def build_prototypes(
    features: np.ndarray,
    contour_ids: np.ndarray,
    train_indices: np.ndarray,
    candidate_ids: list[int],
) -> np.ndarray:
    return np.stack(
        [
            features[
                train_indices[contour_ids[train_indices] == contour_id]
            ].mean(axis=0)
            >= 0.5
            for contour_id in candidate_ids
        ]
    )


def evaluate_identification(
    features: np.ndarray,
    contour_ids: np.ndarray,
    train_indices: np.ndarray,
    test_indices: np.ndarray,
    candidate_ids: list[int],
    scope: str,
    train_run: str,
    test_run: str,
    side: int,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    prototypes = build_prototypes(
        features, contour_ids, train_indices, candidate_ids
    )
    id_to_position = {
        contour_id: position for position, contour_id in enumerate(candidate_ids)
    }
    unique_correct = 0
    inclusive_correct = 0
    margins: list[float] = []
    correct_scores: list[float] = []
    confusion: Counter[tuple[int, int]] = Counter()
    batch_size = 256

    for start in range(0, len(test_indices), batch_size):
        batch_indices = test_indices[start : start + batch_size]
        batch = features[batch_indices]
        intersection = np.logical_and(batch[:, None, :], prototypes[None, :, :]).sum(
            axis=2
        )
        union = np.logical_or(batch[:, None, :], prototypes[None, :, :]).sum(axis=2)
        scores = np.divide(
            intersection,
            union,
            out=np.ones_like(intersection, dtype=np.float64),
            where=union != 0,
        )
        predicted_positions = scores.argmax(axis=1)
        for row_index, sample_index in enumerate(batch_indices):
            actual_id = int(contour_ids[sample_index])
            actual_position = id_to_position[actual_id]
            actual_score = float(scores[row_index, actual_position])
            other_scores = np.delete(scores[row_index], actual_position)
            best_other = float(other_scores.max()) if len(other_scores) else -1.0
            margin = actual_score - best_other
            inclusive_correct += int(margin >= -1e-12)
            unique_correct += int(margin > 1e-12)
            margins.append(100.0 * margin)
            correct_scores.append(100.0 * actual_score)
            predicted_id = candidate_ids[int(predicted_positions[row_index])]
            confusion[(actual_id, predicted_id)] += 1

    total = len(test_indices)
    result = {
        "scope": scope,
        "train_run": train_run,
        "test_run": test_run,
        "side": side,
        "candidate_contour_count": len(candidate_ids),
        "train_frame_count": len(train_indices),
        "test_frame_count": total,
        "unique_best_correct_count": unique_correct,
        "unique_best_accuracy_percent": 100.0 * unique_correct / total,
        "correct_including_ties_count": inclusive_correct,
        "correct_including_ties_percent": 100.0 * inclusive_correct / total,
        "correct_score_p50": percentile(correct_scores, 50.0),
        "correct_margin_p05": percentile(margins, 5.0),
        "correct_margin_p50": percentile(margins, 50.0),
    }
    confusion_rows = [
        {
            "scope": scope,
            "train_run": train_run,
            "test_run": test_run,
            "side": side,
            "actual_contour_id": actual_id,
            "predicted_contour_id": predicted_id,
            "frame_count": count,
        }
        for (actual_id, predicted_id), count in sorted(confusion.items())
    ]
    return result, confusion_rows


def temporal_split_indices(
    run_names: np.ndarray,
    contour_ids: np.ndarray,
    frame_ids: np.ndarray,
    run_name: str,
    min_track_frames: int,
    train_percent: float,
) -> tuple[np.ndarray, np.ndarray, list[int]]:
    candidate_ids = []
    train_parts = []
    test_parts = []
    for contour_id in sorted(set(contour_ids[run_names == run_name].tolist())):
        indices = np.flatnonzero(
            (run_names == run_name) & (contour_ids == contour_id)
        )
        indices = indices[np.argsort(frame_ids[indices])]
        if len(indices) < min_track_frames:
            continue
        split = int(math.floor(len(indices) * train_percent / 100.0))
        split = min(max(split, 1), len(indices) - 1)
        candidate_ids.append(int(contour_id))
        train_parts.append(indices[:split])
        test_parts.append(indices[split:])
    if len(candidate_ids) < 2:
        raise ValueError(f"{run_name}: fewer than two eligible contour tracks")
    return np.concatenate(train_parts), np.concatenate(test_parts), candidate_ids


def cross_run_indices(
    run_names: np.ndarray,
    contour_ids: np.ndarray,
    source_run: str,
    target_run: str,
    minimum_source_frames: int,
    minimum_target_frames: int,
) -> tuple[np.ndarray, np.ndarray, list[int]] | None:
    source_counts = Counter(contour_ids[run_names == source_run].tolist())
    target_counts = Counter(contour_ids[run_names == target_run].tolist())
    candidate_ids = sorted(
        contour_id
        for contour_id in set(source_counts) & set(target_counts)
        if source_counts[contour_id] >= minimum_source_frames
        and target_counts[contour_id] >= minimum_target_frames
    )
    if len(candidate_ids) < 2:
        return None
    candidate_array = np.asarray(candidate_ids)
    train_indices = np.flatnonzero(
        (run_names == source_run) & np.isin(contour_ids, candidate_array)
    )
    test_indices = np.flatnonzero(
        (run_names == target_run) & np.isin(contour_ids, candidate_array)
    )
    return train_indices, test_indices, [int(value) for value in candidate_ids]


def format_value(value: float | int | None) -> str:
    return "n/a" if value is None else f"{float(value):.4f}"


def write_report(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# 压缩轮廓区分存在能力测试",
        "",
        f"- 输入段数：{summary['input_run_count']}",
        f"- 轮廓帧记录：{summary['frame_count']}",
        f"- 压缩层级：{', '.join(str(side) for side in summary['sides'])}",
        f"- 身份测试最短 track：{summary['min_track_frames']} 帧",
        "",
        "## 不同ID精确值碰撞",
        "",
        "| 回放 | 边长 | ID数 | 不同值 | 跨ID碰撞值 | 无歧义帧 | 单值最多ID |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary["collision_metrics"]:
        lines.append(
            f"| {row['run']} | {row['side']} | {row['contour_count']} | "
            f"{row['unique_value_count']} | {row['cross_id_collision_value_count']} | "
            f"{row['unambiguous_frame_percent']:.4f}% | "
            f"{row['max_contour_ids_per_value']} |"
        )
    lines.extend(
        [
            "",
            "## 轮廓ID识别",
            "",
            "每个候选 ID 用训练帧压缩位图的逐位多数值建立一个模板；测试帧按二值IoU与所有模板比较。只有正确模板严格高于所有其他模板才计为唯一正确，平分不算可区分。",
            "",
            "| 范围 | 训练→测试 | 边长 | ID数 | 测试帧 | 唯一正确 | 含平分正确 | margin p05 / p50 |",
            "|---|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in summary["identification_metrics"]:
        lines.append(
            f"| {row['scope']} | {row['train_run']} → {row['test_run']} | "
            f"{row['side']} | {row['candidate_contour_count']} | "
            f"{row['test_frame_count']} | {row['unique_best_accuracy_percent']:.4f}% | "
            f"{row['correct_including_ties_percent']:.4f}% | "
            f"{format_value(row['correct_margin_p05'])}% / "
            f"{format_value(row['correct_margin_p50'])}% |"
        )
    lines.extend(
        [
            "",
            "精确值无碰撞只说明该样本中的位图值没有完全相同，不等于相似轮廓可可靠分类；身份识别结果才是本轮主要区分指标。",
            "轮廓 ID 是观察身份，不是已经裁决的现实存在身份；接替 ID 之间的混淆可能表示 tracker 换号，而不是现实存在无法区分。",
            "temporal_within_run_long 和跨回放测试只纳入覆盖至少 cross_run_min_percent 的长期 ID，用于减少接替标签污染。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    sides = sorted(set(args.side or [8, 16, 32]))
    if any(side < 8 or side & (side - 1) for side in sides):
        raise SystemExit("every --side must be a power of two and at least 8")
    if args.min_track_frames < 2:
        raise SystemExit("--min-track-frames must be at least 2")
    if not 0.0 < args.train_percent < 100.0:
        raise SystemExit("--train-percent must be in (0, 100)")
    if not 0.0 < args.cross_run_min_percent <= 100.0:
        raise SystemExit("--cross-run-min-percent must be in (0, 100]")

    input_dirs = [path.resolve() for path in args.input_dir]
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    rows_by_run = {path.name: read_rows(path) for path in input_dirs}
    run_frame_counts = {
        run_name: len({int(row["frame_id"]) for row in rows})
        for run_name, rows in rows_by_run.items()
    }
    input_by_name = {path.name: path for path in input_dirs}
    cache_path = output_dir / "compressed_binary_features.npz"
    index_path = output_dir / "compressed_feature_index.csv"
    if args.reuse_feature_cache:
        if not cache_path.exists() or not index_path.exists():
            raise SystemExit("--reuse-feature-cache requires existing cache files")
        with index_path.open("r", encoding="utf-8", newline="") as stream:
            index_rows = list(csv.DictReader(stream))
        with np.load(cache_path) as cache:
            run_names = cache["run_names"].astype(str)
            frame_ids = cache["frame_ids"].astype(np.int64)
            contour_ids = cache["contour_ids"].astype(np.int64)
            packed_arrays = {
                side: cache[f"side_{side}"].copy() for side in sides
            }
        if len(index_rows) != len(frame_ids):
            raise ValueError("feature index and cache row counts differ")
    else:
        base_sides = make_base_sides(rows_by_run, max(sides))
        index_rows = []
        packed_by_side: dict[int, list[np.ndarray]] = {
            side: [] for side in sides
        }
        for run_name in sorted(rows_by_run):
            input_dir = input_by_name[run_name]
            for row in rows_by_run[run_name]:
                contour_id = int(row["contour_id"])
                base_side = base_sides[contour_id]
                filled = hierarchy.fill_enclosed_holes(
                    hierarchy.decode_content_mask(input_dir, row)
                )
                pyramid = hierarchy.build_pyramid(
                    hierarchy.center_in_square(filled, base_side), min(sides)
                )
                index_rows.append(
                    {
                        "feature_index": len(index_rows),
                        "run": run_name,
                        "frame_id": int(row["frame_id"]),
                        "contour_id": contour_id,
                        "base_side": base_side,
                        "bbox_area_percent": row["bbox_area_percent"],
                        "mean_depth_mm": row["mean_depth_mm"],
                        "center_radius_percent": row["center_radius_percent"],
                    }
                )
                for side in sides:
                    binary = pyramid[side] >= 0.5
                    packed_by_side[side].append(
                        np.packbits(binary.reshape(-1), bitorder="little")
                    )

        run_names = np.asarray([str(row["run"]) for row in index_rows])
        frame_ids = np.asarray([int(row["frame_id"]) for row in index_rows])
        contour_ids = np.asarray([int(row["contour_id"]) for row in index_rows])
        packed_arrays = {
            side: np.stack(values).astype(np.uint8)
            for side, values in packed_by_side.items()
        }
        np.savez_compressed(
            cache_path,
            run_names=run_names,
            frame_ids=frame_ids,
            contour_ids=contour_ids,
            base_sides=np.asarray(
                [int(row["base_side"]) for row in index_rows]
            ),
            **{f"side_{side}": packed_arrays[side] for side in sides},
        )
        write_csv(index_path, index_rows)

    collisions: list[dict[str, object]] = []
    identification: list[dict[str, object]] = []
    confusion_rows: list[dict[str, object]] = []
    run_list = sorted(rows_by_run)
    for side in sides:
        packed = packed_arrays[side]
        features = unpack_features(packed, side)
        collisions.extend(collision_metrics(run_names, contour_ids, packed, side))
        for run_name in run_list:
            train_indices, test_indices, candidate_ids = temporal_split_indices(
                run_names,
                contour_ids,
                frame_ids,
                run_name,
                args.min_track_frames,
                args.train_percent,
            )
            result, confusion = evaluate_identification(
                features,
                contour_ids,
                train_indices,
                test_indices,
                candidate_ids,
                "temporal_within_run",
                run_name,
                run_name,
                side,
            )
            identification.append(result)
            confusion_rows.extend(confusion)
            long_minimum_frames = max(
                args.min_track_frames,
                math.ceil(
                    run_frame_counts[run_name]
                    * args.cross_run_min_percent
                    / 100.0
                ),
            )
            long_train, long_test, long_candidate_ids = temporal_split_indices(
                run_names,
                contour_ids,
                frame_ids,
                run_name,
                long_minimum_frames,
                args.train_percent,
            )
            result, confusion = evaluate_identification(
                features,
                contour_ids,
                long_train,
                long_test,
                long_candidate_ids,
                "temporal_within_run_long",
                run_name,
                run_name,
                side,
            )
            identification.append(result)
            confusion_rows.extend(confusion)
        for source_run in run_list:
            for target_run in run_list:
                if source_run == target_run:
                    continue
                split = cross_run_indices(
                    run_names,
                    contour_ids,
                    source_run,
                    target_run,
                    max(
                        args.min_track_frames,
                        math.ceil(
                            run_frame_counts[source_run]
                            * args.cross_run_min_percent
                            / 100.0
                        ),
                    ),
                    max(
                        args.min_track_frames,
                        math.ceil(
                            run_frame_counts[target_run]
                            * args.cross_run_min_percent
                            / 100.0
                        ),
                    ),
                )
                if split is None:
                    continue
                train_indices, test_indices, candidate_ids = split
                result, confusion = evaluate_identification(
                    features,
                    contour_ids,
                    train_indices,
                    test_indices,
                    candidate_ids,
                    "cross_run_shared_id",
                    source_run,
                    target_run,
                    side,
                )
                identification.append(result)
                confusion_rows.extend(confusion)

    summary: dict[str, object] = {
        "format": "d455_compressed_contour_discrimination_v1",
        "definition": {
            "feature": "filled contour, centered per-ID base square, binary occupancy >= 0.5",
            "collision": "same packed value used by more than one contour_id within a run",
            "prototype": "per-cell majority of training binary features",
            "similarity": "binary IoU",
            "unique_correct": "correct prototype score strictly exceeds every other prototype",
        },
        "input_run_count": len(input_dirs),
        "input_runs": run_list,
        "frame_count": len(index_rows),
        "sides": sides,
        "min_track_frames": args.min_track_frames,
        "train_percent": args.train_percent,
        "cross_run_min_percent": args.cross_run_min_percent,
        "collision_metrics": collisions,
        "identification_metrics": identification,
    }
    write_csv(output_dir / "compressed_collision_metrics.csv", collisions)
    write_csv(output_dir / "compressed_identification_metrics.csv", identification)
    write_csv(output_dir / "compressed_confusion.csv", confusion_rows)
    (output_dir / "compressed_discrimination_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    write_report(output_dir / "compressed_discrimination_report.md", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
