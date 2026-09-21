"""Evaluate deterministic static stability of completed PCS cluster stream runs.

This evaluator is deliberately limited to recorded stream evidence.  It does
not infer physical stillness, object identity, absolute depth accuracy, or
dynamic tracking quality from matching packets.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from cluster_protocol import unpack_ring, validate_cluster_packet
from cluster_tracker import reconstruct


NONDETERMINISTIC_HEADER_FIELDS = {"包标识", "会话标识", "发布Unix毫秒"}


def normalized_packet(packet: dict) -> bytes:
    value = copy.deepcopy({key: item for key, item in packet.items() if key not in NONDETERMINISTIC_HEADER_FIELDS})
    # This checksum names a published PCS.Observation/1 manifest whose session
    # and publication timestamp legitimately vary across otherwise identical runs.
    # It remains in the packet for traceability, but cannot participate in an
    # algorithm determinism comparison.
    value.get("指标", {}).pop("转换来源清单SHA256", None)
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), sort_keys=True).encode("utf-8")


def load_run(root: Path) -> tuple[list[dict], list[Path]]:
    packet_root = root / "cluster_packets"
    paths = sorted(packet_root.glob("packet_*/packet.json"))
    if not paths:
        raise ValueError(f"No cluster packets: {root}")
    packets = []
    for path in paths:
        validate_cluster_packet(path)
        packets.append(json.loads(path.read_text(encoding="utf-8")))
    return packets, paths


def percentile(values: list[float], percent: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), percent))


def contour_material(packet: dict, packet_path: Path) -> bytes:
    descriptor = packet["材料"]["精确轮廓链"]
    return (packet_path.parent / descriptor["文件"]).read_bytes()


def normalized_filled_mask(entry: dict, contours: bytes, side: int = 32) -> np.ndarray:
    """Rasterize protocol rings into a centered diagnostic silhouette."""
    x, y, width, height = entry["范围XYWH"]
    mask = np.zeros((height, width), dtype=np.uint8)
    rings = entry["轮廓"]
    depths: list[int] = []
    for index, ring in enumerate(rings):
        depth = 0
        parent = ring["父环索引"]
        seen = {index}
        while parent != -1:
            if parent in seen:
                raise ValueError("Contour hierarchy cycle during rasterization")
            seen.add(parent)
            depth += 1
            parent = rings[parent]["父环索引"]
        depths.append(depth)
    for index in sorted(range(len(rings)), key=lambda value: (depths[value], value)):
        ring = rings[index]
        offset = ring["字节偏移"]
        length = (ring["有效位数"] + 7) // 8
        points = unpack_ring(ring["起点XY"], ring["点数"], contours[offset:offset + length], ring["有效位数"])
        local = np.asarray([(px - x, py - y) for px, py in points], dtype=np.int32)
        cv2.fillPoly(mask, [local], 0 if ring["内环"] else 1)
    square_side = max(height, width)
    square = np.zeros((square_side, square_side), dtype=np.uint8)
    top = (square_side - height) // 2
    left = (square_side - width) // 2
    square[top:top + height, left:left + width] = mask
    reduced = np.zeros((side, side), dtype=np.uint8)
    for row in range(side):
        y0, y1 = row * square_side // side, (row + 1) * square_side // side
        for column in range(side):
            x0, x1 = column * square_side // side, (column + 1) * square_side // side
            reduced[row, column] = 1 if np.any(square[y0:y1, x0:x1]) else 0
    return reduced


def candidate_layer(entry: dict, image_width: int, image_height: int) -> str:
    """Classify packet geometry without inferring background or object identity."""
    x, y, width, height = entry["范围XYWH"]
    bbox_coverage = width * height / (image_width * image_height)
    if entry["触及视野边界"] and bbox_coverage >= 0.90:
        return "全画面账本区域"
    if entry["触及视野边界"]:
        return "触边开放候选"
    return "非触边闭合候选"


def geometry_sample(entry: dict, contours: bytes, sequence: int,
                    image_width: int, image_height: int) -> dict:
    mask = normalized_filled_mask(entry, contours)
    packed = np.packbits(mask.reshape(-1), bitorder="little").tobytes()
    _, _, width, height = entry["范围XYWH"]
    current_depth = int(entry["深度证据"]["当前实测像素数"])
    return {
        "sequence": sequence,
        "track_id": entry["相机跟踪候选编号"],
        "frame_cluster_id": entry["帧内簇编号"],
        "center_x": float(entry["图像中心XY"][0]),
        "center_y": float(entry["图像中心XY"][1]),
        "pixels": int(entry["像素数"]),
        "state": entry["跟踪状态"],
        "candidate_layer": candidate_layer(entry, image_width, image_height),
        "touches_border": bool(entry["触及视野边界"]),
        "bbox_coverage_percent": 100.0 * width * height / (image_width * image_height),
        "current_depth_support_percent": 100.0 * current_depth / entry["像素数"],
        "mask": mask,
        "mask_sha256": hashlib.sha256(packed).hexdigest(),
    }


def compare_geometry(previous: dict, current: dict, carried: bool) -> dict:
    left, right = previous["mask"].astype(bool), current["mask"].astype(bool)
    intersection = int(np.count_nonzero(left & right))
    union = int(np.count_nonzero(left | right))
    foreground = int(np.count_nonzero(left)) + int(np.count_nonzero(right))
    equal = int(np.count_nonzero(left == right))
    return {
        "track_id": current["track_id"],
        "previous_sequence": previous["sequence"],
        "current_sequence": current["sequence"],
        "sequence_gap": current["sequence"] - previous["sequence"],
        "carried_state_comparison": int(carried),
        "mask_agreement_percent": 100.0 * equal / left.size,
        "foreground_iou_percent": 100.0 * intersection / union if union else 100.0,
        "foreground_dice_percent": 200.0 * intersection / foreground if foreground else 100.0,
        "center_displacement_pixels": math.hypot(current["center_x"] - previous["center_x"],
                                                  current["center_y"] - previous["center_y"]),
        "exact_value_equal": int(previous["mask_sha256"] == current["mask_sha256"]),
    }


def evaluate_geometry(packets: list[dict], paths: list[Path], run_index: int) -> tuple[list[dict], list[dict], dict | None, dict[str, dict | None]]:
    current: dict[str, dict] = {}
    updates: dict[str, list[dict]] = {}
    states: dict[str, list[dict]] = {}
    state_pairs: list[dict] = []
    update_pairs: list[dict] = []
    for packet, path in zip(packets, paths):
        sequence = int(packet["输出序号"])
        contours = contour_material(packet, path)
        updated_tracks: set[str] = set()
        if packet["包类型"] == "FullSnapshot":
            current = {}
        for change in packet["簇变化"]:
            track = change["相机跟踪候选编号"]
            if track is None:
                continue
            if change["帧内簇编号"] is None:
                if change["变化类型"] in {"Lost", "Removed"}:
                    current.pop(track, None)
                continue
            image_width, image_height = packet["图像尺寸WH"]
            sample = geometry_sample(change, contours, sequence, image_width, image_height)
            current[track] = sample
            updated_tracks.add(track)
            track_updates = updates.setdefault(track, [])
            if track_updates:
                update_pairs.append({"run_index": run_index, "comparison_kind": "material_update",
                                     **compare_geometry(track_updates[-1], sample, False)})
            track_updates.append(sample)
        for track, sample in sorted(current.items(), key=lambda item: int(item[0])):
            state_sample = dict(sample)
            state_sample["sequence"] = sequence
            track_states = states.setdefault(track, [])
            if track_states:
                state_pairs.append({"run_index": run_index, "comparison_kind": "published_state",
                                    **compare_geometry(track_states[-1], state_sample, track not in updated_tracks)})
            track_states.append(state_sample)
    track_rows: list[dict] = []
    for track in sorted(set(states) | set(updates), key=int):
        track_updates = updates.get(track, [])
        track_states = states.get(track, [])
        material_pairs = [row for row in update_pairs if row["track_id"] == track]
        published_pairs = [row for row in state_pairs if row["track_id"] == track]
        target_pairs = material_pairs if material_pairs else published_pairs
        displacements = [row["center_displacement_pixels"] for row in target_pairs]
        ious = [row["foreground_iou_percent"] for row in target_pairs]
        layer_counts: dict[str, int] = {}
        for sample in track_updates:
            layer_counts[sample["candidate_layer"]] = layer_counts.get(sample["candidate_layer"], 0) + 1
        dominant_layer = max(layer_counts, key=lambda value: (layer_counts[value], value), default=None)
        track_rows.append({
            "run_index": run_index,
            "track_id": track,
            "first_sequence": track_states[0]["sequence"] if track_states else None,
            "last_sequence": track_states[-1]["sequence"] if track_states else None,
            "published_state_frame_count": len(track_states),
            "material_update_count": len(track_updates),
            "material_pair_count": len(material_pairs),
            "published_state_pair_count": len(published_pairs),
            "target_comparison_kind": "material_update" if material_pairs else "published_state",
            "dominant_candidate_layer": dominant_layer,
            "border_touch_percent": (100.0 * sum(sample["touches_border"] for sample in track_updates) /
                                     len(track_updates)) if track_updates else None,
            "bbox_coverage_p50_percent": percentile([sample["bbox_coverage_percent"] for sample in track_updates], 50),
            "current_depth_support_p50_percent": percentile(
                [sample["current_depth_support_percent"] for sample in track_updates], 50),
            "median_pixels": percentile([sample["pixels"] for sample in track_updates], 50),
            "distinct_32x32_value_count": len({sample["mask_sha256"] for sample in track_updates}),
            "exact_repeat_pair_percent": (100.0 * sum(row["exact_value_equal"] for row in target_pairs) / len(target_pairs)) if target_pairs else None,
            "contour_iou_p50": percentile(ious, 50),
            "contour_iou_p95": percentile(ious, 95),
            "center_displacement_p50_px": percentile(displacements, 50),
            "center_displacement_p95_px": percentile(displacements, 95),
            "center_displacement_max_px": max(displacements) if displacements else None,
        })
    eligible = [row for row in track_rows if row["material_pair_count"] > 0 or row["published_state_pair_count"] > 0]
    main = max(eligible, key=lambda row: (row["material_update_count"], row["published_state_frame_count"],
                                          row["median_pixels"] or 0.0, -int(row["track_id"])), default=None)
    layer_mains = {}
    for layer in ("全画面账本区域", "触边开放候选", "非触边闭合候选"):
        candidates = [row for row in eligible if row["dominant_candidate_layer"] == layer]
        layer_mains[layer] = max(candidates, key=lambda row: (
            row["material_update_count"], row["published_state_frame_count"],
            row["median_pixels"] or 0.0, -int(row["track_id"])), default=None)
    return track_rows, state_pairs + update_pairs, main, layer_mains


def evaluate_runs(run_roots: list[Path], output: Path) -> dict:
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if len(run_roots) < 1:
        raise ValueError("At least one completed stream run is required")
    output.mkdir(parents=True)
    loaded = [load_run(root.resolve()) for root in run_roots]
    packet_lists = [item[0] for item in loaded]
    lengths = [len(packets) for packets in packet_lists]
    consistent_length = len(set(lengths)) == 1
    hashes = [[hashlib.sha256(normalized_packet(packet)).hexdigest() for packet in packets] for packets in packet_lists]
    deterministic = consistent_length and all(value == hashes[0] for value in hashes[1:])
    events: list[dict] = []
    metrics: list[dict] = []
    all_valid = True
    reconstruction_ok = True
    frame_local_id_reassignments = 0
    static_lifecycle_events = 0
    tentative_removed_count = 0
    added_candidate_count = 0
    run_max_active_tracks: list[int] = []
    geometry_tracks: list[dict] = []
    geometry_pairs: list[dict] = []
    main_geometry: list[dict] = []
    layer_main_geometry: list[dict] = []
    for run_index, (packets, paths) in enumerate(loaded, start=1):
        previous_sequence = 0
        track_by_frame_cluster: dict[int, str] = {}
        incremental_state: dict[str, dict] = {}
        base_sequence: str | None = None
        max_active_tracks = 0
        try:
            active = reconstruct(packets)
        except Exception as error:
            all_valid = False
            reconstruction_ok = False
            events.append({"run_index": run_index, "event": "reconstruction_failure", "detail": str(error)})
            active = {}
        for packet_index, packet in enumerate(packets, start=1):
            sequence = int(packet["输出序号"])
            if sequence != previous_sequence + 1:
                all_valid = False
                events.append({"run_index": run_index, "event": "sequence_gap", "detail": f"{previous_sequence}->{sequence}"})
            previous_sequence = sequence
            if packet["包类型"] == "FullSnapshot":
                incremental_state = {}
                base_sequence = packet["输出序号"]
            elif packet["包类型"] == "Delta" and packet["依赖全量序号"] != base_sequence:
                all_valid = False
                events.append({"run_index": run_index, "event": "base_snapshot_mismatch",
                               "detail": f"packet={packet['输出序号']} base={packet['依赖全量序号']} expected={base_sequence}"})
            for change in packet["簇变化"]:
                change_type = change["变化类型"]
                if change_type == "Added":
                    added_candidate_count += 1
                if change_type == "Removed" and change["遮挡"]["依据"] == "tentative_candidate_not_reobserved":
                    tentative_removed_count += 1
                frame_cluster = change["帧内簇编号"]
                track = change["相机跟踪候选编号"]
                if frame_cluster is not None and track is not None:
                    old = track_by_frame_cluster.get(frame_cluster)
                    if old is not None and old != track:
                        # 本帧簇编号 only identifies one source frame.  Its
                        # reuse across frames cannot prove a physical track-ID
                        # switch, but it is useful to expose as a source-label
                        # churn diagnostic.
                        frame_local_id_reassignments += 1
                        events.append({"run_index": run_index, "event": "frame_local_id_reassignment",
                                       "detail": f"frame_cluster={frame_cluster} {old}->{track}"})
                    track_by_frame_cluster[frame_cluster] = track
                if change_type in {"Lost", "Reappeared", "Occluded"}:
                    static_lifecycle_events += 1
                    events.append({"run_index": run_index, "event": "lifecycle_event", "detail": f"{change_type}:track={track}"})
                if frame_cluster is None:
                    if change_type in {"Lost", "Removed"}:
                        incremental_state.pop(track, None)
                    elif change_type == "Occluded" and track in incremental_state:
                        incremental_state[track]["跟踪状态"] = "Occluded"
                        incremental_state[track]["时效"] = copy.deepcopy(change["时效"])
                else:
                    incremental_state[track] = copy.deepcopy(change)
            max_active_tracks = max(max_active_tracks, len(incremental_state))
            metrics.append({"run_index": run_index, "packet_index": packet_index, "output_sequence": sequence,
                            "packet_type": packet["包类型"], "active_tracks_after_packet": len(incremental_state)})
        if incremental_state != active:
            all_valid = False
            reconstruction_ok = False
            events.append({"run_index": run_index, "event": "incremental_reconstruction_mismatch", "detail": "final state differs"})
        run_max_active_tracks.append(max_active_tracks)
        track_rows, pair_rows, main, layer_mains = evaluate_geometry(packets, paths, run_index)
        geometry_tracks.extend(track_rows)
        geometry_pairs.extend(pair_rows)
        if main is not None:
            main_geometry.append(main)
        for layer, row in layer_mains.items():
            if row is not None:
                layer_main_geometry.append({**row, "candidate_layer": layer})
    candidate_presence_pass = all(value > 0 for value in run_max_active_tracks)
    main_contour_iou_p50 = percentile([row["contour_iou_p50"] for row in main_geometry
                                      if row["contour_iou_p50"] is not None], 50)
    main_center_displacement_p95 = percentile([row["center_displacement_p95_px"] for row in main_geometry
                                               if row["center_displacement_p95_px"] is not None], 95)
    main_geometry_present = len(main_geometry) == len(run_roots)
    main_contour_target_pass = main_geometry_present and main_contour_iou_p50 is not None and main_contour_iou_p50 >= 99.0
    layer_summaries = {}
    for layer in ("全画面账本区域", "触边开放候选", "非触边闭合候选"):
        rows = [row for row in layer_main_geometry if row["candidate_layer"] == layer]
        layer_summaries[layer] = {
            "run_count": len(rows),
            "contour_iou_p50": percentile([row["contour_iou_p50"] for row in rows
                                           if row["contour_iou_p50"] is not None], 50),
            "center_displacement_p95_px": percentile([row["center_displacement_p95_px"] for row in rows
                                                       if row["center_displacement_p95_px"] is not None], 95),
            "current_depth_support_p50_percent": percentile([
                row["current_depth_support_p50_percent"] for row in rows
                if row["current_depth_support_p50_percent"] is not None], 50),
            "status": "diagnostic_no_gate",
        }
    foreground_summary = layer_summaries["非触边闭合候选"]
    foreground_target_applicable = foreground_summary["run_count"] > 0
    foreground_contour_target_pass = (
        foreground_summary["run_count"] == len(run_roots) and
        foreground_summary["contour_iou_p50"] is not None and
        foreground_summary["contour_iou_p50"] >= 99.0
    )
    geometry_target_kind = "nonborder_closed_candidate" if foreground_target_applicable else "coverage_main_fallback"
    geometry_target_pass = foreground_contour_target_pass if foreground_target_applicable else main_contour_target_pass
    layer_summaries["非触边闭合候选"]["status"] = (
        "hard_gate" if foreground_target_applicable else "not_present_fallback_to_coverage_main"
    )
    decision = {
        "format": "PCS.ClusterStabilityDecision/1", "scope": "static replay evidence only",
        "run_count": len(run_roots), "frames_per_run": lengths, "packet_validation_pass": all_valid,
        "normalized_determinism_pass": deterministic, "reconstruction_pass": reconstruction_ok,
        "frame_local_id_reassignment_count": frame_local_id_reassignments,
        "static_lifecycle_event_count": static_lifecycle_events,
        "tentative_removed_count": tentative_removed_count,
        "added_candidate_count": added_candidate_count,
        "run_max_active_tracks": run_max_active_tracks,
        "candidate_presence_pass": candidate_presence_pass,
        "main_geometry_by_run": [{"run_index": row["run_index"], "track_id": row["track_id"],
                                  "dominant_candidate_layer": row["dominant_candidate_layer"],
                                  "border_touch_percent": row["border_touch_percent"],
                                  "bbox_coverage_p50_percent": row["bbox_coverage_p50_percent"],
                                  "current_depth_support_p50_percent": row["current_depth_support_p50_percent"],
                                  "material_update_count": row["material_update_count"],
                                  "distinct_32x32_value_count": row["distinct_32x32_value_count"],
                                  "contour_iou_p50": row["contour_iou_p50"],
                                  "center_displacement_p95_px": row["center_displacement_p95_px"]}
                                 for row in main_geometry],
        "candidate_layer_main_geometry_by_run": [
            {"run_index": row["run_index"], "candidate_layer": row["candidate_layer"],
             "track_id": row["track_id"], "material_update_count": row["material_update_count"],
             "distinct_32x32_value_count": row["distinct_32x32_value_count"],
             "contour_iou_p50": row["contour_iou_p50"],
             "center_displacement_p95_px": row["center_displacement_p95_px"],
             "current_depth_support_p50_percent": row["current_depth_support_p50_percent"]}
            for row in layer_main_geometry],
        "candidate_layer_summaries": layer_summaries,
        "foreground_contour_target_applicable": foreground_target_applicable,
        "foreground_contour_target_percent": 99.0,
        "foreground_contour_target_pass": foreground_contour_target_pass,
        "geometry_target_kind": geometry_target_kind,
        "geometry_target_pass": geometry_target_pass,
        "main_contour_iou_p50": main_contour_iou_p50,
        "main_contour_iou_target_percent": 99.0,
        "main_contour_target_pass": main_contour_target_pass,
        "main_center_displacement_p95_px": main_center_displacement_p95,
        "center_displacement_status": "baseline_only_no_gate",
        "pass": (all_valid and deterministic and reconstruction_ok and candidate_presence_pass and
                 static_lifecycle_events == 0 and geometry_target_pass),
        "not_proven": ["physical_scene_static", "dynamic_tracking", "world_identity", "absolute_depth_accuracy"],
    }
    (output / "run_decision.json").write_text(json.dumps(decision, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    with (output / "packet_metrics.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=["run_index", "packet_index", "output_sequence", "packet_type", "active_tracks_after_packet"])
        writer.writeheader()
        writer.writerows(metrics)
    with (output / "events.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=["run_index", "event", "detail"])
        writer.writeheader()
        writer.writerows(events)
    with (output / "track_geometry_metrics.csv").open("w", newline="", encoding="utf-8") as file:
        fields = list(geometry_tracks[0]) if geometry_tracks else ["run_index", "track_id"]
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(geometry_tracks)
    pair_fields = ["run_index", "comparison_kind", "track_id", "previous_sequence", "current_sequence", "sequence_gap",
                   "carried_state_comparison", "mask_agreement_percent", "foreground_iou_percent", "foreground_dice_percent",
                   "center_displacement_pixels", "exact_value_equal"]
    with (output / "geometry_pairs.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=pair_fields)
        writer.writeheader()
        writer.writerows(geometry_pairs)
    return decision


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", required=True, type=Path, nargs="+", help="Completed cluster_stream output directories")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(evaluate_runs(args.runs, args.output.resolve()), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
