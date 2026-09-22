"""Diagnose which tracked cluster candidates explain local T3 frame-to-frame motion."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from cluster_protocol import unpack_ring, validate_cluster_packet


def _material(root: Path, relative: str) -> Path:
    path = (root / relative).resolve()
    if root.resolve() not in path.parents or not path.is_file():
        raise ValueError(f"Sequence material escapes capture root: {relative}")
    return path


def _packet_paths(control_run: Path) -> list[Path]:
    paths = sorted((control_run / "control/cluster_packets").glob("packet_*/packet.json"))
    if not paths:
        raise ValueError("Control run has no cluster packets")
    return paths


def _contour_material(packet: dict, path: Path) -> bytes:
    descriptor = packet["材料"]["精确轮廓链"]
    raw = (path.parent / descriptor["文件"]).read_bytes()
    if len(raw) != descriptor["字节数"] or hashlib.sha256(raw).hexdigest() != descriptor["SHA256"]:
        raise ValueError("Contour material integrity mismatch")
    return raw


def _apply_packet(state: dict[str, tuple[dict, bytes]], packet: dict, contours: bytes) -> None:
    if packet["包类型"] == "FullSnapshot":
        state.clear()
    if packet["包类型"] == "Heartbeat":
        return
    for entry in packet["簇变化"]:
        track_id = entry["相机跟踪候选编号"]
        if track_id is None:
            continue
        if entry["变化类型"] in {"Lost", "Removed"}:
            state.pop(track_id, None)
        elif entry["帧内簇编号"] is None:
            if track_id in state:
                retained, retained_contours = state[track_id]
                retained = dict(retained)
                retained["跟踪状态"] = "Occluded"
                retained["时效"] = entry["时效"]
                state[track_id] = (retained, retained_contours)
        else:
            state[track_id] = (entry, contours)


def _cluster_mask(entry: dict, contours: bytes, width: int, height: int) -> np.ndarray:
    mask = np.zeros((height, width), dtype=np.uint8)
    rings = entry["轮廓"]
    depths = []
    for index, ring in enumerate(rings):
        depth = 0
        parent = ring["父环索引"]
        seen = {index}
        while parent != -1:
            if parent in seen or not 0 <= parent < len(rings):
                raise ValueError("Contour hierarchy cycle during T3 rasterization")
            seen.add(parent)
            depth += 1
            parent = rings[parent]["父环索引"]
        depths.append(depth)
    for index in sorted(range(len(rings)), key=lambda value: (depths[value], value)):
        ring = rings[index]
        offset = ring["字节偏移"]
        length = (ring["有效位数"] + 7) // 8
        points = unpack_ring(ring["起点XY"], ring["点数"],
                             contours[offset:offset + length], ring["有效位数"])
        cv2.fillPoly(mask, [np.asarray(points, dtype=np.int32)], 0 if ring["内环"] else 1)
    return mask.astype(bool)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def evaluate(sequence_path: Path, control_run: Path, output: Path) -> dict:
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    output.mkdir(parents=True)
    sequence_path = sequence_path.resolve()
    sequence = json.loads(sequence_path.read_text(encoding="utf-8"))
    if sequence.get("格式") != "PCS.RawSequence/1":
        raise ValueError("Expected PCS.RawSequence/1")
    frames = sequence.get("帧列表")
    packet_paths = _packet_paths(control_run.resolve())
    if not isinstance(frames, list) or len(frames) != len(packet_paths):
        raise ValueError("Raw frames and cluster packets must have equal nonzero counts")
    colors = [np.asarray(Image.open(_material(sequence_path.parent, frame["彩图"])).convert("RGB"),
                         dtype=np.uint8) for frame in frames]
    height, width = colors[0].shape[:2]
    pixels = width * height
    if any(color.shape != colors[0].shape for color in colors):
        raise ValueError("Color dimensions change inside the capture")

    state: dict[str, tuple[dict, bytes]] = {}
    rows = []
    selected_ids = []
    review_candidates = []
    for frame_index, packet_path in enumerate(packet_paths, start=1):
        validate_cluster_packet(packet_path)
        packet = json.loads(packet_path.read_text(encoding="utf-8"))
        contours = _contour_material(packet, packet_path)
        _apply_packet(state, packet, contours)
        if frame_index == 1:
            continue
        delta = np.max(np.abs(colors[frame_index - 1].astype(np.int16) -
                              colors[frame_index - 2].astype(np.int16)), axis=2)
        motion = delta >= 30
        motion_pixels = int(np.count_nonzero(motion))
        candidates = []
        for track_id, (entry, entry_contours) in state.items():
            if entry["跟踪状态"] == "Occluded" or entry["轮廓"] is None:
                continue
            mask = _cluster_mask(entry, entry_contours, width, height)
            cluster_pixels = int(np.count_nonzero(mask))
            overlap = int(np.count_nonzero(mask & motion))
            score = (overlap / np.sqrt(max(1, motion_pixels * cluster_pixels))) if overlap else 0.0
            x, y, box_width, box_height = entry["范围XYWH"]
            excessive_ledger = bool(entry["触及视野边界"] and box_width * box_height > pixels * 0.5)
            candidates.append((score, overlap, -cluster_pixels, str(track_id), entry, excessive_ledger, mask))
        eligible = [item for item in candidates if not item[5]]
        selected = max(eligible, default=None)
        active = motion_pixels * 100.0 / pixels >= 0.1
        selected_id = selected[3] if active and selected and selected[1] > 0 else None
        if active:
            selected_ids.append(selected_id)
            review_candidates.append((frame_index, colors[frame_index - 1], motion,
                                      None if selected_id is None else selected[6], selected_id))
        rows.append({
            "frame_index": frame_index,
            "motion_pixels": motion_pixels,
            "motion_percent": motion_pixels * 100.0 / pixels,
            "eligible_candidate_count": len(eligible),
            "selected_track_id": "" if selected_id is None else selected_id,
            "selected_overlap_pixels": 0 if selected is None else selected[1],
            "selected_motion_recall_percent": 0.0 if selected is None or motion_pixels == 0 else selected[1] * 100.0 / motion_pixels,
            "selected_cluster_pixels": 0 if selected is None else -selected[2],
            "selected_score": 0.0 if selected is None else selected[0],
            "selected_bbox_xywh": "" if selected is None else json.dumps(selected[4]["范围XYWH"], separators=(",", ":")),
            "selected_track_state": "" if selected is None else selected[4]["跟踪状态"],
            "excluded_large_border_candidate_count": sum(1 for item in candidates if item[5]),
        })

    observed_ids = [value for value in selected_ids if value is not None]
    counts = {track_id: observed_ids.count(track_id) for track_id in sorted(set(observed_ids), key=int)}
    dominant_id = max(counts, key=counts.get) if counts else None
    active_state_ids = [row["selected_track_id"] for row in rows
                        if row["selected_track_id"] and row["selected_track_state"] == "Active"]
    active_state_counts = {track_id: active_state_ids.count(track_id)
                           for track_id in sorted(set(active_state_ids), key=int)}
    active_state_dominant = max(active_state_counts, key=active_state_counts.get) if active_state_counts else None
    transitions = sum(1 for left, right in zip(selected_ids, selected_ids[1:])
                      if left is not None and right is not None and left != right)
    longest = 0
    current = 0
    previous = object()
    for track_id in selected_ids:
        if track_id is not None and track_id == previous:
            current += 1
        elif track_id is not None:
            current = 1
        else:
            current = 0
        longest = max(longest, current)
        previous = track_id
    review_samples = []
    if review_candidates:
        indices = np.linspace(0, len(review_candidates) - 1, min(6, len(review_candidates)), dtype=int)
        for review_index in sorted(set(int(value) for value in indices)):
            frame_index, color, motion, selected_mask, selected_id = review_candidates[review_index]
            overlay = color.copy()
            overlay[motion] = ((overlay[motion].astype(np.uint16) + [255, 0, 0]) // 2).astype(np.uint8)
            if selected_mask is not None:
                outlines, _ = cv2.findContours(selected_mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(overlay, outlines, -1, (0, 255, 0), 2)
            name = f"frame_{frame_index:06d}_overlay.png"
            path = output / name
            Image.fromarray(overlay).save(path)
            review_samples.append({"frame_index": frame_index, "selected_track_id": selected_id,
                                   "overlay": name, "overlay_sha256": _sha256(path),
                                   "legend": "red=motion; green=selected cluster contour"})

    result = {
        "格式": "PCS.T3MotionTrackingDiagnostic/1",
        "状态": "diagnostic_not_gate",
        "输入清单": str(sequence_path),
        "输入清单SHA256": hashlib.sha256(sequence_path.read_bytes()).hexdigest(),
        "控制运行": str(control_run.resolve()),
        "帧数": len(frames),
        "活动帧对数": len(selected_ids),
        "有候选重叠活动帧数": len(observed_ids),
        "所选跟踪号种类数": len(counts),
        "主导跟踪号": dominant_id,
        "主导跟踪号活动帧占比": None if dominant_id is None else counts[dominant_id] * 100.0 / len(selected_ids),
        "相邻活动帧跟踪号切换数": transitions,
        "同一跟踪号最长连续活动样本数": longest,
        "跟踪号命中次数": counts,
        "已激活候选命中帧数": len(active_state_ids),
        "已激活候选跟踪号种类数": len(active_state_counts),
        "已激活主导跟踪号": active_state_dominant,
        "已激活主导跟踪号占比": None if active_state_dominant is None else
            active_state_counts[active_state_dominant] * 100.0 / len(active_state_ids),
        "已激活跟踪号命中次数": active_state_counts,
        "选择规则": "motion-mask-overlap-cosine/1; exclude border candidate bboxes covering more than 50 percent of frame",
        "视觉复核样本": review_samples,
        "人工视觉复核": "pending",
        "未证明": ["现实目标身份", "所选簇等于完整手部", "动态跟踪正确性", "绝对深度精度"],
    }
    with (output / "frame_metrics.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output / "diagnostic.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                                             encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", type=Path, required=True)
    parser.add_argument("--control-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(evaluate(args.sequence, args.control_run, args.output), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
