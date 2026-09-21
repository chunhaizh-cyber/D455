"""Evaluate whether a PCS.RawSequence/1 capture contains the intended T3 or T7 scene evidence."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image


SCENARIOS = {"T3_local_motion", "T7_dark_depth_hole"}


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int((len(ordered) - 1) * fraction))]


def _material(root: Path, relative: str) -> Path:
    path = (root / relative).resolve()
    if root.resolve() not in path.parents or not path.is_file():
        raise ValueError(f"Sequence material escapes capture root: {relative}")
    return path


def _read_frames(sequence_path: Path) -> tuple[dict, list[tuple[np.ndarray, np.ndarray]]]:
    sequence_path = sequence_path.resolve()
    sequence = json.loads(sequence_path.read_text(encoding="utf-8"))
    if sequence.get("格式") != "PCS.RawSequence/1":
        raise ValueError("Expected PCS.RawSequence/1")
    frames = sequence.get("帧列表")
    if not isinstance(frames, list) or not frames:
        raise ValueError("Raw sequence has no frames")
    decoded = []
    shape = None
    for frame in frames:
        color = np.asarray(Image.open(_material(sequence_path.parent, frame["彩图"])).convert("RGB"), dtype=np.uint8)
        depth = np.asarray(Image.open(_material(sequence_path.parent, frame["深度"])), dtype=np.uint16)
        if color.shape[:2] != depth.shape or (shape is not None and depth.shape != shape):
            raise ValueError("Color/depth dimensions change inside the capture")
        shape = depth.shape
        decoded.append((color, depth))
    return sequence, decoded


def evaluate_capture(sequence_path: Path, scenario: str, output: Path) -> dict:
    if scenario not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {scenario}")
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    output.mkdir(parents=True)
    sequence, frames = _read_frames(sequence_path)
    height, width = frames[0][1].shape
    pixels = width * height
    rows = []

    if scenario == "T3_local_motion":
        for index, ((previous, _), (current, _depth)) in enumerate(zip(frames, frames[1:]), start=2):
            delta = np.max(np.abs(current.astype(np.int16) - previous.astype(np.int16)), axis=2)
            motion = delta >= 30
            count = int(np.count_nonzero(motion))
            if count:
                ys, xs = np.where(motion)
                bbox_pixels = int((xs.max() - xs.min() + 1) * (ys.max() - ys.min() + 1))
            else:
                bbox_pixels = 0
            rows.append({"frame_index": index, "motion_pixels": count,
                         "motion_percent": count * 100.0 / pixels,
                         "motion_bbox_percent": bbox_pixels * 100.0 / pixels,
                         "dark_pixels": "", "dark_hole_pixels": "", "dark_hole_percent": ""})
        active = [row for row in rows if row["motion_percent"] >= 0.1]
        local = [row for row in active if row["motion_percent"] <= 35.0 and row["motion_bbox_percent"] <= 50.0]
        decision = {
            "active_pair_percent": len(active) * 100.0 / max(1, len(rows)),
            "local_pair_percent": len(local) * 100.0 / max(1, len(rows)),
            "motion_percent_p50": percentile([row["motion_percent"] for row in rows], 0.50),
            "motion_percent_p95": percentile([row["motion_percent"] for row in rows], 0.95),
            "motion_bbox_percent_p50": percentile([row["motion_bbox_percent"] for row in rows], 0.50),
            "motion_bbox_percent_p95": percentile([row["motion_bbox_percent"] for row in rows], 0.95),
        }
        hard_gates = {
            "minimum_60_frames": len(frames) >= 60,
            "active_pairs_at_least_20_percent": decision["active_pair_percent"] >= 20.0,
            "local_pairs_at_least_20_percent": decision["local_pair_percent"] >= 20.0,
            "motion_p95_below_35_percent": decision["motion_percent_p95"] <= 35.0,
            "motion_bbox_p95_below_50_percent": decision["motion_bbox_percent_p95"] <= 50.0,
        }
    else:
        for index, (color, depth) in enumerate(frames, start=1):
            luminance = (color[:, :, 0].astype(np.uint16) * 54 + color[:, :, 1].astype(np.uint16) * 183 +
                         color[:, :, 2].astype(np.uint16) * 19) // 256
            dark = luminance <= 60
            holes = depth == 0
            dark_pixels = int(np.count_nonzero(dark))
            dark_holes = int(np.count_nonzero(dark & holes))
            rows.append({"frame_index": index, "motion_pixels": "", "motion_percent": "",
                         "motion_bbox_percent": "", "dark_pixels": dark_pixels,
                         "dark_percent": dark_pixels * 100.0 / pixels,
                         "dark_hole_pixels": dark_holes,
                         "dark_hole_percent": dark_holes * 100.0 / max(1, dark_pixels)})
        minimum_hole_pixels = max(10, int(round(pixels * 0.0003)))
        evidence_frames = [row for row in rows if row["dark_hole_pixels"] >= minimum_hole_pixels]
        decision = {
            "dark_pixels_p50": percentile([float(row["dark_pixels"]) for row in rows], 0.50),
            "dark_percent_p50": percentile([row["dark_percent"] for row in rows], 0.50),
            "dark_hole_pixels_p50": percentile([float(row["dark_hole_pixels"]) for row in rows], 0.50),
            "dark_hole_percent_p50": percentile([row["dark_hole_percent"] for row in rows], 0.50),
            "frames_with_dark_hole_evidence_percent": len(evidence_frames) * 100.0 / len(rows),
        }
        hard_gates = {
            "minimum_60_frames": len(frames) >= 60,
            "dark_region_p50_at_least_0_3_percent": decision["dark_percent_p50"] >= 0.3,
            "dark_hole_p50_at_least_0_03_percent_of_frame":
                decision["dark_hole_pixels_p50"] >= minimum_hole_pixels,
            "dark_hole_evidence_in_at_least_80_percent_frames":
                decision["frames_with_dark_hole_evidence_percent"] >= 80.0,
        }

    result = {
        "格式": "PCS.ClusterScenarioCaptureDecision/1", "场景": scenario,
        "输入清单": str(sequence_path.resolve()),
        "输入清单SHA256": hashlib.sha256(sequence_path.resolve().read_bytes()).hexdigest(),
        "设备标识": sequence.get("设备标识"), "帧数": len(frames), "图像尺寸WH": [width, height],
        "指标": decision, "硬门禁": hard_gates, "通过": all(hard_gates.values()),
        "人工视觉复核": "pending",
        "未证明": ["目标现实身份", "场景真值", "动态跟踪正确性", "绝对深度精度"],
    }
    with (output / "scenario_metrics.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=["frame_index", "motion_pixels", "motion_percent",
                                                  "motion_bbox_percent", "dark_pixels", "dark_percent", "dark_hole_pixels",
                                                  "dark_hole_percent"])
        writer.writeheader()
        writer.writerows(rows)
    (output / "scenario_decision.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                                                     encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", type=Path, required=True)
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(evaluate_capture(args.sequence, args.scenario, args.output), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
