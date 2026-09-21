"""Synthetic positive and negative tests for T3/T7 capture-content gates."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image

from evaluate_cluster_scenario_capture import evaluate_capture


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def write_sequence(root: Path, kind: str, frames: int = 60) -> Path:
    frame_root = root / "frames"
    frame_root.mkdir(parents=True)
    records = []
    for index in range(frames):
        color = np.full((48, 64, 3), 180, dtype=np.uint8)
        depth = np.full((48, 64), 1000, dtype=np.uint16)
        if kind == "local":
            left = 5 + index % 20
            color[18:30, left:left + 10] = [220, 20, 20]
        elif kind == "global":
            color[:] = 0 if index % 2 else 255
        elif kind == "hole":
            color[12:38, 18:46] = [10, 10, 10]
            depth[12:38, 18:46] = 0
        color_name = f"frames/{index:06d}_color.png"
        depth_name = f"frames/{index:06d}_depth.png"
        Image.fromarray(color, "RGB").save(root / color_name)
        Image.fromarray(depth).save(root / depth_name)
        records.append({"彩图": color_name, "深度": depth_name, "源帧号": str(index + 1),
                        "彩图时间戳毫秒": index * 33.0, "深度时间戳毫秒": index * 33.0,
                        "时间域": "synthetic"})
    sequence = {"格式": "PCS.RawSequence/1", "材料来源": "合成测试", "设备标识": "synthetic",
                "帧列表": records}
    path = root / "sequence.json"
    path.write_text(json.dumps(sequence, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    report = {"status": "running", "tests": []}

    def run(name, fn):
        started = time.perf_counter()
        detail = fn()
        report["tests"].append({"name": name, "status": "pass",
                                "seconds": round(time.perf_counter() - started, 4), "detail": detail})
        print(f"PASS {name}", flush=True)

    try:
        local = evaluate_capture(write_sequence(root / "local", "local"), "T3_local_motion", root / "local_result")
        global_motion = evaluate_capture(write_sequence(root / "global", "global"), "T3_local_motion",
                                         root / "global_result")
        hole = evaluate_capture(write_sequence(root / "hole", "hole"), "T7_dark_depth_hole", root / "hole_result")
        no_hole = evaluate_capture(write_sequence(root / "no_hole", "local"), "T7_dark_depth_hole",
                                   root / "no_hole_result")
        run("localized_motion_capture_passes_T3_content_gate", lambda: check(
            local["通过"] and local["指标"]["local_pair_percent"] >= 20.0,
            "Localized motion was not accepted"))
        run("whole_frame_change_fails_T3_locality_gate", lambda: check(
            not global_motion["通过"] and not global_motion["硬门禁"]["motion_p95_below_35_percent"],
            "Whole-frame change was mislabeled as local motion"))
        run("dark_depth_hole_capture_passes_T7_content_gate", lambda: check(
            hole["通过"] and hole["指标"]["dark_hole_pixels_p50"] > 0,
            "Dark depth-hole evidence was not accepted"))
        run("dark_region_without_depth_hole_fails_T7_gate", lambda: check(
            not no_hole["通过"] and
            not no_hole["硬门禁"]["dark_hole_p50_at_least_0_03_percent_of_frame"],
            "Color darkness alone was mistaken for depth-hole evidence"))
        report["status"] = "pass"
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
