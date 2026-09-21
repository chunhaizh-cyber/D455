"""Visual-evidence renderer tests for bounded T3/T7 samples and overlay semantics."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image

from render_cluster_scenario_review import render_review
from test_evaluate_cluster_scenario_capture import write_sequence


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


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
        local = render_review(write_sequence(root / "local", "local"), "T3_local_motion", root / "local_review")
        hole = render_review(write_sequence(root / "hole", "hole"), "T7_dark_depth_hole", root / "hole_review")
        run("visual_review_is_bounded_to_six_samples", lambda: check(
            len(local["样本"]) == 6 and len(hole["样本"]) == 6,
            "Visual review sample budget changed"))
        local_overlay = np.asarray(Image.open(root / "local_review" / local["样本"][2]["overlay"]).convert("RGB"))
        hole_overlay_path = root / "hole_review" / hole["样本"][0]["overlay"]
        hole_overlay = np.asarray(Image.open(hole_overlay_path).convert("RGB"))
        run("T3_overlay_contains_motion_and_bbox_colors", lambda: check(
            np.any((local_overlay[:, :, 0] > 180) & (local_overlay[:, :, 1] < 100)) and
            np.any((local_overlay[:, :, 1] > 180) & (local_overlay[:, :, 0] < 100)),
            "T3 overlay lacks motion or bbox evidence"))
        run("T7_overlay_contains_dark_hole_color", lambda: check(
            np.any((hole_overlay[:, :, 0] == 255) & (hole_overlay[:, :, 1] == 0) &
                   (hole_overlay[:, :, 2] == 0)), "T7 overlay lacks dark-hole evidence"))
        run("review_manifest_hash_matches_overlay", lambda: check(
            hashlib.sha256(hole_overlay_path.read_bytes()).hexdigest() == hole["样本"][0]["overlay_sha256"],
            "Review manifest hash does not match overlay"))
        report["status"] = "pass"
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
