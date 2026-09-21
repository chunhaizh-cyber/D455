"""End-to-end synthetic integration test for scenario capture, content gate, and cluster control gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image

from capture_and_verify_cluster_scenario import capture_and_verify
from test_cluster_conversion import make_replay


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def make_local_motion_replay(root: Path) -> Path:
    path = make_replay(root)
    sequence = json.loads(path.read_text(encoding="utf-8"))
    original = sequence["帧列表"][0]
    frames = []
    for index in range(60):
        color = np.full((24, 32, 3), 160, dtype=np.uint8)
        left = 3 + index % 16
        color[9:14, left:left + 5] = [230, 20, 20]
        name = f"color_{index:06d}.png"
        Image.fromarray(color).save(root / name)
        frames.append({"彩图": name, "深度": original["深度"], "源帧号": str(index + 1),
                       "彩图时间戳毫秒": index * 33.0, "深度时间戳毫秒": index * 33.0,
                       "时间域": original["时间域"]})
    sequence["帧列表"] = frames
    path.write_text(json.dumps(sequence, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def main() -> None:
    project = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, default=project / "x64/Release/PixelClusterSensor.exe")
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
        replay = make_local_motion_replay(root / "source")
        result = capture_and_verify(executable=args.exe, output=root / "run", scenario="T3_local_motion",
                                    frames=60, replay_source=replay, minimum_cluster_pixels=1,
                                    retained_cluster_pixels=1, confirmation_frames=1)
        run("capture_content_and_control_gates_pass_together", lambda: check(
            result["自动门禁通过"] and result["场景内容门禁"]["通过"] and result["控制门禁"]["通过"],
            "Combined scenario gate did not pass"))
        run("combined_gate_keeps_raw_and_decision_evidence", lambda: check(
            (root / "run/capture/sequence.json").is_file() and
            (root / "run/scenario_evaluation/scenario_decision.json").is_file() and
            (root / "run/control_gate/run_manifest.json").is_file(),
            "Combined scenario evidence is incomplete"))
        report["status"] = "pass"
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
