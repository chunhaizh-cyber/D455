"""End-to-end synthetic integration test for scenario capture, content gate, and cluster control gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image

import capture_and_verify_cluster_scenario as scenario_gate
from evaluate_t3_motion_tracking import evaluate as evaluate_t3_tracking
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
        result = scenario_gate.capture_and_verify(executable=args.exe, output=root / "run", scenario="T3_local_motion",
                                                  frames=60, replay_source=replay, minimum_cluster_pixels=1,
                                                  retained_cluster_pixels=1, confirmation_frames=1,
                                                  allow_tentative_growth_association=True)
        run("capture_content_and_control_gates_pass_together", lambda: check(
            result["自动门禁通过"] and result["场景内容门禁"]["通过"] and result["控制门禁"]["通过"],
            "Combined scenario gate did not pass"))
        control_manifest = json.loads((root / "run/control_gate/run_manifest.json").read_text(encoding="utf-8"))
        run("scenario_gate_preserves_growth_probe_switch", lambda: check(
            control_manifest["配置"]["允许未确认候选增长关联"] is True,
            "Scenario gate dropped the growth-association switch"))
        run("combined_gate_keeps_raw_and_decision_evidence", lambda: check(
            (root / "run/capture/sequence.json").is_file() and
            (root / "run/scenario_evaluation/scenario_decision.json").is_file() and
            (root / "run/visual_review/review.html").is_file() and
            (root / "run/visual_review/review_manifest.json").is_file() and
            (root / "run/control_gate/run_manifest.json").is_file(),
            "Combined scenario evidence is incomplete"))
        diagnostic = evaluate_t3_tracking(root / "run/capture/sequence.json", root / "run/control_gate",
                                          root / "motion_tracking_diagnostic")
        run("T3_motion_tracking_diagnostic_reads_complete_control_stream", lambda: check(
            diagnostic["状态"] == "diagnostic_not_gate" and diagnostic["帧数"] == 60 and
            diagnostic["活动帧对数"] > 0 and
            (root / "motion_tracking_diagnostic/frame_metrics.csv").is_file(),
            "T3 motion tracking diagnostic did not preserve its evidence boundary"))

        def interrupted_gate_keeps_completed_stages():
            original = scenario_gate.run_gate
            try:
                scenario_gate.run_gate = lambda **_: (_ for _ in ()).throw(RuntimeError("forced control failure"))
                try:
                    scenario_gate.capture_and_verify(
                        executable=args.exe, output=root / "interrupted", scenario="T3_local_motion",
                        frames=60, replay_source=replay, minimum_cluster_pixels=1,
                        retained_cluster_pixels=1, confirmation_frames=1)
                except RuntimeError as error:
                    check(str(error) == "forced control failure", "Unexpected interrupted-gate failure")
                else:
                    raise AssertionError("Interrupted gate unexpectedly passed")
            finally:
                scenario_gate.run_gate = original
            manifest = json.loads((root / "interrupted/run_manifest.json").read_text(encoding="utf-8"))
            check(manifest["状态"] == "失败" and manifest["采集"]["帧数"] == 60,
                  "Interrupted gate lost completed capture evidence")
            check(manifest["场景内容门禁"]["通过"] and manifest["视觉复核材料"]["状态"] == "pending",
                  "Interrupted gate lost completed evaluation evidence")
            check(manifest["控制门禁"] is None and manifest["错误"] == "forced control failure",
                  "Interrupted gate recorded an invalid control result")
            return {"capture_preserved": True, "content_gate_preserved": True,
                    "visual_review_preserved": True, "control_gate": None}
        run("interrupted_control_gate_preserves_completed_stages", interrupted_gate_keeps_completed_stages)
        report["status"] = "pass"
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
