"""Integration test for the PCS.RawSequence/1 cluster-control gate runner."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from run_cluster_control_gate import run_gate
from test_cluster_conversion import make_replay


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


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
        replay = make_replay(root / "replay")
        manifest = json.loads(replay.read_text(encoding="utf-8"))
        original = manifest["帧列表"][0]
        manifest["帧列表"] = []
        for index in range(1, 5):
            frame = dict(original)
            frame["源帧号"] = str(index)
            frame["彩图时间戳毫秒"] = original["彩图时间戳毫秒"] + index * 33.0
            frame["深度时间戳毫秒"] = original["深度时间戳毫秒"] + index * 33.0
            manifest["帧列表"].append(frame)
        replay.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        result = run_gate(executable=args.exe, output=root / "gate", frames=4, replay=replay,
                          confirmation_frames=1, allow_tentative_growth_association=True)
        run("raw_sequence_control_gate_passes", lambda: check(
            result["通过"] and result["扫描结果数"] == 4 and result["跟踪结果数"] == 3 and
            result["观察材料校验通过"] and result["配置"]["允许未确认候选增长关联"],
            "Integrated control gate did not pass or preserve the growth switch"))
        run("gate_writes_standard_evidence", lambda: check(
            (root / "gate/run_manifest.json").is_file() and
            (root / "gate/frame_metrics.csv").is_file() and
            (root / "gate/control/control_manifest.json").is_file(),
            "Control gate evidence is incomplete"))
        report["status"] = "pass"
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
