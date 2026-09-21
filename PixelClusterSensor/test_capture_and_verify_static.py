"""Synthetic integration test for the capture-to-static-gate orchestrator."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from capture_and_verify_static import capture_and_verify
from test_cluster_stream import make_static_sequence


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    project = Path(__file__).resolve().parent.parent
    parser.add_argument("--exe", type=Path, default=project / "x64/Release/PixelClusterSensor.exe")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    try:
        source = make_static_sequence(root / "source", 4)
        report = capture_and_verify(executable=args.exe, output=root / "run", frames=4, repetitions=3,
                                    replay_source=source, minimum_cluster_pixels=10,
                                    retained_cluster_pixels=5, maximum_tentative_match_cost=123_456)
        check(report["status"] == "pass" and report["matrix"]["decision"]["pass"], "Capture-to-gate did not pass")
        check(report["minimum_cluster_pixels"] == 10 and report["retained_cluster_pixels"] == 5 and
              report["maximum_tentative_match_cost"] == 123_456,
              "Capture orchestrator did not preserve tracker gate configuration")
        for name in ("capture/sequence.json", "capture/export_manifest.json", "matrix/run_manifest.json", "matrix/decision/run_decision.json", "run_manifest.json"):
            check((root / "run" / name).is_file(), f"Missing orchestration evidence: {name}")
        (root / "report.json").write_text(json.dumps({"status": "pass", "run": report}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print("PASS capture_and_verify_static", flush=True)
    except Exception as error:
        (root / "report.json").write_text(json.dumps({"status": "fail", "error": str(error)}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        raise


if __name__ == "__main__":
    main()
