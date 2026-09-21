"""Synthetic integration test for repeated cluster stability matrix execution."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from run_cluster_stability_matrix import run_matrix
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
        replay = make_static_sequence(root / "source", 4)
        result = run_matrix(executable=args.exe, replay=replay, output=root / "matrix", frames=4, repetitions=3,
                            minimum_cluster_pixels=10, retained_cluster_pixels=5,
                            maximum_tentative_match_cost=123_456)
        check(result["status"] == "pass" and result["decision"]["pass"], "Static matrix did not pass")
        check(len(result["runs"]) == 3 and all(item["status"] == "pass" for item in result["runs"]), "Matrix did not complete three runs")
        config = json.loads((root / "matrix" / "config_snapshot.json").read_text(encoding="utf-8"))
        check(config["minimum_cluster_pixels"] == 10 and config["retained_cluster_pixels"] == 5 and
              config["maximum_tentative_match_cost"] == 123_456 and config["max_missing_frames"] == 3 and
              config["occlusion_confirmation_frames"] == 2,
              "Matrix did not preserve tracker gate configuration")
        for name in ("run_manifest.json", "config_snapshot.json", "decision/run_decision.json", "decision/packet_metrics.csv", "decision/events.csv"):
            check((root / "matrix" / name).is_file(), f"Missing matrix evidence: {name}")
        (root / "report.json").write_text(json.dumps({"status": "pass", "matrix": result}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print("PASS repeated_cluster_stability_matrix", flush=True)
    except Exception as error:
        (root / "report.json").write_text(json.dumps({"status": "fail", "error": str(error)}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        raise


if __name__ == "__main__":
    main()
