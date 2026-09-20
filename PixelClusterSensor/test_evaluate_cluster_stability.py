"""Synthetic repeated-static test for the cluster stability decision bridge."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from cluster_stream import run_stream
from evaluate_cluster_stability import evaluate_runs
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
        sequence = make_static_sequence(root / "source", 3)
        runs = []
        for index in range(1, 4):
            target = root / f"run_{index}"
            run_stream(executable=args.exe, output=target, frames=3, replay=sequence)
            runs.append(target)
        decision = evaluate_runs(runs, root / "decision")
        check(decision["pass"], f"Static stability decision failed: {decision}")
        check(decision["normalized_determinism_pass"], "Repeated static stream was not deterministic")
        check(decision["track_id_switch_count"] == 0, "Static stream changed track ID")
        check((root / "decision" / "packet_metrics.csv").is_file() and (root / "decision" / "events.csv").is_file(),
              "Decision evidence files are missing")
        (root / "report.json").write_text(json.dumps({"status": "pass", "decision": decision}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print("PASS repeated_static_stability_decision", flush=True)
    except Exception as error:
        (root / "report.json").write_text(json.dumps({"status": "fail", "error": str(error)}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        raise


if __name__ == "__main__":
    main()
