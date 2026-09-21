"""Capture one RGBD sequence and run the repeated cluster static-stability gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from export_raw_sequence import export_sequence
from run_cluster_stability_matrix import run_matrix


def capture_and_verify(*, executable: Path, output: Path, frames: int = 600,
                       repetitions: int = 3, replay_source: Path | None = None,
                       serial: str = "", minimum_cluster_pixels: int = 1) -> dict:
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if minimum_cluster_pixels < 1:
        raise ValueError("minimum_cluster_pixels must be positive")
    output.mkdir(parents=True)
    report = {
        "format": "PCS.StaticCaptureAndVerify/1", "status": "running", "frames": frames,
        "repetitions": repetitions, "capture_source": "directory_replay" if replay_source else "live_camera",
        "capture": None, "matrix": None, "minimum_cluster_pixels": minimum_cluster_pixels, "error": None,
    }
    try:
        capture = export_sequence(executable=executable, output=output / "capture", frames=frames,
                                  replay=replay_source, serial=serial)
        matrix = run_matrix(executable=executable, replay=Path(capture["sequence"]), output=output / "matrix",
                            frames=frames, repetitions=repetitions, minimum_cluster_pixels=minimum_cluster_pixels)
        report["capture"] = capture
        report["matrix"] = {"status": matrix["status"], "decision": matrix.get("decision")}
        report["status"] = "pass" if matrix["status"] == "pass" else "fail"
        report["completed_unix_ms"] = int(time.time() * 1000)
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (output / "run_manifest.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    project = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, default=project / "x64/Release/PixelClusterSensor.exe")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--camera", action="store_true")
    source.add_argument("--replay-source", type=Path, help="Only for synthetic/integration verification")
    parser.add_argument("--serial", default="")
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--minimum-cluster-pixels", type=int, default=1)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(capture_and_verify(executable=args.exe, output=args.output.resolve(), frames=args.frames,
                                        repetitions=args.repetitions, replay_source=args.replay_source,
                                        serial=args.serial, minimum_cluster_pixels=args.minimum_cluster_pixels), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
