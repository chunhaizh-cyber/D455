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
                       serial: str = "", minimum_cluster_pixels: int = 1,
                       clustering_mode: str = "深度主导", confirmation_frames: int = 5,
                       retained_cluster_pixels: int | None = None,
                       maximum_tentative_match_cost: int = 200_000,
                       max_missing_frames: int = 3,
                       occlusion_confirmation_frames: int = 2,
                       allow_tentative_growth_association: bool = False) -> dict:
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if minimum_cluster_pixels < 1:
        raise ValueError("minimum_cluster_pixels must be positive")
    if retained_cluster_pixels is None:
        retained_cluster_pixels = max(1, minimum_cluster_pixels // 2)
    if not 1 <= retained_cluster_pixels <= minimum_cluster_pixels:
        raise ValueError("retained_cluster_pixels must be between 1 and minimum_cluster_pixels")
    if clustering_mode not in {"轮廓主导", "深度主导"}:
        raise ValueError("clustering_mode must be 轮廓主导 or 深度主导")
    if not 1 <= confirmation_frames <= 30:
        raise ValueError("confirmation_frames must be 1..30")
    if not 0 <= maximum_tentative_match_cost < 1_000_000:
        raise ValueError("maximum_tentative_match_cost must be 0..999999")
    if not 1 <= occlusion_confirmation_frames < max_missing_frames:
        raise ValueError("occlusion_confirmation_frames must be at least 1 and below max_missing_frames")
    output.mkdir(parents=True)
    report = {
        "format": "PCS.StaticCaptureAndVerify/1", "status": "running", "frames": frames,
        "repetitions": repetitions, "capture_source": "directory_replay" if replay_source else "live_camera",
        "capture": None, "matrix": None, "minimum_cluster_pixels": minimum_cluster_pixels, "error": None,
        "retained_cluster_pixels": retained_cluster_pixels,
        "clustering_mode": clustering_mode,
        "confirmation_frames": confirmation_frames,
        "maximum_tentative_match_cost": maximum_tentative_match_cost,
        "max_missing_frames": max_missing_frames,
        "occlusion_confirmation_frames": occlusion_confirmation_frames,
        "allow_tentative_growth_association": allow_tentative_growth_association,
    }
    try:
        capture = export_sequence(executable=executable, output=output / "capture", frames=frames,
                                  replay=replay_source, serial=serial)
        matrix = run_matrix(executable=executable, replay=Path(capture["sequence"]), output=output / "matrix",
                            frames=frames, repetitions=repetitions, minimum_cluster_pixels=minimum_cluster_pixels,
                            clustering_mode=clustering_mode, confirmation_frames=confirmation_frames,
                            retained_cluster_pixels=retained_cluster_pixels,
                            maximum_tentative_match_cost=maximum_tentative_match_cost,
                            max_missing_frames=max_missing_frames,
                            occlusion_confirmation_frames=occlusion_confirmation_frames,
                            allow_tentative_growth_association=allow_tentative_growth_association)
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
    parser.add_argument("--retained-cluster-pixels", type=int)
    parser.add_argument("--clustering-mode", choices=["轮廓主导", "深度主导"], default="深度主导")
    parser.add_argument("--confirmation-frames", type=int, default=5)
    parser.add_argument("--maximum-tentative-match-cost", type=int, default=200_000)
    parser.add_argument("--max-missing-frames", type=int, default=3)
    parser.add_argument("--occlusion-confirmation-frames", type=int, default=2)
    parser.add_argument("--allow-tentative-growth-association", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(capture_and_verify(executable=args.exe, output=args.output.resolve(), frames=args.frames,
                                        repetitions=args.repetitions, replay_source=args.replay_source,
                                        serial=args.serial, minimum_cluster_pixels=args.minimum_cluster_pixels,
                                        clustering_mode=args.clustering_mode,
                                        confirmation_frames=args.confirmation_frames,
                                        retained_cluster_pixels=args.retained_cluster_pixels,
                                        maximum_tentative_match_cost=args.maximum_tentative_match_cost,
                                        max_missing_frames=args.max_missing_frames,
                                        occlusion_confirmation_frames=args.occlusion_confirmation_frames,
                                        allow_tentative_growth_association=args.allow_tentative_growth_association), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
