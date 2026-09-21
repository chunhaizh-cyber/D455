"""Run repeated PCS cluster streams and produce one static-stability decision."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from cluster_stream import run_stream
from evaluate_cluster_stability import evaluate_runs


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_revision(project: Path) -> str | None:
    try:
        return subprocess.check_output(["git", "-C", str(project), "rev-parse", "HEAD"], text=True, timeout=10).strip()
    except (OSError, subprocess.SubprocessError):
        return None


def run_matrix(*, executable: Path, replay: Path, output: Path, frames: int = 600,
               repetitions: int = 3, max_missing_frames: int = 3, minimum_cluster_pixels: int = 1,
               clustering_mode: str = "深度主导", confirmation_frames: int = 5, start_frame: int = 1,
               retained_cluster_pixels: int | None = None,
               maximum_tentative_match_cost: int = 200_000,
               occlusion_confirmation_frames: int = 2) -> dict:
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if not 1 <= frames <= 2048:
        raise ValueError("frames must be 1..2048")
    if not 1 <= start_frame <= 2048 or start_frame - 1 + frames > 2048:
        raise ValueError("start_frame and frames must fit the 2048-frame source bound")
    if not 1 <= repetitions <= 5:
        raise ValueError("repetitions must be 1..5")
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
    replay = replay.resolve()
    if not replay.is_file():
        raise ValueError(f"Replay manifest does not exist: {replay}")
    sequence = json.loads(replay.read_text(encoding="utf-8"))
    if sequence.get("格式") != "PCS.RawSequence/1":
        raise ValueError("Replay must be PCS.RawSequence/1")
    if len(sequence.get("帧列表", [])) < start_frame - 1 + frames:
        raise ValueError("Replay has fewer frames than requested")
    output.mkdir(parents=True)
    project = Path(__file__).resolve().parent.parent
    manifest = {
        "format": "PCS.ClusterStabilityMatrix/1", "status": "running", "input_manifest": str(replay),
        "input_manifest_sha256": sha256(replay), "requested_frames": frames, "repetitions": repetitions,
        "max_missing_frames": max_missing_frames, "minimum_cluster_pixels": minimum_cluster_pixels,
        "retained_cluster_pixels": retained_cluster_pixels,
        "clustering_mode": clustering_mode,
        "confirmation_frames": confirmation_frames,
        "maximum_tentative_match_cost": maximum_tentative_match_cost,
        "occlusion_confirmation_frames": occlusion_confirmation_frames,
        "start_frame": start_frame,
        "git_revision": git_revision(project), "runs": [], "error": None,
    }
    try:
        run_roots = []
        for index in range(1, repetitions + 1):
            target = output / f"repeat_{index:03d}"
            report = run_stream(executable=executable, output=target, frames=frames, replay=replay,
                                max_missing_frames=max_missing_frames, minimum_cluster_pixels=minimum_cluster_pixels,
                                clustering_mode=clustering_mode, confirmation_frames=confirmation_frames,
                                start_frame=start_frame, retained_cluster_pixels=retained_cluster_pixels,
                                maximum_tentative_match_cost=maximum_tentative_match_cost,
                                occlusion_confirmation_frames=occlusion_confirmation_frames)
            manifest["runs"].append({"index": index, "path": str(target), "status": report["status"]})
            run_roots.append(target)
        decision = evaluate_runs(run_roots, output / "decision")
        manifest["status"] = "pass" if decision["pass"] else "fail"
        manifest["decision"] = decision
    except Exception as error:
        manifest["status"] = "fail"
        manifest["error"] = str(error)
        raise
    finally:
        (output / "run_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        (output / "config_snapshot.json").write_text(json.dumps({
            "frames": frames, "repetitions": repetitions, "max_missing_frames": max_missing_frames,
            "minimum_cluster_pixels": minimum_cluster_pixels,
            "retained_cluster_pixels": retained_cluster_pixels,
            "clustering_mode": clustering_mode,
            "confirmation_frames": confirmation_frames,
            "maximum_tentative_match_cost": maximum_tentative_match_cost,
            "occlusion_confirmation_frames": occlusion_confirmation_frames,
            "start_frame": start_frame,
            "source": "PCS.RawSequence/1", "evaluation": "static_stability_only",
        }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    project = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, default=project / "x64/Release/PixelClusterSensor.exe")
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--max-missing-frames", type=int, default=3)
    parser.add_argument("--occlusion-confirmation-frames", type=int, default=2)
    parser.add_argument("--minimum-cluster-pixels", type=int, default=1)
    parser.add_argument("--retained-cluster-pixels", type=int)
    parser.add_argument("--clustering-mode", choices=["轮廓主导", "深度主导"], default="深度主导")
    parser.add_argument("--confirmation-frames", type=int, default=5)
    parser.add_argument("--maximum-tentative-match-cost", type=int, default=200_000)
    parser.add_argument("--start-frame", type=int, default=1)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run_matrix(executable=args.exe, replay=args.replay, output=args.output.resolve(), frames=args.frames,
                                repetitions=args.repetitions, max_missing_frames=args.max_missing_frames,
                                minimum_cluster_pixels=args.minimum_cluster_pixels,
                                clustering_mode=args.clustering_mode,
                                confirmation_frames=args.confirmation_frames,
                                start_frame=args.start_frame,
                                retained_cluster_pixels=args.retained_cluster_pixels,
                                maximum_tentative_match_cost=args.maximum_tentative_match_cost,
                                occlusion_confirmation_frames=args.occlusion_confirmation_frames), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
