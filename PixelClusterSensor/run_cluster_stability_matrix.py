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
               repetitions: int = 3, max_missing_frames: int = 2) -> dict:
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if not 1 <= frames <= 2048:
        raise ValueError("frames must be 1..2048")
    if not 1 <= repetitions <= 5:
        raise ValueError("repetitions must be 1..5")
    replay = replay.resolve()
    if not replay.is_file():
        raise ValueError(f"Replay manifest does not exist: {replay}")
    sequence = json.loads(replay.read_text(encoding="utf-8"))
    if sequence.get("格式") != "PCS.RawSequence/1":
        raise ValueError("Replay must be PCS.RawSequence/1")
    if len(sequence.get("帧列表", [])) < frames:
        raise ValueError("Replay has fewer frames than requested")
    output.mkdir(parents=True)
    project = Path(__file__).resolve().parent.parent
    manifest = {
        "format": "PCS.ClusterStabilityMatrix/1", "status": "running", "input_manifest": str(replay),
        "input_manifest_sha256": sha256(replay), "requested_frames": frames, "repetitions": repetitions,
        "max_missing_frames": max_missing_frames, "git_revision": git_revision(project), "runs": [], "error": None,
    }
    try:
        run_roots = []
        for index in range(1, repetitions + 1):
            target = output / f"repeat_{index:03d}"
            report = run_stream(executable=executable, output=target, frames=frames, replay=replay,
                                max_missing_frames=max_missing_frames)
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
    parser.add_argument("--max-missing-frames", type=int, default=2)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run_matrix(executable=args.exe, replay=args.replay, output=args.output.resolve(), frames=args.frames,
                                repetitions=args.repetitions, max_missing_frames=args.max_missing_frames), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
