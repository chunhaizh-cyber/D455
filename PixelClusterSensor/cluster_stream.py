"""Bounded P3 bridge from PixelClusterSensor observations to tracked cluster packets.

The child C++ process remains the sole camera owner.  This bridge is a separate
Python process for evaluation and shadow use; it does not alter PCS.Observation/1
or claim that cluster tracking is already wired into the C++ service.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import shutil
import time

from client import Client
from cluster_protocol import validate_cluster_packet
from cluster_tracker import ClusterTracker, reconstruct
from convert_cluster_observation import convert


def write_tracked_packet(packet: dict, snapshot: Path, target: Path) -> Path:
    target.mkdir(parents=True)
    descriptor = packet["材料"]["精确轮廓链"]
    shutil.copy2(snapshot / descriptor["文件"], target / descriptor["文件"])
    path = target / "packet.json"
    path.write_text(json.dumps(packet, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    validate_cluster_packet(path)
    return path


def run_stream(*, executable: Path, output: Path, frames: int, replay: Path | None, serial: str = "", max_missing_frames: int = 2,
               minimum_cluster_pixels: int = 1) -> dict:
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if not 1 <= frames <= 2048:
        raise ValueError("frames must be 1..2048")
    if minimum_cluster_pixels < 1:
        raise ValueError("minimum_cluster_pixels must be positive")
    output.mkdir(parents=True)
    report = {
        "status": "running", "format": "PCS.ClusterStreamRun/1", "requested_frames": frames,
        "source": "directory_replay" if replay else "live_camera", "packets": [], "error": None,
        "quality_promotion": "not_evaluated", "tracking_validation": "not_established_for_real_dynamic_motion",
        "minimum_cluster_pixels": minimum_cluster_pixels,
    }
    tracker = ClusterTracker(max_missing_frames=max_missing_frames)
    packet_documents = []
    metrics = []
    events = []
    try:
        with Client(executable, output / "observation_source") as client:
            parameters = {"来源": "目录回放", "清单": str(replay.resolve())} if replay else {"来源": "实时相机", "设备序列号": serial}
            opened = client.call("打开设备", parameters)
            report["session"] = opened["会话标识"]
            report["configuration_version"] = opened["配置版本"]
            for index in range(1, frames + 1):
                started = time.perf_counter()
                observation = client.call("获取单帧观察")
                source_path = Path(observation["材料路径"])
                snapshot_root = output / "snapshots" / f"packet_{index:06d}"
                convert_result = convert(source_path, snapshot_root, minimum_cluster_pixels)
                snapshot_path = snapshot_root / "packet.json"
                snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
                tracked = tracker.update(snapshot)
                tracked_root = output / "cluster_packets" / f"packet_{index:06d}"
                tracked_path = write_tracked_packet(tracked, snapshot_root, tracked_root)
                client.call("释放观察材料", {"输出序号": observation["输出序号"]})
                elapsed = (time.perf_counter() - started) * 1000.0
                packet_documents.append(json.loads(tracked_path.read_text(encoding="utf-8")))
                metrics.append({
                    "frame_index": index, "source_frame": observation["源帧号"], "output_sequence": tracked["输出序号"],
                    "packet_type": tracked["包类型"], "cluster_changes": len(tracked["簇变化"]), "elapsed_ms": round(elapsed, 4),
                    "active_tracks": len(tracker.tracks), "source_processing_ms": observation["指标"].get("处理毫秒"),
                })
                for entry in tracked["簇变化"]:
                    events.append({"frame_index": index, "track_id": entry["相机跟踪候选编号"], "frame_cluster_id": entry["帧内簇编号"],
                                   "change": entry["变化类型"], "state": entry["跟踪状态"]})
                report["packets"].append(str(tracked_path))
            client.call("关闭设备")
        reconstructed = reconstruct(packet_documents)
        report["status"] = "pass"
        report["active_tracks_after_reconstruction"] = len(reconstructed)
        report["source_observation_count"] = len(packet_documents)
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (output / "run_manifest.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        with (output / "packet_metrics.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=["frame_index", "source_frame", "output_sequence", "packet_type", "cluster_changes", "elapsed_ms", "active_tracks", "source_processing_ms"])
            writer.writeheader()
            writer.writerows(metrics)
        with (output / "events.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=["frame_index", "track_id", "frame_cluster_id", "change", "state"])
            writer.writeheader()
            writer.writerows(events)
    return report


def main() -> None:
    project = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, default=project / "x64/Release/PixelClusterSensor.exe")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--camera", action="store_true")
    source.add_argument("--replay", type=Path)
    parser.add_argument("--serial", default="")
    parser.add_argument("--frames", type=int, default=1)
    parser.add_argument("--max-missing-frames", type=int, default=2)
    parser.add_argument("--minimum-cluster-pixels", type=int, default=1)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(run_stream(executable=args.exe, output=args.output.resolve(), frames=args.frames, replay=args.replay,
                                serial=args.serial, max_missing_frames=args.max_missing_frames,
                                minimum_cluster_pixels=args.minimum_cluster_pixels), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
