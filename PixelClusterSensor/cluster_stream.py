"""Bounded P3 bridge from PixelClusterSensor observations to tracked cluster packets.

The child C++ process remains the sole camera owner.  This bridge is a separate
Python process for evaluation and shadow use; it does not alter PCS.Observation/1
or claim that cluster tracking is already wired into the C++ service.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import time

from client import Client
from cluster_protocol import validate_cluster_packet
from cluster_tracker import ClusterTracker, reconstruct
from convert_cluster_observation import convert


def write_tracked_packet(packet: dict, contours: bytes, target: Path) -> Path:
    target.mkdir(parents=True)
    descriptor = packet["材料"]["精确轮廓链"]
    descriptor["字节数"] = len(contours)
    descriptor["SHA256"] = hashlib.sha256(contours).hexdigest()
    (target / descriptor["文件"]).write_bytes(contours)
    path = target / "packet.json"
    path.write_text(json.dumps(packet, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    validate_cluster_packet(path)
    return path


def run_stream(*, executable: Path, output: Path, frames: int, replay: Path | None, serial: str = "", max_missing_frames: int = 3,
               minimum_cluster_pixels: int = 1, clustering_mode: str = "深度主导", confirmation_frames: int = 5,
               start_frame: int = 1, retained_cluster_pixels: int | None = None,
               depth_split_min_support_pixels: int = 32,
               cross_color_merge_min_boundary_pixels: int = 8,
               allow_global_missing_depth_merge: bool = True,
               maximum_tentative_match_cost: int = 200_000,
               occlusion_confirmation_frames: int = 2) -> dict:
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if not 1 <= frames <= 2048:
        raise ValueError("frames must be 1..2048")
    if not 1 <= start_frame <= 2048 or start_frame - 1 + frames > 2048:
        raise ValueError("start_frame and frames must fit the 2048-frame source bound")
    if minimum_cluster_pixels < 1:
        raise ValueError("minimum_cluster_pixels must be positive")
    if retained_cluster_pixels is None:
        retained_cluster_pixels = max(1, minimum_cluster_pixels // 2)
    if not 1 <= retained_cluster_pixels <= minimum_cluster_pixels:
        raise ValueError("retained_cluster_pixels must be between 1 and minimum_cluster_pixels")
    if clustering_mode not in {"轮廓主导", "深度主导"}:
        raise ValueError("clustering_mode must be 轮廓主导 or 深度主导")
    if not 1 <= depth_split_min_support_pixels <= 100_000:
        raise ValueError("depth_split_min_support_pixels must be 1..100000")
    if not 1 <= cross_color_merge_min_boundary_pixels <= 10_000:
        raise ValueError("cross_color_merge_min_boundary_pixels must be 1..10000")
    if not 1 <= confirmation_frames <= 30:
        raise ValueError("confirmation_frames must be 1..30")
    if not 0 <= maximum_tentative_match_cost < 1_000_000:
        raise ValueError("maximum_tentative_match_cost must be 0..999999")
    if not 1 <= occlusion_confirmation_frames < max_missing_frames:
        raise ValueError("occlusion_confirmation_frames must be at least 1 and below max_missing_frames")
    output.mkdir(parents=True)
    report = {
        "status": "running", "format": "PCS.ClusterStreamRun/1", "requested_frames": frames,
        "source": "directory_replay" if replay else "live_camera", "packets": [], "error": None,
        "quality_promotion": "not_evaluated", "tracking_validation": "not_established_for_real_dynamic_motion",
        "minimum_cluster_pixels": minimum_cluster_pixels,
        "retained_cluster_pixels": retained_cluster_pixels,
        "clustering_mode": clustering_mode,
        "depth_split_min_support_pixels": depth_split_min_support_pixels,
        "cross_color_merge_min_boundary_pixels": cross_color_merge_min_boundary_pixels,
        "allow_global_missing_depth_merge": allow_global_missing_depth_merge,
        "confirmation_frames": confirmation_frames,
        "maximum_tentative_match_cost": maximum_tentative_match_cost,
        "max_missing_frames": max_missing_frames,
        "occlusion_confirmation_frames": occlusion_confirmation_frames,
        "start_frame": start_frame,
    }
    tracker = ClusterTracker(max_missing_frames=max_missing_frames, confirmation_frames=confirmation_frames,
                             minimum_new_cluster_pixels=minimum_cluster_pixels,
                             minimum_retained_cluster_pixels=retained_cluster_pixels,
                             maximum_tentative_match_cost=maximum_tentative_match_cost,
                             occlusion_confirmation_frames=occlusion_confirmation_frames)
    packet_documents = []
    metrics = []
    events = []
    try:
        with Client(executable, output / "observation_source") as client:
            parameters = {"来源": "目录回放", "清单": str(replay.resolve())} if replay else {"来源": "实时相机", "设备序列号": serial}
            opened = client.call("打开设备", parameters)
            report["session"] = opened["会话标识"]
            configured = client.call("设置处理配置", {
                "聚簇模式": clustering_mode,
                "深度拆分最小支持像素": depth_split_min_support_pixels,
                "跨颜色合并最小连续边界像素": cross_color_merge_min_boundary_pixels,
                "允许同彩图区域跨缺测全局合并": allow_global_missing_depth_merge,
            }, 预期配置版本=client.revision)
            report["configuration_version"] = configured["配置版本"]
            report["processing_configuration"] = configured["处理配置"]
            for _ in range(1, start_frame):
                skipped = client.call("获取单帧观察")
                client.call("释放观察材料", {"输出序号": skipped["输出序号"]})
            for index in range(1, frames + 1):
                started = time.perf_counter()
                observation = client.call("获取单帧观察")
                source_path = Path(observation["材料路径"])
                snapshot_root = output / "snapshots" / f"packet_{index:06d}"
                # Keep the tracker hysteresis band, but do not construct full
                # contour packages for fragments that no active track may retain.
                convert(source_path, snapshot_root, retained_cluster_pixels)
                snapshot_path = snapshot_root / "packet.json"
                snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
                # The cluster stream has its own contiguous sequence.  Upstream
                # observation numbers remain in packet_metrics for traceability.
                snapshot["包标识"] = f"cluster-{index}"
                snapshot["输出序号"] = str(index)
                snapshot["场景版本"] = str(index)
                snapshot_path.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                validate_cluster_packet(snapshot_path)
                snapshot_descriptor = snapshot["材料"]["精确轮廓链"]
                snapshot_contours = (snapshot_root / snapshot_descriptor["文件"]).read_bytes()
                tracked = tracker.update(snapshot, snapshot_contours)
                tracked_root = output / "cluster_packets" / f"packet_{index:06d}"
                if tracker.last_output_contours is None:
                    raise ValueError("Tracker did not provide output contour material")
                tracked_path = write_tracked_packet(tracked, tracker.last_output_contours, tracked_root)
                client.call("释放观察材料", {"输出序号": observation["输出序号"]})
                elapsed = (time.perf_counter() - started) * 1000.0
                packet_documents.append(json.loads(tracked_path.read_text(encoding="utf-8")))
                metrics.append({
                    "frame_index": index, "source_frame": observation["源帧号"], "output_sequence": tracked["输出序号"],
                    "packet_type": tracked["包类型"], "cluster_changes": len(tracked["簇变化"]), "elapsed_ms": round(elapsed, 4),
                    "active_tracks": len(tracker.tracks), "source_processing_ms": observation["指标"].get("处理毫秒"),
                    "filtered_new_clusters": tracked["指标"]["本帧过滤新候选数"],
                    "filtered_new_pixels": tracked["指标"]["本帧过滤新候选像素数"],
                    "source_cluster_count": observation["指标"].get("簇数量"),
                    "source_color_region_count": observation["指标"].get("彩图初始区域数"),
                    "source_depth_seed_count": observation["指标"].get("当前深度种子区域数"),
                    "source_ignored_small_depth_seed_count": observation["指标"].get("深度拆分忽略小种子数"),
                    "source_retained_nonlocal_small_depth_seed_count": observation["指标"].get("深度拆分无局部邻接保留小种子数"),
                    "source_forced_incompatible_depth_noise_merge_count": observation["指标"].get("强制吸收不相容深度噪点数"),
                    "source_cross_color_merge_count": observation["指标"].get("跨颜色连续深度合并数"),
                    "source_missing_compatible_depth_merge_count": observation["指标"].get("跨缺测相容深度合并数"),
                    "source_cross_color_boundary_candidate_count": observation["指标"].get("跨颜色连续边界候选数"),
                    "source_rejected_short_cross_color_boundary_count": observation["指标"].get("跨颜色短边界拒绝合并数"),
                    "source_enclosed_missing_inherited_pixels": observation["指标"].get("封闭缺深度继承像素数"),
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
            writer = csv.DictWriter(file, fieldnames=[
                "frame_index", "source_frame", "output_sequence", "packet_type", "cluster_changes", "elapsed_ms",
                "active_tracks", "source_processing_ms", "filtered_new_clusters", "filtered_new_pixels",
                "source_cluster_count", "source_color_region_count", "source_depth_seed_count",
                "source_ignored_small_depth_seed_count", "source_cross_color_merge_count",
                "source_retained_nonlocal_small_depth_seed_count",
                "source_forced_incompatible_depth_noise_merge_count",
                "source_missing_compatible_depth_merge_count",
                "source_cross_color_boundary_candidate_count", "source_rejected_short_cross_color_boundary_count",
                "source_enclosed_missing_inherited_pixels",
            ])
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
    parser.add_argument("--max-missing-frames", type=int, default=3)
    parser.add_argument("--occlusion-confirmation-frames", type=int, default=2)
    parser.add_argument("--minimum-cluster-pixels", type=int, default=1)
    parser.add_argument("--retained-cluster-pixels", type=int)
    parser.add_argument("--clustering-mode", choices=["轮廓主导", "深度主导"], default="深度主导")
    parser.add_argument("--depth-split-min-support-pixels", type=int, default=32)
    parser.add_argument("--cross-color-merge-min-boundary-pixels", type=int, default=8)
    parser.add_argument("--disable-global-missing-depth-merge", action="store_true")
    parser.add_argument("--confirmation-frames", type=int, default=5)
    parser.add_argument("--maximum-tentative-match-cost", type=int, default=200_000)
    parser.add_argument("--start-frame", type=int, default=1)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(run_stream(executable=args.exe, output=args.output.resolve(), frames=args.frames, replay=args.replay,
                                serial=args.serial, max_missing_frames=args.max_missing_frames,
                                minimum_cluster_pixels=args.minimum_cluster_pixels,
                                clustering_mode=args.clustering_mode,
                                depth_split_min_support_pixels=args.depth_split_min_support_pixels,
                                cross_color_merge_min_boundary_pixels=args.cross_color_merge_min_boundary_pixels,
                                allow_global_missing_depth_merge=not args.disable_global_missing_depth_merge,
                                confirmation_frames=args.confirmation_frames,
                                start_frame=args.start_frame,
                                retained_cluster_pixels=args.retained_cluster_pixels,
                                maximum_tentative_match_cost=args.maximum_tentative_match_cost,
                                occlusion_confirmation_frames=args.occlusion_confirmation_frames), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
