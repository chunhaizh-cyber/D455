"""Run the P3 cluster-control shadow gate on a PCS.RawSequence/1 or live camera."""

from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import time

from client import Client
from cluster_control import ClusterControlSession
from cluster_tracker import ClusterTracker
from convert_cluster_observation import convert


def git_revision(project: Path) -> str | None:
    result = subprocess.run(["git", "rev-parse", "HEAD"], cwd=project, capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else None


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int((len(ordered) - 1) * fraction))]


def run_gate(*, executable: Path, output: Path, frames: int, replay: Path | None,
             camera: bool = False, serial: str = "", minimum_cluster_pixels: int = 1,
             retained_cluster_pixels: int | None = None, confirmation_frames: int = 5) -> dict:
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if not 2 <= frames <= 600:
        raise ValueError("frames must be 2..600")
    if camera == (replay is not None):
        raise ValueError("Choose exactly one source: camera or replay")
    retained = retained_cluster_pixels if retained_cluster_pixels is not None else max(1, minimum_cluster_pixels // 2)
    output.mkdir(parents=True)
    report = {
        "格式": "PCS.ClusterControlGate/1", "状态": "运行中", "来源": "实时相机" if camera else "历史回放",
        "请求帧数": frames, "已处理帧数": 0, "扫描结果数": 0, "跟踪结果数": 0,
        "观察材料校验通过": False, "目标相机跟踪候选编号": None, "错误": None,
        "边界": ["P3 Python影子门禁", "不证明动态跟踪质量", "不证明实时性能", "不创建世界身份"],
        "Git修订": git_revision(Path(__file__).resolve().parent.parent),
        "输入清单": None if replay is None else str(replay.resolve()),
        "输入清单SHA256": None if replay is None else hashlib.sha256(replay.resolve().read_bytes()).hexdigest(),
        "配置": {"最小簇像素数": minimum_cluster_pixels, "保留簇像素数": retained,
                 "确认帧数": confirmation_frames},
    }
    metrics = []
    controller = ClusterControlSession(
        output / "control",
        ClusterTracker(max_missing_frames=3, confirmation_frames=confirmation_frames,
                       minimum_new_cluster_pixels=minimum_cluster_pixels,
                       minimum_retained_cluster_pixels=retained,
                       occlusion_confirmation_frames=2),
        maximum_results_per_task=min(1024, frames + 8),
        maximum_result_age_frames=max(120, frames + 8),
    )
    scan_task = controller.command("开始扫描", {"区域": "全画面"})["任务编号"]
    track_task = None
    observe_task = None
    try:
        with Client(executable, output / "producer", max_packets=32) as client:
            parameters = ({"来源": "实时相机", "设备序列号": serial} if camera else
                          {"来源": "目录回放", "清单": str(replay.resolve())})
            client.call("打开设备", parameters)
            for index in range(1, frames + 1):
                started = time.perf_counter()
                observation = client.call("获取单帧观察")
                try:
                    source_path = Path(observation["材料路径"])
                    snapshot_root = output / "snapshots" / f"frame_{index:06d}"
                    convert(source_path, snapshot_root, minimum_cluster_pixels)
                    snapshot_path = snapshot_root / "packet.json"
                    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
                    descriptor = snapshot["材料"]["精确轮廓链"]
                    contours = (snapshot_root / descriptor["文件"]).read_bytes()
                    packet_path = controller.process_source(snapshot, contours)
                finally:
                    client.call("释放观察材料", {"输出序号": observation["输出序号"]})

                if report["目标相机跟踪候选编号"] is None:
                    state = controller.tracker.active_snapshot()
                    if state:
                        selected = max(state.values(), key=lambda entry: (entry["像素数"], -int(entry["相机跟踪候选编号"])))
                        target = selected["相机跟踪候选编号"]
                        report["目标相机跟踪候选编号"] = target
                        track_task = controller.command("开始跟踪簇候选", {
                            "相机跟踪候选编号": target, "最长帧数": max(1, frames - index),
                            "自我绑定令牌": "cluster-control-gate",
                        })["任务编号"]
                        observe_task = controller.command("观察簇候选", {
                            "相机跟踪候选编号": target, "截止帧数": min(30, max(1, frames - index)),
                        })["任务编号"]

                scan_results = controller.command("读取簇级结果", {
                    "任务编号": scan_task, "最多包数": 8,
                })["结果"]
                track_results = ([] if track_task is None else controller.command("读取簇级结果", {
                    "任务编号": track_task, "最多包数": 8,
                })["结果"])
                observe_results = ([] if observe_task is None else controller.command("读取簇级结果", {
                    "任务编号": observe_task, "最多包数": 8,
                })["结果"])
                for result in observe_results:
                    detail = result.get("详细材料")
                    if detail is None:
                        continue
                    read = controller.command("读取详细材料", {"句柄": detail["句柄"],
                                                                "预期SHA256": detail["SHA256"]})
                    raw = base64.b64decode(read["数据Base64"])
                    if hashlib.sha256(raw).hexdigest() != detail["SHA256"]:
                        raise ValueError("Observed contour material hash mismatch")
                    controller.command("释放详细材料", {"句柄": detail["句柄"]})
                    report["观察材料校验通过"] = True
                report["已处理帧数"] += 1
                report["扫描结果数"] += len(scan_results)
                report["跟踪结果数"] += len(track_results)
                metrics.append({
                    "frame_index": index, "source_frame": observation["源帧号"],
                    "packet_path": str(packet_path), "scan_results": len(scan_results),
                    "track_results": len(track_results), "observe_results": len(observe_results),
                    "elapsed_ms": round((time.perf_counter() - started) * 1000.0, 4),
                })
            if track_task is not None:
                controller.command("停止跟踪簇候选", {"任务编号": track_task})
            controller.command("停止扫描", {"任务编号": scan_task})
            client.call("关闭设备")
        controller.write_manifest()
        elapsed = [row["elapsed_ms"] for row in metrics]
        report["处理耗时毫秒"] = {"p50": percentile(elapsed, 0.50), "p95": percentile(elapsed, 0.95),
                                  "p99": percentile(elapsed, 0.99), "max": max(elapsed) if elapsed else None}
        report["性能门禁已应用"] = False
        report["通过"] = (report["已处理帧数"] == frames and report["扫描结果数"] == frames and
                          report["跟踪结果数"] > 0 and report["观察材料校验通过"] and
                          report["目标相机跟踪候选编号"] is not None)
        report["状态"] = "通过" if report["通过"] else "失败"
    except Exception as error:
        report["状态"] = "失败"
        report["通过"] = False
        report["错误"] = str(error)
        raise
    finally:
        with (output / "frame_metrics.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=["frame_index", "source_frame", "packet_path",
                                                      "scan_results", "track_results", "observe_results", "elapsed_ms"])
            writer.writeheader()
            writer.writerows(metrics)
        (output / "run_manifest.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                                   encoding="utf-8")
    return report


def main() -> None:
    project = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, default=project / "x64/Release/PixelClusterSensor.exe")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--camera", action="store_true")
    source.add_argument("--replay", type=Path)
    parser.add_argument("--serial", default="")
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--minimum-cluster-pixels", type=int, default=1)
    parser.add_argument("--retained-cluster-pixels", type=int)
    parser.add_argument("--confirmation-frames", type=int, default=5)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run_gate(executable=args.exe, output=args.output, frames=args.frames, replay=args.replay,
                              camera=args.camera, serial=args.serial,
                              minimum_cluster_pixels=args.minimum_cluster_pixels,
                              retained_cluster_pixels=args.retained_cluster_pixels,
                              confirmation_frames=args.confirmation_frames), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
