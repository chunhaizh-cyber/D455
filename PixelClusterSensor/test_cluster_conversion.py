"""P1 integration test: synthesize PCS.Observation/1, convert it, and independently read the cluster snapshot."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image

from client import Client
from cluster_protocol import validate_cluster_packet
from convert_cluster_observation import convert, encode_valid_rings


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def calibration(width: int, height: int) -> dict:
    return {"宽": width, "高": height, "焦距X": 100.0, "焦距Y": 100.0, "主点X": (width - 1) / 2,
            "主点Y": (height - 1) / 2, "畸变模型": 0, "畸变系数": [0.0] * 5}


def make_replay(root: Path) -> Path:
    root.mkdir(parents=True)
    color = np.full((24, 32, 3), [40, 80, 160], dtype=np.uint8)
    depth = np.full((24, 32), 1000, dtype=np.uint16)
    Image.fromarray(color).save(root / "color.png")
    Image.fromarray(depth).save(root / "depth.png")
    data = {
        "格式": "PCS.RawSequence/1", "材料来源": "合成夹具", "设备标识": "cluster-conversion-synthetic", "深度单位米": 0.001,
        "彩图内参": calibration(32, 24), "深度内参": calibration(32, 24),
        "深度到彩图外参": {"旋转列优先": [1, 0, 0, 0, 1, 0, 0, 0, 1], "平移米": [0, 0, 0]},
        "帧列表": [{"彩图": "color.png", "深度": "depth.png", "源帧号": "1", "彩图时间戳毫秒": 0.0,
                    "深度时间戳毫秒": 0.0, "时间域": "synthetic_clock"}],
    }
    path = root / "sequence.json"
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    project = Path(__file__).resolve().parent.parent
    parser.add_argument("--exe", type=Path, default=project / "x64/Release/PixelClusterSensor.exe")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    report = {"status": "running", "tests": []}

    def run(name, fn):
        started = time.perf_counter()
        detail = fn()
        report["tests"].append({"name": name, "status": "pass", "seconds": round(time.perf_counter() - started, 4), "detail": detail})
        print(f"PASS {name}", flush=True)

    try:
        replay = make_replay(root / "replay")
        with Client(args.exe, root / "producer") as client:
            opened = client.call("打开设备", {"来源": "目录回放", "清单": str(replay)})
            reference = client.call("获取单帧观察")
            client.call("关闭设备")
        source = Path(reference["材料路径"])
        converted_a = convert(source, root / "cluster_a")
        converted_b = convert(source, root / "cluster_b")
        packet_a = (root / "cluster_a" / "packet.json").read_bytes()
        packet_b = (root / "cluster_b" / "packet.json").read_bytes()
        data = json.loads(packet_a)
        run("cluster_packet_validates", lambda: validate_cluster_packet(root / "cluster_a" / "packet.json"))
        run("same_source_conversion_is_byte_deterministic", lambda: check(packet_a == packet_b, "Repeated conversion differs"))
        run("one_source_cluster_becomes_one_cluster_record", lambda: check(len(data["簇变化"]) == 1 and converted_a["clusters"] == 1, "Cluster count changed"))
        run("converter_does_not_fake_precise_depth", lambda: check(data["簇变化"][0]["距离"]["模式"] == "UnknownDistance" and
                                                                    data["簇变化"][0]["距离"]["值米"] is None, "In-range depth was upgraded"))
        run("converter_has_no_tracking_or_world_identity", lambda: check(data["簇变化"][0]["相机跟踪候选编号"] is None and
                                                                           data["簇变化"][0]["自我绑定令牌"] is None, "Unexpected identity claim"))
        run("converter_emits_exact_chain_material", lambda: check(converted_a["contour_bytes"] > 0 and
                                                                      data["材料"]["精确轮廓链"]["编码"] == "PCS.ContourChain8/1", "Missing contour chain"))
        def rejects_degenerate_source_rings_without_inventing_geometry():
            points = np.array([[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1], [5, 5, 2]], dtype=np.int32)
            rings, raw, rejected = encode_valid_rings([
                {"起始点": 0, "点数": 4, "内环": False, "父轮廓索引": -1, "闭合区域边界": True},
                {"起始点": 4, "点数": 1, "内环": True, "父轮廓索引": 0, "闭合区域边界": True},
            ], points)
            check(len(rings) == 1 and raw and rejected == 1, "Degenerate ring was not excluded")
        run("converter_excludes_degenerate_source_rings", rejects_degenerate_source_rings_without_inventing_geometry)
        filtered = convert(source, root / "cluster_filtered", minimum_cluster_pixels=769)
        filtered_packet = json.loads((root / "cluster_filtered" / "packet.json").read_text(encoding="utf-8"))
        run("converter_downgrades_below_threshold_clusters_to_unknown", lambda: check(
            filtered["clusters"] == 0 and not filtered_packet["簇变化"] and
            filtered_packet["全局覆盖摘要"]["未知像素数"] == 32 * 24 and
            filtered_packet["指标"]["低于最小簇像素数簇数"] == 1,
            "Below-threshold cluster was not explicitly downgraded"))
        report["status"] = "pass"
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
