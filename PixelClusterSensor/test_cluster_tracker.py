"""P2 integration tests for deterministic short-term cluster tracking and delta reconstruction."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import shutil
import time

from client import Client
from cluster_protocol import validate_cluster_packet
from cluster_tracker import ClusterTracker, reconstruct, write_packets
from convert_cluster_observation import convert
from test_cluster_conversion import make_replay


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def write_packet(directory: Path, data: dict, source_contours: Path) -> Path:
    directory.mkdir(parents=True)
    shutil.copy2(source_contours, directory / "contours.bin")
    path = directory / "packet.json"
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    validate_cluster_packet(path)
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
            client.call("打开设备", {"来源": "目录回放", "清单": str(replay)})
            observations = [Path(client.call("获取单帧观察")["材料路径"]) for _ in range(1)]
            client.call("关闭设备")
        # The source fixture has one frame.  Convert it twice to test deterministic repeated static observations.
        convert(observations[0], root / "input_one")
        convert(observations[0], root / "input_two")
        first = root / "input_one" / "packet.json"
        second = root / "input_two" / "packet.json"
        second_data = json.loads(second.read_text(encoding="utf-8"))
        second_data["输出序号"], second_data["场景版本"], second_data["包标识"] = "2", "2", "cluster-2"
        second = write_packet(root / "input_two_resequenced", second_data, root / "input_two" / "contours.bin")
        tracked_paths = write_packets([first, second], root / "tracked")
        tracked = [json.loads(path.read_text(encoding="utf-8")) for path in tracked_paths]
        run("first_frame_is_full_snapshot", lambda: check(tracked[0]["包类型"] == "FullSnapshot" and tracked[0]["依赖全量序号"] is None, "First packet is not a snapshot"))
        run("second_static_frame_is_delta", lambda: check(tracked[1]["包类型"] == "Delta" and tracked[1]["依赖全量序号"] == "1", "Second packet is not based on first snapshot"))
        run("static_cluster_keeps_camera_track_id", lambda: check(tracked[0]["簇变化"][0]["相机跟踪候选编号"] == "1" and
                                                                     tracked[1]["簇变化"][0]["相机跟踪候选编号"] == "1", "Static cluster switched ID"))
        run("static_cluster_becomes_active", lambda: check(tracked[1]["簇变化"][0]["跟踪状态"] == "Active" and
                                                               tracked[1]["簇变化"][0]["变化类型"] == "Updated", "Static tracking state is wrong"))
        run("full_plus_delta_reconstructs_active_state", lambda: check(list(reconstruct(tracked)) == ["1"], "Reconstruction lost active track"))

        source = json.loads(first.read_text(encoding="utf-8"))
        blank_one, blank_two = copy.deepcopy(source), copy.deepcopy(source)
        for index, value in enumerate((blank_one, blank_two), start=2):
            value["输出序号"], value["场景版本"], value["包标识"], value["簇变化"] = str(index), str(index), f"cluster-{index}", []
        blank_one_path = write_packet(root / "blank_one", blank_one, first.parent / "contours.bin")
        blank_two_path = write_packet(root / "blank_two", blank_two, first.parent / "contours.bin")
        tracker = ClusterTracker(max_missing_frames=2)
        full = tracker.update(source)
        occluded = tracker.update(json.loads(blank_one_path.read_text(encoding="utf-8")))
        lost = tracker.update(json.loads(blank_two_path.read_text(encoding="utf-8")))
        for index, value in enumerate((full, occluded, lost), start=1):
            write_packet(root / f"absence_{index}", value, first.parent / "contours.bin")
        run("first_missing_frame_is_explicit_occlusion", lambda: check(occluded["簇变化"][0]["变化类型"] == "Occluded" and
                                                                          occluded["簇变化"][0]["帧内簇编号"] is None, "Missing track was silent"))
        run("second_missing_frame_is_explicit_lost_tombstone", lambda: check(lost["簇变化"][0]["变化类型"] == "Lost" and
                                                                                lost["簇变化"][0]["跟踪状态"] == "Retired", "Lost tombstone is wrong"))
        run("lost_tombstone_removes_reconstructed_track", lambda: check(not reconstruct([full, occluded, lost]), "Lost track remained reconstructed"))
        report["status"] = "pass"
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
