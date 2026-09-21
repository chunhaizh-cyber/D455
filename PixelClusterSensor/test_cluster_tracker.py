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
from cluster_protocol import pack_ring
from cluster_tracker import ClusterTracker, reconstruct, write_packets
from convert_cluster_observation import convert
from test_cluster_conversion import make_replay
from test_cluster_protocol import digest, package


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


def with_cluster_pixels(packet: dict, pixels: int, sequence: int) -> dict:
    result = copy.deepcopy(packet)
    result["输出序号"] = result["场景版本"] = str(sequence)
    result["包标识"] = f"cluster-{sequence}"
    entry = result["簇变化"][0]
    entry["像素数"] = pixels
    entry["深度证据"] = {
        "当前实测像素数": 0, "历史候选像素数": 0, "估算像素数": 0,
        "缺失像素数": pixels, "范围外像素数": 0,
    }
    entry["颜色摘要"]["有效像素数"] = pixels
    return result


def moved_snapshot(root: Path) -> tuple[Path, Path]:
    base = package(root / "base")
    shifted = json.loads(base.read_text(encoding="utf-8"))
    shifted["输出序号"], shifted["场景版本"], shifted["包标识"] = "2", "2", "packet-2"
    entry = shifted["簇变化"][0]
    entry["范围XYWH"], entry["图像中心XY"] = [3, 2, 3, 3], [4.0, 3.0]
    ring, raw, bits = pack_ring([(3, 2), (4, 2), (5, 2), (5, 3), (5, 4), (4, 4), (3, 4), (3, 3)], inner=False)
    entry["轮廓"][0].update({"字节偏移": 0, "有效位数": bits, "起点XY": list(ring[0]), "点数": len(ring)})
    shifted["材料"]["精确轮廓链"].update({"字节数": len(raw), "SHA256": digest(raw)})
    directory = root / "shifted"
    directory.mkdir()
    (directory / "contours.bin").write_bytes(raw)
    path = directory / "packet.json"
    path.write_text(json.dumps(shifted, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    validate_cluster_packet(path)
    return base, path


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
        run("second_static_frame_is_heartbeat", lambda: check(tracked[1]["包类型"] == "Heartbeat" and
                                                                 tracked[1]["依赖全量序号"] == "1" and not tracked[1]["簇变化"],
                                                           "Unchanged static frame is not an empty heartbeat"))
        run("static_cluster_keeps_camera_track_id", lambda: check(tracked[0]["簇变化"][0]["相机跟踪候选编号"] == "1", "Static cluster did not receive a stable ID"))
        run("full_plus_delta_reconstructs_active_state", lambda: check(list(reconstruct(tracked)) == ["1"], "Reconstruction lost active track"))

        source = json.loads(first.read_text(encoding="utf-8"))
        blank_one, blank_two = copy.deepcopy(source), copy.deepcopy(source)
        for index, value in enumerate((blank_one, blank_two), start=2):
            value["输出序号"], value["场景版本"], value["包标识"], value["簇变化"] = str(index), str(index), f"cluster-{index}", []
        blank_one_path = write_packet(root / "blank_one", blank_one, first.parent / "contours.bin")
        blank_two_path = write_packet(root / "blank_two", blank_two, first.parent / "contours.bin")
        tracker = ClusterTracker(max_missing_frames=2, confirmation_frames=1)
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

        reappearance_source = copy.deepcopy(source)
        reappearance_source["输出序号"], reappearance_source["场景版本"], reappearance_source["包标识"] = "3", "3", "cluster-reappeared"
        reappearance_path = write_packet(root / "reappearance_source", reappearance_source, first.parent / "contours.bin")
        tracker = ClusterTracker(max_missing_frames=2, confirmation_frames=1)
        first_visible = tracker.update(source)
        one_missing = tracker.update(json.loads(blank_one_path.read_text(encoding="utf-8")))
        reappeared = tracker.update(json.loads(reappearance_path.read_text(encoding="utf-8")))
        run("reappearance_is_delta_not_heartbeat", lambda: check(reappeared["包类型"] == "Delta" and
                                                                    reappeared["簇变化"][0]["变化类型"] == "Updated" and
                                                                    reappeared["簇变化"][0]["跟踪状态"] == "Reappeared",
                                                              "Reappeared track was hidden by a heartbeat"))
        run("reappearance_keeps_prior_track_id", lambda: check(first_visible["簇变化"][0]["相机跟踪候选编号"] ==
                                                                  reappeared["簇变化"][0]["相机跟踪候选编号"] and
                                                                  one_missing["簇变化"][0]["变化类型"] == "Occluded",
                                                            "Reappearance changed or skipped the candidate ID"))
        run("reappearance_resets_consecutive_visible_count", lambda: check(reappeared["簇变化"][0]["时效"]["连续可见帧数"] == 1,
                                                                            "Reappearance retained a stale visible streak"))

        tentative_tracker = ClusterTracker(max_missing_frames=2, confirmation_frames=3)
        tentative_added = tentative_tracker.update(source)
        tentative_removed = tentative_tracker.update(json.loads(blank_one_path.read_text(encoding="utf-8")))
        write_packet(root / "tentative_added", tentative_added, first.parent / "contours.bin")
        write_packet(root / "tentative_removed", tentative_removed, first.parent / "contours.bin")
        run("unconfirmed_candidate_is_removed_not_occluded", lambda: check(
            tentative_added["簇变化"][0]["跟踪状态"] == "Tentative" and
            tentative_removed["簇变化"][0]["变化类型"] == "Removed" and
            tentative_removed["簇变化"][0]["跟踪状态"] == "Retired" and
            tentative_removed["簇变化"][0]["遮挡"]["状态"] == "Unknown" and
            tentative_removed["簇变化"][0]["遮挡"]["依据"] == "tentative_candidate_not_reobserved",
            "Unconfirmed candidate entered the occlusion state machine"))
        run("tentative_removal_reconstructs_empty_state", lambda: check(
            not reconstruct([tentative_added, tentative_removed]), "Removed tentative candidate remained reconstructed"))

        hysteresis_tracker = ClusterTracker(max_missing_frames=2, confirmation_frames=1,
                                            minimum_new_cluster_pixels=10,
                                            minimum_retained_cluster_pixels=5)
        accepted = hysteresis_tracker.update(with_cluster_pixels(source, 12, 1))
        retained = hysteresis_tracker.update(with_cluster_pixels(source, 8, 2))
        below_retained = hysteresis_tracker.update(with_cluster_pixels(source, 4, 3))
        filtered_new = ClusterTracker(minimum_new_cluster_pixels=10, minimum_retained_cluster_pixels=5).update(
            with_cluster_pixels(source, 9, 1))
        run("new_candidate_below_admission_threshold_is_unknown", lambda: check(
            filtered_new["簇变化"] == [] and
            filtered_new["指标"]["本帧过滤新候选数"] == 1 and
            filtered_new["指标"]["本帧过滤新候选像素数"] == 9 and
            filtered_new["全局覆盖摘要"]["已处理像素数"] == source["全局覆盖摘要"]["已处理像素数"] - 9 and
            filtered_new["全局覆盖摘要"]["未知像素数"] == source["全局覆盖摘要"]["未知像素数"] + 9,
            "Sub-threshold new candidate was not reassigned to Unknown"))
        run("existing_candidate_is_retained_inside_hysteresis_band", lambda: check(
            accepted["簇变化"][0]["相机跟踪候选编号"] == retained["簇变化"][0]["相机跟踪候选编号"] == "1" and
            retained["簇变化"][0]["像素数"] == 8 and retained["指标"]["本帧过滤新候选数"] == 0,
            "Existing candidate was not retained below the new-candidate threshold"))
        tentative_hysteresis = ClusterTracker(max_missing_frames=2, confirmation_frames=3,
                                              minimum_new_cluster_pixels=10,
                                              minimum_retained_cluster_pixels=5)
        tentative_first = tentative_hysteresis.update(with_cluster_pixels(source, 12, 1))
        tentative_small = tentative_hysteresis.update(with_cluster_pixels(source, 8, 2))
        run("tentative_candidate_cannot_use_retention_threshold", lambda: check(
            tentative_first["簇变化"][0]["跟踪状态"] == "Tentative" and
            tentative_small["指标"]["本帧过滤新候选数"] == 1 and
            any(entry["变化类型"] == "Removed" and entry["相机跟踪候选编号"] == "1"
                for entry in tentative_small["簇变化"]),
            "Tentative candidate incorrectly used the confirmed-track retention threshold"))
        run("candidate_below_retention_threshold_enters_missing_path", lambda: check(
            below_retained["指标"]["本帧过滤新候选数"] == 1 and
            below_retained["指标"]["本帧过滤新候选像素数"] == 4 and
            below_retained["簇变化"][0]["变化类型"] == "Occluded" and
            below_retained["全局覆盖摘要"]["已处理像素数"] +
            below_retained["全局覆盖摘要"]["未知像素数"] +
            below_retained["全局覆盖摘要"]["无效像素数"] +
            below_retained["全局覆盖摘要"]["遮挡像素数"] +
            below_retained["全局覆盖摘要"]["未处理像素数"] ==
            below_retained["图像尺寸WH"][0] * below_retained["图像尺寸WH"][1],
            "Below-retention candidate did not follow missing-state semantics or preserve coverage"))

        move_base, move_shifted = moved_snapshot(root / "movement")
        tentative_move_paths = write_packets([move_base, move_shifted], root / "tentative_movement_tracked")
        tentative_moved = [json.loads(path.read_text(encoding="utf-8")) for path in tentative_move_paths]
        run("high_cost_tentative_match_cannot_accumulate_confirmation", lambda: check(
            tentative_moved[0]["簇变化"][0]["相机跟踪候选编号"] == "1" and
            any(entry["变化类型"] == "Removed" and entry["相机跟踪候选编号"] == "1"
                for entry in tentative_moved[1]["簇变化"]) and
            any(entry["变化类型"] == "Added" and entry["相机跟踪候选编号"] == "2"
                for entry in tentative_moved[1]["簇变化"]),
            "High-cost tentative association retained a drifting candidate"))
        moved_paths = write_packets([move_base, move_shifted], root / "movement_tracked", confirmation_frames=1)
        moved = [json.loads(path.read_text(encoding="utf-8")) for path in moved_paths]
        run("small_position_shift_keeps_track_id", lambda: check(moved[0]["簇变化"][0]["相机跟踪候选编号"] == "1" and
                                                                  moved[1]["簇变化"][0]["相机跟踪候选编号"] == "1", "Small movement switched ID"))
        run("small_position_shift_is_moved_event", lambda: check(moved[1]["簇变化"][0]["变化类型"] == "Moved", "Movement was not explicit"))
        report["status"] = "pass"
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
