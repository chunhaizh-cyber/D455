"""P3 shadow-control tests for scan, observe, track, resync, and bounded materials."""

from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import json
from pathlib import Path
import time

from cluster_control import ClusterControlSession
from cluster_protocol import validate_cluster_packet
from cluster_tracker import ClusterTracker
from test_cluster_protocol import package


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def expect_error(fn, expected: str) -> None:
    try:
        fn()
    except ValueError as error:
        check(str(error).startswith(expected), f"Expected {expected}, got {error}")
        return
    raise AssertionError(f"Expected error: {expected}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    report = {"status": "running", "tests": []}

    def run(name, fn):
        started = time.perf_counter()
        detail = fn()
        report["tests"].append({"name": name, "status": "pass",
                                "seconds": round(time.perf_counter() - started, 4), "detail": detail})
        print(f"PASS {name}", flush=True)

    try:
        source_path = package(root / "source")
        source = json.loads(source_path.read_text(encoding="utf-8"))
        descriptor = source["材料"]["精确轮廓链"]
        contours = (source_path.parent / descriptor["文件"]).read_bytes()

        control = ClusterControlSession(root / "control", ClusterTracker(confirmation_frames=1),
                                        maximum_results_per_task=32)
        scan = control.command("开始扫描", {"区域": "全画面"})["任务编号"]
        first_path = control.process_source(source, contours)
        check(validate_cluster_packet(first_path)["status"] == "pass", "First scheduled packet is invalid")
        track = control.command("开始跟踪簇候选", {"相机跟踪候选编号": "1", "最长帧数": 5,
                                                    "自我绑定令牌": "self-track-1"})["任务编号"]
        observe = control.command("观察簇候选", {"相机跟踪候选编号": "1", "截止帧数": 3})["任务编号"]
        run("active_tracking_prevents_scan_shutdown", lambda: expect_error(
            lambda: control.command("停止扫描", {"任务编号": scan}), "scan_required_by_active_tasks"))

        for sequence in range(2, 7):
            frame = copy.deepcopy(source)
            frame["输出序号"] = frame["场景版本"] = str(sequence)
            frame["包标识"] = f"source-{sequence}"
            control.process_source(frame, contours)

        scan_results = control.command("读取簇级结果", {"任务编号": scan, "最多包数": 16})["结果"]
        track_results = control.command("读取簇级结果", {"任务编号": track, "最多包数": 16})["结果"]
        observe_results = control.command("读取簇级结果", {"任务编号": observe, "最多包数": 8})["结果"]
        run("mixed_load_preserves_every_full_frame_scan", lambda: check(
            len(scan_results) == 6 and all(item["全画面扫描保留"] for item in scan_results),
            "Observe or track load starved the full-frame scan"))
        run("bounded_track_task_receives_requested_frames", lambda: check(
            len(track_results) == 5 and all(item["相机跟踪候选编号"] == "1" and
                                             item["自我绑定令牌"] == "self-track-1"
                                             for item in track_results),
            "Track task did not receive its bounded frame window"))
        run("observe_task_returns_one_candidate_and_material_handle", lambda: check(
            len(observe_results) == 1 and "详细材料" in observe_results[0] and
            observe_results[0]["簇记录"]["相机跟踪候选编号"] == "1",
            "Observe task did not return the selected candidate"))

        handle = observe_results[0]["详细材料"]
        material_result = control.command("读取详细材料", {"句柄": handle["句柄"],
                                                          "预期SHA256": handle["SHA256"]})
        decoded = base64.b64decode(material_result["数据Base64"])
        run("detailed_material_range_read_preserves_hash", lambda: check(
            len(decoded) == handle["字节数"] and hashlib.sha256(decoded).hexdigest() == handle["SHA256"],
            "Detailed material read changed bytes"))
        control.command("释放详细材料", {"句柄": handle["句柄"]})
        run("released_material_is_not_readable", lambda: expect_error(
            lambda: control.command("读取详细材料", {"句柄": handle["句柄"],
                                                    "预期SHA256": handle["SHA256"]}), "unknown_material"))

        run("full_snapshot_request_checks_session", lambda: expect_error(
            lambda: control.command("请求全量快照", {"预期会话": "wrong", "预期跟踪时期": "1"}),
            "session_mismatch"))
        control.command("请求全量快照", {"预期会话": source["会话标识"], "预期跟踪时期": "1"})
        frame_seven = copy.deepcopy(source)
        frame_seven["输出序号"] = frame_seven["场景版本"] = "7"
        frame_seven["包标识"] = "source-7"
        resync_path = control.process_source(frame_seven, contours)
        resync_packet = json.loads(resync_path.read_text(encoding="utf-8"))
        run("control_request_publishes_next_packet_as_full_snapshot", lambda: check(
            resync_packet["包类型"] == "FullSnapshot" and resync_packet["指标"]["重同步全量"] is True,
            "Requested full snapshot was not published"))

        control.command("停止跟踪簇候选", {"任务编号": track})
        control.command("停止扫描", {"任务编号": scan})
        manifest = json.loads(control.write_manifest().read_text(encoding="utf-8"))
        run("released_material_leaves_zero_material_budget", lambda: check(
            manifest["未释放材料数"] == 0 and manifest["未释放材料字节数"] == 0,
            "Released material still consumes the shadow budget"))

        bounded = ClusterControlSession(root / "bounded", ClusterTracker(confirmation_frames=1),
                                        maximum_results_per_task=2)
        bounded.command("开始扫描")
        bounded.process_source(source, contours)
        second = copy.deepcopy(source)
        second["输出序号"] = second["场景版本"] = "2"
        second["包标识"] = "bounded-2"
        bounded.process_source(second, contours)
        third = copy.deepcopy(source)
        third["输出序号"] = third["场景版本"] = "3"
        third["包标识"] = "bounded-3"
        run("full_result_queue_applies_backpressure_before_state_advance", lambda: (
            expect_error(lambda: bounded.process_source(third, contours), "result_queue_backpressure"),
            check(bounded.packet_count == 2, "Backpressure advanced the published packet sequence")))

        aged = ClusterControlSession(root / "aged", ClusterTracker(confirmation_frames=1),
                                     maximum_results_per_task=16, maximum_result_age_frames=2)
        aged.command("开始扫描")
        for sequence in range(1, 4):
            frame = copy.deepcopy(source)
            frame["输出序号"] = frame["场景版本"] = str(sequence)
            frame["包标识"] = f"aged-{sequence}"
            aged.process_source(frame, contours)
        aged_four = copy.deepcopy(source)
        aged_four["输出序号"] = aged_four["场景版本"] = "4"
        aged_four["包标识"] = "aged-4"
        run("stale_result_queue_applies_age_backpressure", lambda: (
            expect_error(lambda: aged.process_source(aged_four, contours), "result_queue_backpressure:1:age"),
            check(aged.packet_count == 3, "Age backpressure advanced tracker state")))

        byte_limited = ClusterControlSession(root / "byte_limited", ClusterTracker(confirmation_frames=1),
                                            maximum_results_per_task=64, maximum_result_bytes_per_task=1024)
        byte_limited.command("开始扫描")
        byte_failure_at = None
        for sequence in range(1, 20):
            frame = copy.deepcopy(source)
            frame["输出序号"] = frame["场景版本"] = str(sequence)
            frame["包标识"] = f"byte-{sequence}"
            try:
                byte_limited.process_source(frame, contours)
            except ValueError as error:
                check(str(error).endswith(":bytes"), f"Unexpected byte-budget error: {error}")
                byte_failure_at = sequence
                break
        run("result_byte_budget_is_bounded_and_transactional", lambda: check(
            byte_failure_at is not None and byte_limited.packet_count == byte_failure_at - 1,
            "Result byte budget did not stop before advancing state"))
        report["status"] = "pass"
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
