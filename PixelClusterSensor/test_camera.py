"""Bounded, opt-in real camera smoke. Raw images stay under the unique output directory."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from client import Client
from validate_packet import check, read_material, validate_packet


def main():
    project = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, default=project / "x64/Release/PixelClusterSensor.exe")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--serial", default="")
    parser.add_argument("--continuous-frames", type=int, default=12)
    args = parser.parse_args()
    if not 2 <= args.continuous_frames <= 20:
        parser.error("--continuous-frames must be 2..20")
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    report = {"status": "running", "scope": "real RGBD transport, lifecycle and packet integrity",
              "executable_sha256": hashlib.sha256(args.exe.read_bytes()).hexdigest(),
              "started_unix_ms": int(time.time() * 1000), "tests": [], "frames": [],
              "physical_segmentation_accuracy": "not_established", "self_integration": "not_tested"}
    references = []
    timings = []
    opened = []

    def save():
        (root / "camera_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    def passed(name, detail):
        report["tests"].append({"name": name, "status": "pass", "detail": detail})
        save()
        print(f"PASS {name}", flush=True)

    def open_camera(client):
        result = client.call("打开设备", {"来源": "实时相机", "设备序列号": args.serial})
        check(result["会话标识"] not in opened, "Session reused across opens")
        opened.append(result["会话标识"])
        report.setdefault("opens", []).append(result)
        return result

    def single(client):
        start = time.perf_counter()
        ref = client.call("获取单帧观察")
        timings.append((time.perf_counter() - start) * 1000)
        references.append(ref)
        return ref

    def drain(client, task, *, cancel_after_first=False):
        deadline = time.monotonic() + 60
        received = []
        cancelled = False
        while time.monotonic() < deadline:
            result = client.call("读取观察结果", {"任务编号": task, "最多帧数": 8})
            received.extend(result["观察结果"])
            references.extend(result["观察结果"])
            state = result["任务"]
            if cancel_after_first and received and not cancelled and state["状态"] == "运行中":
                client.call("停止连续观察", {"任务编号": task})
                cancelled = True
            if state["状态"] != "运行中" and state["待取结果数"] == 0:
                check(len(received) == state["完成帧数"], "Task references missing or duplicated")
                return state
            time.sleep(0.01)
        raise TimeoutError("Continuous camera task did not terminate")

    save()
    try:
        with Client(args.exe, root / "packets", max_packets=128, max_bytes=512 * 1024 * 1024) as client:
            report["capabilities"] = client.call("查询设备能力")
            check(report["capabilities"]["设备"], "No camera connected")
            opened_info = open_camera(client)
            check(client.call("读取配置与标定")["实际配置与标定"] == opened_info["实际配置与标定"], "Calibration readback differs")
            for _ in range(3):
                single(client)
            passed("open_calibration_three_single_frames", {"count": 3})

            started = client.call("开始连续观察", {"帧数": args.continuous_frames, "模式": "实时优先", "最长秒数": 30})
            start = time.perf_counter()
            state = drain(client, started["任务编号"])
            latest_refs = references[-args.continuous_frames:]
            check(state["状态"] == "完成" and state["完成帧数"] == args.continuous_frames, f"Latest-mode failed: {state}")
            passed("latest_continuous", {"task": state, "wall_ms": (time.perf_counter() - start) * 1000})

            # Deliberately sample slower than 30 Hz: full mode must report the real source gap.
            task = client.call("开始连续观察", {"帧数": 3, "模式": "完整处理", "间隔毫秒": 150})["任务编号"]
            state = drain(client, task)
            check(state["状态"] == "失败" and state["错误"]["代码"] == "source_gap", f"Full-mode gap not rejected: {state}")
            passed("full_mode_rejects_deliberate_source_gap", state)

            task = client.call("开始连续观察", {"帧数": 60, "模式": "实时优先", "间隔毫秒": 100})["任务编号"]
            state = drain(client, task, cancel_after_first=True)
            check(state["状态"] == "取消" and 0 < state["完成帧数"] < 60, f"Cancel failed: {state}")
            passed("cancel_and_drain", state)
            client.call("关闭设备")
            check(client.call("查询运行状态")["设备打开"] is False, "Device remained open")
            for _ in range(3):
                open_camera(client)
                single(client)
                client.call("关闭设备")
            passed("same_process_three_reopens", {"distinct_sessions": len(opened)})
            open_camera(client)
            single(client)
            # Context closes stdin without sending close-device, exercising owner teardown.
        check(client.process.returncode == 0, "Sensor did not exit cleanly on EOF")
        with Client(args.exe, root / "after_eof") as client:
            open_camera(client)
            single(client)
            client.call("关闭设备")
        check(client.process.returncode == 0, "Reopened process did not exit cleanly")
        passed("eof_releases_device_for_next_process", {"distinct_sessions": len(opened)})

        # No readback work runs during capture; it would change the sampling workload.
        seen = set()
        last_source = {}
        for index, ref in enumerate(references):
            path = Path(ref["材料路径"])
            check(path not in seen, "Duplicate packet reference")
            seen.add(path)
            validation = validate_packet(path, reference=ref, output=root / "review" if index == 1 else None)
            m = json.loads(path.read_text(encoding="utf-8"))
            s = m["源信息"]
            check(s["类型"] == "实时相机", "Real camera mislabeled")
            check(s["彩图时间域"] == s["深度时间域"], "Incompatible time domains")
            check(s["预热状态"] == "未证明稳定", "Pair acceptance was confused with warmup stability")
            check(s["绝对采集时间已校准"] is False, "Unproven timestamp calibration claim")
            delta = abs(s["彩图时间戳毫秒"] - s["深度时间戳毫秒"])
            check(delta <= 50, "Unpaired streams published")
            previous = last_source.get(m["会话标识"])
            numbers = (int(s["彩图源帧号"]), int(s["深度源帧号"]))
            if previous:
                check(all(a > b for a, b in zip(numbers, previous)), "Stale source published")
            last_source[m["会话标识"]] = numbers
            arrays = {key: read_material(path.parent, m["材料"][key]) for key in
                      ("原始深度", "当前深度状态", "簇归属图", "归属状态")}
            z = arrays["原始深度"]
            row = {"reference": ref, "source": s, "validation": validation, "first_in_session": previous is None,
                   "stream_delta_ms": delta,
                   "raw_invalid_pixels": int(np.count_nonzero((z == 0) | (z == 65535))),
                   "aligned_missing_pixels": int(np.count_nonzero(arrays["当前深度状态"] == 0)),
                   "unassigned_pixels": int(np.count_nonzero(arrays["簇归属图"] == 0)),
                   "image_only_pixels": int(np.count_nonzero(arrays["归属状态"] == 2))}
            report["frames"].append(row)
            save()
        passed("independent_readback_all_packets", {"count": len(references)})
        process_ms = [r["reference"]["指标"]["处理毫秒"] for r in report["frames"]]
        def distribution(values):
            return dict(zip(("min", "p50", "p95", "max"), map(float, np.percentile(values, [0, 50, 95, 100]))))
        report["summary"] = {"packets": len(references), "sessions": len(opened),
            "processing_only_ms": distribution(process_ms), "single_roundtrip_ms": distribution(timings),
            "stream_delta_ms": distribution([r["stream_delta_ms"] for r in report["frames"]]),
            "cluster_count": distribution([r["validation"]["clusters"] for r in report["frames"]]),
            "raw_invalid_percent": distribution([100 * r["raw_invalid_pixels"] / r["validation"]["pixels"] for r in report["frames"]]),
            "aligned_missing_percent": distribution([100 * r["aligned_missing_pixels"] / r["validation"]["pixels"] for r in report["frames"]]),
            "quality_promotion": "not_evaluated; this is not the fixed replay score gate"}
        for name, rows in (("first_in_session", [r for r in report["frames"] if r["first_in_session"]]),
                           ("subsequent_in_session", [r for r in report["frames"] if not r["first_in_session"]]),
                           ("latest_continuous", [r for r in report["frames"] if r["reference"] in latest_refs])):
            report["summary"][name] = {"packets": len(rows),
                "processing_only_ms": distribution([r["reference"]["指标"]["处理毫秒"] for r in rows]),
                "raw_invalid_percent": distribution([100 * r["raw_invalid_pixels"] / r["validation"]["pixels"] for r in rows]),
                "color_source_gaps": sum(r["source"]["彩图源帧缺口"] for r in rows),
                "depth_source_gaps": sum(r["source"]["深度源帧缺口"] for r in rows)}
        report["status"] = "pass"
        print(json.dumps(report["summary"], ensure_ascii=False, indent=2))
    except Exception as exc:
        report["status"] = "fail"
        report["error"] = str(exc)
        raise
    finally:
        report["references"] = references
        report["finished_unix_ms"] = int(time.time() * 1000)
        save()


if __name__ == "__main__":
    main()
