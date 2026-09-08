"""Bounded synthetic verification. Fixtures are generated under the selected output directory."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import shutil
import time

import numpy as np
from PIL import Image

from client import Client
from validate_packet import check, read_material, validate_packet


def calibration(width=32, height=24):
    return {"宽": width, "高": height, "焦距X": 100.0, "焦距Y": 100.0,
            "主点X": (width - 1) / 2, "主点Y": (height - 1) / 2, "畸变模型": 0, "畸变系数": [0.0] * 5}


def fixture(root, name, color, depth, frames=2, modify=None):
    directory = root / "fixtures" / name
    directory.mkdir(parents=True)
    Image.fromarray(color).save(directory / "color.png")
    Image.fromarray(depth).save(directory / "depth.png")
    ci = calibration(color.shape[1], color.shape[0])
    di = calibration(depth.shape[1], depth.shape[0])
    data = {"格式": "PCS.RawSequence/1", "材料来源": "合成夹具", "设备标识": "synthetic-test-only",
            "深度单位米": 0.001, "彩图内参": ci, "深度内参": di,
            "深度到彩图外参": {"旋转列优先": [1, 0, 0, 0, 1, 0, 0, 0, 1], "平移米": [0, 0, 0]},
            "帧列表": [{"彩图": "color.png", "深度": "depth.png", "源帧号": str(i + 1),
                       "彩图时间戳毫秒": i * 1000 / 30, "深度时间戳毫秒": i * 1000 / 30,
                       "时间域": "synthetic_clock"} for i in range(frames)]}
    if modify:
        modify(data)
    path = directory / "sequence.json"
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def open_replay(client, path):
    return client.call("打开设备", {"来源": "目录回放", "清单": str(path.resolve())})


def error(client, request, code):
    result = client.send(request)
    check(result["状态"] == "失败" and result["错误"]["代码"] == code, f"Expected {code}: {result}")
    return result


def arrays_from(path):
    manifest = json.loads(path.read_text(encoding="utf-8"))
    arrays = {key: read_material(path.parent, descriptor) for key, descriptor in manifest["材料"].items()}
    return manifest, arrays


def main():
    project = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, default=project / "x64/Release/PixelClusterSensor.exe")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    report = {"status": "running", "tests": [], "camera_validation": "not_run",
              "quality_promotion": "not_evaluated", "scope": "protocol, synthetic geometry and independent readback"}

    def run(name, fn):
        start = time.perf_counter()
        detail = fn()
        report["tests"].append({"name": name, "status": "pass", "seconds": round(time.perf_counter() - start, 4), "detail": detail})
        print(f"PASS {name}", flush=True)

    color = np.full((24, 32, 3), [160, 80, 40], dtype=np.uint8)
    depth = np.full((24, 32), 1000, dtype=np.uint16)
    saved = {}

    def geometry_case(name, rgb, z, assertion, modify=None):
        path = fixture(root, name, rgb, z, modify=modify)
        with Client(args.exe, root / name) as client:
            open_replay(client, path)
            first = client.call("获取单帧观察")
            first_path = Path(first["材料路径"])
            validation = validate_packet(first_path, reference=first, expected_color=rgb, expected_depth=z,
                                         output=root / "reviews" / name)
            manifest, arrays = arrays_from(first_path)
            assertion(manifest, arrays)
            second = client.call("获取单帧观察")
            second_manifest, _ = arrays_from(Path(second["材料路径"]))
            check(manifest["材料"] == second_manifest["材料"], "Repeated identical input produced different full materials")
            check(manifest["簇目录"] == second_manifest["簇目录"], "Repeated contour directory differs")
            client.call("关闭设备")
            saved[name] = first_path
            return validation

    try:
        run("constant_plane", lambda: geometry_case("plane", color, depth,
            lambda m, a: check(len(m["簇目录"]) == 1, "Constant plane was split")))
        patterned = color.copy()
        patterned[:, 16:] = [10, 240, 220]
        run("paint_does_not_split_depth_plane", lambda: geometry_case("paint", patterned, depth,
            lambda m, a: check(len(m["簇目录"]) == 1, "Paint was treated as a geometric boundary")))
        slope = depth + np.arange(32, dtype=np.uint16)[None, :] * 4
        run("continuous_slope", lambda: geometry_case("slope", color, slope,
            lambda m, a: check(len(m["簇目录"]) == 1, "Allowed slope was split")))
        step = depth.copy()
        step[:, 16:] = 2000
        run("same_color_depth_step", lambda: geometry_case("step", color, step,
            lambda m, a: check(len(m["簇目录"]) == 2 and a["簇归属图"][12, 14] != a["簇归属图"][12, 17], "Depth step merged")))
        hole = depth.copy()
        hole[12, 16] = 0
        run("isolated_missing_depth_preserved_and_filled", lambda: geometry_case("hole", color, hole,
            lambda m, a: check(a["当前深度状态"][12, 16] == 0 and a["补全状态"][12, 16] == 1 and
                              abs(float(a["补全深度米"][12, 16]) - 1) < 1e-6 and len(m["簇目录"]) == 1,
                              "Isolated hole evidence was lost or mislabeled")))
        weighted_color = np.full_like(color, 230)
        weighted_depth = np.full_like(depth, 1020)
        weighted_color[12, 16] = [200, 20, 20]
        weighted_depth[12, 16] = 0
        for y, x in [(12, 15), (12, 17), (11, 16)]:
            weighted_color[y, x] = [200, 20, 20]
            weighted_depth[y, x] = 1000
        run("missing_pixel_color_guides_neighbors", lambda: geometry_case("weighted", weighted_color, weighted_depth,
            lambda m, a: check(a["补全状态"][12, 16] == 1 and abs(float(a["补全深度米"][12, 16]) - 1) < 1e-6,
                              "Dissimilar-color neighbors affected interpolation")))
        ambiguous = step.copy()
        ambiguous[12, 15] = 0
        run("missing_depth_does_not_bridge_two_anchors", lambda: geometry_case("ambiguous", color, ambiguous,
            lambda m, a: check(a["补全状态"][12, 15] == 0 and a["簇归属图"][12, 14] != a["簇归属图"][12, 16], "Ambiguous gap filled or merged")))
        large = depth.copy()
        large[10:13, 14:17] = 0
        run("large_gap_no_recursive_fill", lambda: geometry_case("large", color, large,
            lambda m, a: check(np.count_nonzero(a["补全状态"]) == 0 and len(m["簇目录"]) == 1, "Large gap dropped or extrapolated")))
        ring_depth = np.full_like(depth, 2000)
        ring_depth[5:20, 5:26] = 1000
        ring_depth[10:15, 12:18] = 2000
        run("background_opening_retains_own_membership", lambda: geometry_case("ring", color, ring_depth,
            lambda m, a: check(a["簇归属图"][12, 14] != a["簇归属图"][7, 7] and
                              any(r["内环"] for c in m["簇目录"] for r in c["轮廓"]), "Background opening erased")))
        run("no_depth_still_has_color_ownership", lambda: geometry_case("all_missing", color, np.zeros_like(depth),
            lambda m, a: check(np.all(a["归属状态"] == 2) and np.count_nonzero(a["补全状态"]) == 0, "All-missing scene fabricated depth")))
        run("far_depth_not_promoted_to_precise", lambda: geometry_case("far", color, np.full_like(depth, 5000),
            lambda m, a: check(np.all(a["当前深度状态"] == 2) and np.all(a["归属状态"] == 2), "Far depth claimed usable")))
        saturation = depth.copy()
        saturation[12, 16] = 65535
        run("saturated_raw_code_is_not_measurement", lambda: geometry_case("saturation", color, saturation,
            lambda m, a: check(a["当前深度状态"][12, 16] == 0 and a["原始深度"][12, 16] == 65535, "Saturation misclassified or overwritten")))

        def shifted(m):
            m["深度到彩图外参"]["平移米"] = [0.01, 0, 0.2]
        run("extrinsics_transform_optical_z", lambda: geometry_case("extrinsics", color, depth,
            lambda m, a: check(np.allclose(a["当前观测深度米"][a["当前深度状态"] != 0], 1.2), "Source Z copied without transform"), shifted))
        def distorted(m):
            m["彩图内参"].update({"畸变模型": 2, "畸变系数": [-0.06, 0.07, -0.0002, 0.0005, -0.02]})
            m["深度内参"].update({"畸变模型": 4, "畸变系数": [0.02, -0.01, 0.0001, -0.0002, 0.001]})
        run("brown_depth_inverse_brown_color", lambda: geometry_case("distortion", color, depth,
            lambda m, a: check(m["标定"]["彩图内参"]["畸变模型"] == 2, "Distortion model lost"), distorted))

        def protocol():
            path = fixture(root, "protocol", color, hole, frames=10)
            with Client(args.exe, root / "protocol") as client:
                open_replay(client, path)
                request = client.request("获取单帧观察")
                first = client.send(request)
                check(first["状态"] == "完成", "Capture failed")
                check(client.send(request) == first, "Retry did not replay exact receipt")
                changed = copy.deepcopy(request)
                changed["指令"] = "关闭设备"
                error(client, changed, "request_conflict")
                check(client.send(request) == first, "Conflict poisoned original receipt")
                check(client.call("查询运行状态")["已发布包数"] == 1, "Duplicate request created new observations")
                error(client, client.request("获取单帧观察", 会话标识="old-session"), "stale_session")
                error(client, client.request("获取单帧观察", 截止Unix毫秒=1), "expired_request")
                error(client, client.request("设置处理配置", {"启用补全": False}, 预期配置版本="0"), "stale_config")
                error(client, client.request("设置处理配置", {"补全最少样本": 2.5}, 预期配置版本=client.revision), "invalid_config")
                error(client, client.request("设置处理配置", {"补全最少样本": 4294967299}, 预期配置版本=client.revision), "invalid_config")
                error(client, client.request("设置处理配置", {"未知参数": 1}, 预期配置版本=client.revision), "unknown_field")
                check(client.call("读取配置与标定")["配置版本"] == "1", "Rejected configuration changed state")
                client.call("设置处理配置", {"启用补全": False}, 预期配置版本=client.revision)
                second = client.call("获取单帧观察")
                _, arrays = arrays_from(Path(second["材料路径"]))
                check(np.count_nonzero(arrays["补全状态"]) == 0, "Disable fill not applied")
                check(np.array_equal(arrays["原始深度"], hole), "Config changed raw data")
                error(client, client.request("设置成像参数", {"曝光": 100}), "unsupported_command")
                duplicate = '{"协议":"PCS.Control/1","协议":"PCS.Control/1"}'
                check(client.send_raw(duplicate)["错误"]["代码"] == "invalid_json", "Duplicate JSON keys accepted")
                check(client.send_raw("x" * 65537)["错误"]["代码"] == "request_too_large", "Oversized request accepted")
                check(client.call("查询运行状态")["设备打开"], "Malformed request killed service")
                old = client.session
                client.call("关闭设备")
                open_replay(client, path)
                check(client.session != old, "Device reopen reused old session")
                error(client, client.request("获取单帧观察", 会话标识=old), "stale_session")
                client.call("关闭设备")
            return {"idempotence": "pass", "stale_guards": "pass", "config_readback": "pass", "malformed_input": "pass"}
        run("control_protocol", protocol)

        def invalid_sources():
            def expect_open_failure(name, modify, code):
                path = fixture(root, name, color, depth, modify=modify)
                with Client(args.exe, root / name) as client:
                    error(client, client.request("打开设备", {"来源": "目录回放", "清单": str(path)}), code)
                    check(not client.call("查询运行状态")["设备打开"], "Failed open retained session")
            expect_open_failure("missing_calibration", lambda m: m.pop("彩图内参"), "invalid_argument")
            expect_open_failure("bad_distortion", lambda m: m["彩图内参"].update({"畸变模型": 99}), "unsupported_calibration")
            expect_open_failure("bad_rotation", lambda m: m["深度到彩图外参"].update({"旋转列优先": [0] * 9}), "invalid_calibration")
            expect_open_failure("overflow_size", lambda m: m["彩图内参"].update({"宽": 4294967328}), "invalid_calibration")
            expect_open_failure("duplicate_frames", lambda m: m["帧列表"][1].update({"源帧号": "1"}), "invalid_source")
            expect_open_failure("false_live_source", lambda m: m.update({"材料来源": "实时相机"}), "invalid_source")
            for name, modify, code in [
                ("wrong_size", lambda m: m["彩图内参"].update({"宽": 16}), "calibration_mismatch"),
                ("unsynchronized", lambda m: m["帧列表"][0].update({"深度时间戳毫秒": 200}), "unsynchronized_frame"),
                ("path_escape", lambda m: m["帧列表"][0].update({"彩图": "../plane/color.png"}), "invalid_path"),
            ]:
                path = fixture(root, name, color, depth, modify=modify)
                with Client(args.exe, root / name) as client:
                    open_replay(client, path)
                    error(client, client.request("获取单帧观察"), code)
                    check(client.call("查询运行状态")["已发布包数"] == 0, "Invalid frame was published")
                    client.call("关闭设备")
            return {"invalid_cases": 9, "silent_calibration_fallback": False}
        run("source_rejection", invalid_sources)

        def streaming(latest):
            name = "latest_stream" if latest else "complete_stream"
            path = fixture(root, name, color, depth, frames=45)
            with Client(args.exe, root / name) as client:
                open_replay(client, path)
                task = client.call("开始连续观察", {"帧数": 40, "模式": "实时优先" if latest else "完整处理", "最长秒数": 20})["任务编号"]
                deadline = time.monotonic() + 20
                while time.monotonic() < deadline:
                    state = client.call("查询运行状态")["任务"]
                    if state["状态"] == "完成" or state["待取结果数"] == 32:
                        if latest and state["状态"] != "完成":
                            time.sleep(0.02)
                            continue
                        break
                    time.sleep(0.02)
                else:
                    raise TimeoutError("Stream did not reach bounded queue")
                if not latest:
                    check(state["完成帧数"] == 32 and state["丢弃未消费引用数"] == 0, "Full mode failed to apply backpressure")
                    time.sleep(0.1)
                    check(client.call("查询运行状态")["任务"]["完成帧数"] == 32, "Full mode consumed frames while blocked")
                else:
                    check(state["完成帧数"] == 40 and state["丢弃未消费引用数"] == 8, "Latest mode drop accounting incorrect")
                error(client, client.request("关闭设备"), "unread_results")
                references = []
                while time.monotonic() < deadline:
                    reply = client.call("读取观察结果", {"任务编号": task})
                    references.extend(reply["观察结果"])
                    if reply["任务"]["状态"] == "完成" and not reply["任务"]["待取结果数"]:
                        break
                    time.sleep(0.01)
                expected = list(range(9 if latest else 1, 41))
                check([int(r["源帧号"]) for r in references] == expected, "Stream order/gaps incorrect")
                client.call("关闭设备")
            return {"published": 40, "delivered": len(references), "dropped_references": 8 if latest else 0}
        run("full_stream_backpressure", lambda: streaming(False))
        run("latest_stream_gap_accounting", lambda: streaming(True))

        def cancellation_and_exhaustion():
            path = fixture(root, "cancel", color, depth, frames=40)
            with Client(args.exe, root / "cancel") as client:
                open_replay(client, path)
                task = client.call("开始连续观察", {"帧数": 40, "间隔毫秒": 100})["任务编号"]
                result = client.call("取消请求", {"任务编号": task})
                check(result["状态"] == "取消" and result["完成帧数"] < 40, "Cancellation failed")
                client.call("读取观察结果", {"任务编号": task})
                client.call("关闭设备")
            one = fixture(root, "exhaustion", color, depth, frames=1)
            with Client(args.exe, root / "exhaustion") as client:
                open_replay(client, one)
                task = client.call("开始连续观察", {"帧数": 2})["任务编号"]
                for _ in range(100):
                    state = client.call("查询运行状态")["任务"]
                    if state["状态"] == "失败":
                        break
                    time.sleep(0.01)
                check(state["状态"] == "失败" and state["完成帧数"] == 1 and state["错误"]["代码"] == "end_of_source", "EOF claimed complete task")
                check(len(client.call("读取观察结果", {"任务编号": task})["观察结果"]) == 1, "Completed partial result lost")
                client.call("关闭设备")
            with Client(args.exe, root / "eof") as client:
                open_replay(client, path)
                client.call("开始连续观察", {"帧数": 40, "间隔毫秒": 100})
                start = time.monotonic()
                client.close()
                check(client.process.returncode == 0 and time.monotonic() - start < 5, "EOF shutdown failed")
            return {"cancellation": "pass", "source_exhaustion": "honest_failure", "eof_shutdown": "pass"}
        run("cancel_exhaustion_and_shutdown", cancellation_and_exhaustion)

        def discontinuity_and_deadline():
            def gap(m):
                m["帧列表"][1]["源帧号"] = "3"
            path = fixture(root, "source_gap", color, depth, modify=gap)
            with Client(args.exe, root / "source_gap") as client:
                open_replay(client, path)
                task = client.call("开始连续观察", {"帧数": 2})["任务编号"]
                for _ in range(100):
                    state = client.call("查询运行状态")["任务"]
                    if state["状态"] == "失败":
                        break
                    time.sleep(0.01)
                check(state["状态"] == "失败" and state["错误"]["代码"] == "source_gap", "Source gap was hidden by full mode")
                client.call("读取观察结果", {"任务编号": task})
                client.call("关闭设备")
            sequence = fixture(root, "deadline", color, depth, frames=3)
            with Client(args.exe, root / "deadline") as client:
                open_replay(client, sequence)
                task = client.call("开始连续观察", {"帧数": 3, "最长秒数": 1, "间隔毫秒": 1000})["任务编号"]
                time.sleep(1.2)
                state = client.call("查询运行状态")["任务"]
                check(state["状态"] == "未完成" and state["错误"]["代码"] == "task_deadline", "Deadline reported success")
                client.call("读取观察结果", {"任务编号": task})
                client.call("关闭设备")
            return {"source_gap": "rejected", "deadline": "incomplete"}
        run("source_continuity_and_task_deadline", discontinuity_and_deadline)

        def quota_and_tamper():
            path = fixture(root, "quota", color, depth, frames=3)
            with Client(args.exe, root / "quota", max_packets=1) as client:
                open_replay(client, path)
                client.call("获取单帧观察")
                error(client, client.request("获取单帧观察"), "storage_quota")
                check(client.call("查询运行状态")["已发布包数"] == 1, "Quota failure published extra packet")
                client.call("关闭设备")
            copied = root / "tampered"
            shutil.copytree(saved["plane"].parent, copied)
            target = copied / "labels.bin"
            damaged = bytearray(target.read_bytes())
            damaged[0] ^= 1
            target.write_bytes(damaged)
            try:
                validate_packet(copied / "frame.json")
            except ValueError as exc:
                check("SHA256" in str(exc), "Unexpected tamper failure")
            else:
                raise AssertionError("Modified payload was accepted")
            return {"quota": "pass", "corruption_rejected": True}
        run("storage_quota_and_checksum", quota_and_tamper)
        report["status"] = "pass"
    except Exception as exc:
        report["status"] = "fail"
        report["error"] = repr(exc)
        raise
    finally:
        (root / "test_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"status": report["status"], "tests": len(report["tests"]), "report": str(root / "test_report.json")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
