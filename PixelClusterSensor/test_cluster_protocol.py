"""Synthetic contract tests for PCS.ClusterObservation/1."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import time

from cluster_protocol import pack_ring, unpack_ring, validate_cluster_packet


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def digest(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def package(root: Path, packet_type: str = "FullSnapshot") -> Path:
    root.mkdir(parents=True)
    ring, raw, bits = pack_ring([(2, 2), (3, 2), (4, 2), (4, 3), (4, 4), (3, 4), (2, 4), (2, 3)], inner=False)
    (root / "contours.bin").write_bytes(raw)
    levels = []
    for side in (8, 16, 32):
        bitmap = bytes([side]) * ((side * side + 7) // 8)
        levels.append({"边长": side, "位图SHA256": digest(bitmap), "前景像素数": 1})
    entry = {
        "帧内簇编号": 1,
        "相机跟踪候选编号": None,
        "自我绑定令牌": None,
        "变化类型": "Added",
        "跟踪状态": "Tentative",
        "范围XYWH": [2, 2, 3, 3],
        "图像中心XY": [3.0, 3.0],
        "像素数": 9,
        "触及视野边界": False,
        "轮廓": [{
            "材料键": "精确轮廓链", "字节偏移": 0, "有效位数": bits, "起点XY": list(ring[0]), "点数": len(ring),
            "父环索引": -1, "内环": False, "闭合区域边界": True, "物理孔洞确认": False, "边界原因位": 1,
        }],
        "形状指纹": {"归一化规则": "filled-contour-center-square/1", "层级": levels},
        "颜色摘要": {"颜色空间": "sRGB", "RGB均值": [10, 20, 30], "有效像素数": 9, "可比性": "当前帧"},
        "距离": {"模式": "UnknownDistance", "值米": None, "区间米": None, "不确定度米": None, "依据": "证据不足"},
        "三维中心米": None,
        "尺寸米": None,
        "深度证据": {"当前实测像素数": 0, "历史候选像素数": 0, "估算像素数": 0, "缺失像素数": 9, "范围外像素数": 0},
        "运动": None,
        "遮挡": {"状态": "Unknown", "比例": None, "依据": "单帧无时序证据"},
        "关联证据": None,
        "时效": {"连续可见帧数": 1, "连续缺失帧数": 0, "证据年龄毫秒": None},
        "详细材料句柄": None,
    }
    packet = {
        "格式": "PCS.ClusterObservation/1", "发布状态": "完整", "包标识": "packet-1", "会话标识": "session-1",
        "跟踪时期": "1", "输出序号": "1", "场景版本": "1", "包类型": packet_type, "任务意图": "Scan",
        "依赖全量序号": None if packet_type == "FullSnapshot" else "1",
        "源时间": {"值": 1.0, "单位": "ms", "时间域": "synthetic"}, "发布Unix毫秒": "9999999999999", "结果年龄毫秒": None,
        "配置版本": "1", "标定版本": "sha256:synthetic", "坐标系": "Synthetic_Color_Optical", "图像尺寸WH": [8, 8],
        "相机姿态": None,
        "处理区域": {"高精度ROI": [], "外围扫描区": [[0, 0, 8, 8]], "未处理区": []},
        "输入质量": {"源缺口": False, "同步状态": "通过", "预热状态": "未证明稳定"},
        "全局覆盖摘要": {"已处理像素数": 64, "未知像素数": 0, "无效像素数": 0, "遮挡像素数": 0, "未处理像素数": 0},
        "材料": {"精确轮廓链": {"文件": "contours.bin", "编码": "PCS.ContourChain8/1", "字节数": len(raw), "SHA256": digest(raw)}},
        "簇变化": [] if packet_type == "Heartbeat" else [entry],
        "指标": {"scope": "synthetic"},
    }
    path = root / "packet.json"
    path.write_text(json.dumps(packet, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def mutate(path: Path, output: Path, fn) -> Path:
    shutil.copytree(path.parent, output)
    target = output / path.name
    data = json.loads(target.read_text(encoding="utf-8"))
    fn(data)
    target.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return target


def expect_invalid(path: Path, message: str) -> str:
    try:
        validate_cluster_packet(path)
    except ValueError as error:
        return str(error)
    raise AssertionError(message)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
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
        base = package(root / "valid")
        run("valid_full_snapshot", lambda: validate_cluster_packet(base))
        run("contour_codec_round_trip", lambda: check(unpack_ring([2, 2], 8, (base.parent / "contours.bin").read_bytes(), 24) ==
                                                        [(2, 2), (3, 2), (4, 2), (4, 3), (4, 4), (3, 4), (2, 4), (2, 3)], "Contour round trip failed"))
        heartbeat = package(root / "heartbeat", "Heartbeat")
        run("valid_heartbeat", lambda: validate_cluster_packet(heartbeat))
        delta = package(root / "delta", "Delta")
        run("valid_delta", lambda: validate_cluster_packet(delta))
        run("reject_unknown_top_level", lambda: expect_invalid(mutate(base, root / "unknown", lambda x: x.update({"世界存在身份": "forbidden"})), "Unknown field accepted"))
        run("reject_fake_precise_depth", lambda: expect_invalid(mutate(base, root / "fake_precise", lambda x: x["簇变化"][0]["距离"].update({"模式": "PreciseDepth3D", "值米": 1.0})), "Fake precise depth accepted"))
        run("reject_broken_material_hash", lambda: expect_invalid(mutate(base, root / "hash", lambda x: x["材料"]["精确轮廓链"].update({"SHA256": "0" * 64})), "Bad material hash accepted"))
        run("reject_bad_coverage_partition", lambda: expect_invalid(mutate(base, root / "coverage", lambda x: x["全局覆盖摘要"].update({"未处理像素数": 1})), "Bad coverage accepted"))
        run("reject_delta_without_base", lambda: expect_invalid(mutate(delta, root / "delta_base", lambda x: x.update({"依赖全量序号": None})), "Delta without base accepted"))
        run("reject_heartbeat_payload", lambda: expect_invalid(mutate(heartbeat, root / "heartbeat_payload", lambda x: x.update({"簇变化": json.loads(base.read_text(encoding="utf-8"))["簇变化"]})), "Heartbeat payload accepted"))
        run("reject_reappearance_state_mismatch", lambda: expect_invalid(mutate(base, root / "reappearance_mismatch", lambda x: x["簇变化"][0].update({"跟踪状态": "Reappeared"})), "Mismatched reappearance accepted"))
        run("reject_noncanonical_contour", lambda: expect_invalid(mutate(base, root / "noncanonical", lambda x: x["簇变化"][0]["轮廓"][0].update({"起点XY": [3, 2]})), "Noncanonical contour accepted"))
        report["status"] = "pass"
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
