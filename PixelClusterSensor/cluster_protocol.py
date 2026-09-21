"""Strict PCS.ClusterObservation/1 reader, contour codec, and semantic validator.

This module is independent of the camera process.  It deliberately validates
the compact cluster package rather than treating an image overlay as evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable


MAX_PACKET_JSON_BYTES = 1024 * 1024
MAX_MATERIAL_BYTES = 16 * 1024 * 1024
MAX_CLUSTERS = 32768
MAX_IMAGE_PIXELS = 1280 * 720

PACKET_TYPES = {"FullSnapshot", "Delta", "Heartbeat"}
INTENTS = {"Scan", "Observe", "Track", "Mixed"}
CHANGE_TYPES = {"Added", "Updated", "Moved", "ShapeChanged", "PartiallyOccluded", "Occluded", "Reappeared", "Lost", "Removed"}
TRACK_STATES = {"Tentative", "Active", "PartOccluded", "Occluded", "Reappeared", "Lost", "Retired"}
DISTANCE_MODES = {"PreciseDepth3D", "ApproxStereoContour", "ImageOnlyContour", "UnknownDistance"}

TOP_LEVEL_KEYS = {
    "格式", "发布状态", "包标识", "会话标识", "跟踪时期", "输出序号", "场景版本", "包类型", "任务意图", "依赖全量序号",
    "源时间", "发布Unix毫秒", "结果年龄毫秒", "配置版本", "标定版本", "坐标系", "图像尺寸WH", "相机姿态", "处理区域",
    "输入质量", "全局覆盖摘要", "材料", "簇变化", "指标",
}
CLUSTER_KEYS = {
    "帧内簇编号", "相机跟踪候选编号", "自我绑定令牌", "变化类型", "跟踪状态", "范围XYWH", "图像中心XY", "像素数", "触及视野边界",
    "轮廓", "形状指纹", "颜色摘要", "距离", "三维中心米", "尺寸米", "深度证据", "运动", "遮挡", "关联证据", "时效", "详细材料句柄",
}


def check(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _only_keys(value: dict, allowed: set[str], context: str) -> None:
    check(isinstance(value, dict), f"{context} must be an object")
    unknown = set(value) - allowed
    check(not unknown, f"{context} contains unknown fields: {sorted(unknown)}")


def _string(value, context: str, minimum: int = 1, maximum: int = 256) -> str:
    check(isinstance(value, str), f"{context} must be a string")
    encoded = value.encode("utf-8")
    check(minimum <= len(encoded) <= maximum, f"{context} byte length is out of range")
    return value


def _decimal(value, context: str, minimum: int = 0) -> str:
    text = _string(value, context, 1, 32)
    check(text.isascii() and text.isdecimal() and (text == "0" or not text.startswith("0")), f"{context} must be canonical decimal")
    check(int(text) >= minimum, f"{context} is below minimum")
    return text


def _finite(value, context: str, minimum: float | None = None) -> float:
    check(type(value) in (int, float) and math.isfinite(value), f"{context} must be finite")
    result = float(value)
    if minimum is not None:
        check(result >= minimum, f"{context} is below minimum")
    return result


def _integer(value, context: str, minimum: int, maximum: int) -> int:
    check(type(value) is int and minimum <= value <= maximum, f"{context} is out of range")
    return value


def _rect(value, context: str, width: int, height: int) -> list[int]:
    check(isinstance(value, list) and len(value) == 4, f"{context} must be XYWH")
    x, y, w, h = (_integer(v, context, 0, max(width, height)) for v in value)
    check(w > 0 and h > 0 and x + w <= width and y + h <= height, f"{context} is outside image")
    return [x, y, w, h]


def _sha256(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def _signed_area(points: list[tuple[int, int]]) -> int:
    return sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]))


def canonicalize_ring(points: Iterable[Iterable[int]], inner: bool) -> list[tuple[int, int]]:
    """Normalize an 8-connected ring without changing its pixels."""
    ring = []
    for point in points:
        pair = list(point)
        check(len(pair) == 2 and all(type(v) is int for v in pair), "Contour point must be integer XY")
        ring.append((pair[0], pair[1]))
    check(len(ring) >= 3, "Contour ring needs at least three points")
    if ring[0] == ring[-1]:
        ring.pop()
    check(len(ring) >= 3 and len(set(ring)) >= 3, "Contour ring is degenerate")
    for current, following in zip(ring, ring[1:] + ring[:1]):
        dx, dy = following[0] - current[0], following[1] - current[1]
        check(max(abs(dx), abs(dy)) == 1, "Contour is not an 8-connected chain")
    area = _signed_area(ring)
    check(area != 0, "Contour ring area is zero")
    # In image coordinates (Y downward), a positive shoelace area is clockwise.
    want_positive = not inner
    if (area > 0) != want_positive:
        ring.reverse()
    start = min(range(len(ring)), key=lambda index: (ring[index][1], ring[index][0]))
    return ring[start:] + ring[:start]


_DIRECTION_TO_CODE = {(1, 0): 0, (1, 1): 1, (0, 1): 2, (-1, 1): 3, (-1, 0): 4, (-1, -1): 5, (0, -1): 6, (1, -1): 7}
_CODE_TO_DIRECTION = {value: key for key, value in _DIRECTION_TO_CODE.items()}


def pack_ring(points: Iterable[Iterable[int]], inner: bool) -> tuple[list[tuple[int, int]], bytes, int]:
    ring = canonicalize_ring(points, inner)
    codes = []
    for current, following in zip(ring, ring[1:] + ring[:1]):
        codes.append(_DIRECTION_TO_CODE[(following[0] - current[0], following[1] - current[1])])
    raw = bytearray((len(codes) * 3 + 7) // 8)
    for index, code in enumerate(codes):
        bit = index * 3
        raw[bit // 8] |= (code << (bit % 8)) & 0xFF
        if bit % 8 > 5:
            raw[bit // 8 + 1] |= code >> (8 - (bit % 8))
    return ring, bytes(raw), len(codes) * 3


def unpack_ring(start_xy: Iterable[int], point_count: int, raw: bytes, valid_bits: int) -> list[tuple[int, int]]:
    start = list(start_xy)
    check(len(start) == 2 and all(type(v) is int for v in start), "Contour start must be integer XY")
    check(type(point_count) is int and point_count >= 3 and valid_bits == point_count * 3, "Contour bit count mismatch")
    check(len(raw) == (valid_bits + 7) // 8, "Contour byte size mismatch")
    if valid_bits % 8:
        check(raw[-1] >> (valid_bits % 8) == 0, "Contour padding bits must be zero")
    ring = [(start[0], start[1])]
    x, y = start
    for index in range(point_count):
        bit = index * 3
        word = raw[bit // 8] | ((raw[bit // 8 + 1] if bit // 8 + 1 < len(raw) else 0) << 8)
        code = (word >> (bit % 8)) & 7
        dx, dy = _CODE_TO_DIRECTION[code]
        x, y = x + dx, y + dy
        if index + 1 < point_count:
            ring.append((x, y))
    check((x, y) == ring[0], "Contour chain does not close")
    return ring


def _read_material(root: Path, descriptor: dict) -> bytes:
    _only_keys(descriptor, {"文件", "编码", "字节数", "SHA256"}, "Material descriptor")
    name = _string(descriptor.get("文件"), "Material filename", 1, 240)
    check(Path(name).name == name and name not in {".", ".."}, "Material must be a direct package child")
    check(descriptor.get("编码") == "PCS.ContourChain8/1", "Unsupported cluster material encoding")
    expected_size = _integer(descriptor.get("字节数"), "Material byte count", 0, MAX_MATERIAL_BYTES)
    expected_hash = _string(descriptor.get("SHA256"), "Material SHA256", 64, 64)
    check(all(char in "0123456789abcdef" for char in expected_hash), "Material SHA256 must be lowercase hex")
    path = (root / name).resolve()
    check(path.parent == root.resolve() and path.is_file(), "Material location is invalid")
    raw = path.read_bytes()
    check(len(raw) == expected_size and _sha256(raw) == expected_hash, "Material integrity mismatch")
    return raw


def _validate_header(packet: dict) -> tuple[int, int]:
    _only_keys(packet, TOP_LEVEL_KEYS, "Cluster packet")
    required = TOP_LEVEL_KEYS - {"指标"}
    check(required <= set(packet), "Cluster packet is missing required fields")
    check(packet["格式"] == "PCS.ClusterObservation/1" and packet["发布状态"] == "完整", "Unsupported cluster package")
    _string(packet["包标识"], "Package ID", 1, 128)
    _string(packet["会话标识"], "Session ID", 1, 128)
    _decimal(packet["跟踪时期"], "Tracking epoch", 1)
    _decimal(packet["输出序号"], "Output sequence", 1)
    _decimal(packet["场景版本"], "Scene version", 1)
    check(packet["包类型"] in PACKET_TYPES, "Unknown packet type")
    check(packet["任务意图"] in INTENTS, "Unknown packet intent")
    dependency = packet["依赖全量序号"]
    if packet["包类型"] == "FullSnapshot":
        check(dependency is None, "Full snapshot must not depend on another snapshot")
    else:
        _decimal(dependency, "Base snapshot sequence", 1)
    source_time = packet["源时间"]
    _only_keys(source_time, {"值", "单位", "时间域"}, "Source time")
    _finite(source_time.get("值"), "Source time value")
    _string(source_time.get("单位"), "Source time unit", 1, 16)
    _string(source_time.get("时间域"), "Source time domain", 1, 128)
    _decimal(packet["发布Unix毫秒"], "Publication time", 1)
    if packet["结果年龄毫秒"] is not None:
        _finite(packet["结果年龄毫秒"], "Result age", 0)
    _string(packet["配置版本"], "Configuration version", 1, 128)
    _string(packet["标定版本"], "Calibration version", 1, 256)
    _string(packet["坐标系"], "Coordinate system", 1, 128)
    image = packet["图像尺寸WH"]
    check(isinstance(image, list) and len(image) == 2, "Image dimensions must be WH")
    width = _integer(image[0], "Image width", 1, 1280)
    height = _integer(image[1], "Image height", 1, 720)
    check(width * height <= MAX_IMAGE_PIXELS, "Image exceeds protocol pixel limit")
    check(packet["相机姿态"] is None or isinstance(packet["相机姿态"], dict), "Camera pose must be null or object")
    regions = packet["处理区域"]
    _only_keys(regions, {"高精度ROI", "外围扫描区", "未处理区"}, "Processing regions")
    for key in regions:
        check(isinstance(regions[key], list), f"{key} must be an array")
        for index, rect in enumerate(regions[key]):
            _rect(rect, f"{key}[{index}]", width, height)
    quality = packet["输入质量"]
    _only_keys(quality, {"源缺口", "同步状态", "预热状态"}, "Input quality")
    check(type(quality.get("源缺口")) is bool, "Source gap must be boolean")
    check(quality.get("同步状态") in {"通过", "不通过", "未知"}, "Invalid synchronization state")
    check(quality.get("预热状态") in {"未证明稳定", "通过", "不通过"}, "Invalid warmup state")
    coverage = packet["全局覆盖摘要"]
    _only_keys(coverage, {"已处理像素数", "未知像素数", "无效像素数", "遮挡像素数", "未处理像素数"}, "Coverage summary")
    counts = [_integer(coverage.get(key), key, 0, width * height) for key in coverage]
    check(sum(counts) == width * height, "Coverage categories must partition the image")
    return width, height


def _validate_cluster(entry: dict, width: int, height: int, contours: bytes, active_track_ids: set[str]) -> dict:
    _only_keys(entry, CLUSTER_KEYS, "Cluster entry")
    check(CLUSTER_KEYS <= set(entry), "Cluster entry is missing required fields")
    track = entry["相机跟踪候选编号"]
    if track is not None:
        track = _decimal(track, "Camera tracking candidate ID", 1)
        check(track not in active_track_ids, "Duplicate active tracking candidate ID")
        active_track_ids.add(track)
    if entry["自我绑定令牌"] is not None:
        _string(entry["自我绑定令牌"], "Self binding token", 1, 256)
    check(entry["变化类型"] in CHANGE_TYPES, "Unknown change type")
    check(entry["跟踪状态"] in TRACK_STATES, "Unknown tracking state")
    if entry["帧内簇编号"] is None:
        check(entry["变化类型"] in {"Occluded", "Lost", "Removed"} and track is not None and entry["跟踪状态"] in {"Occluded", "Lost", "Retired"},
              "Absent-cluster event needs a tracked candidate")
        if entry["变化类型"] == "Occluded":
            check(entry["跟踪状态"] == "Occluded", "Occlusion event needs Occluded state")
        else:
            check(entry["跟踪状态"] in {"Lost", "Retired"}, "Terminal event needs Lost or Retired state")
        for key in {"范围XYWH", "图像中心XY", "像素数", "触及视野边界", "轮廓", "形状指纹", "颜色摘要", "距离", "三维中心米", "尺寸米", "深度证据", "运动"}:
            check(entry[key] is None, f"Tombstone field must be null: {key}")
        occlusion = entry["遮挡"]
        _only_keys(occlusion, {"状态", "比例", "依据"}, "Tombstone occlusion")
        expected_occlusion = "Unknown" if entry["变化类型"] == "Removed" else "Occluded"
        check(occlusion.get("状态") == expected_occlusion and occlusion.get("比例") is None,
              "Tombstone occlusion state disagrees with removal reason")
        _string(occlusion.get("依据"), "Tombstone reason", 1, 256)
        freshness = entry["时效"]
        _only_keys(freshness, {"连续可见帧数", "连续缺失帧数", "证据年龄毫秒"}, "Tombstone freshness")
        _integer(freshness.get("连续可见帧数"), "Visible frame count", 0, 1_000_000)
        _integer(freshness.get("连续缺失帧数"), "Missing frame count", 1, 1_000_000)
        if freshness.get("证据年龄毫秒") is not None:
            _finite(freshness["证据年龄毫秒"], "Evidence age", 0)
        check(entry["关联证据"] is None and entry["详细材料句柄"] is None, "Tombstone cannot carry current association or material")
        return {"frame_cluster_id": None, "track_id": track, "pixels": 0}
    _integer(entry["帧内簇编号"], "Frame cluster ID", 1, MAX_CLUSTERS)
    rect = _rect(entry["范围XYWH"], "Cluster bounding box", width, height)
    center = entry["图像中心XY"]
    check(isinstance(center, list) and len(center) == 2, "Cluster center must be XY")
    cx, cy = (_finite(v, "Cluster center") for v in center)
    check(rect[0] <= cx <= rect[0] + rect[2] - 1 and rect[1] <= cy <= rect[1] + rect[3] - 1, "Cluster center is outside bbox")
    _integer(entry["像素数"], "Cluster pixel count", 1, width * height)
    check(type(entry["触及视野边界"]) is bool, "Border-touch flag must be boolean")
    rings = entry["轮廓"]
    check(isinstance(rings, list) and rings, "Cluster needs at least one contour ring")
    ring_topology: list[tuple[int, bool]] = []
    for index, ring in enumerate(rings):
        _only_keys(ring, {"材料键", "字节偏移", "有效位数", "起点XY", "点数", "父环索引", "内环", "闭合区域边界", "物理孔洞确认", "边界原因位"}, "Contour ring")
        check(ring.get("材料键") == "精确轮廓链", "Contour ring must reference exact contour chain")
        offset = _integer(ring.get("字节偏移"), "Contour byte offset", 0, len(contours))
        bits = _integer(ring.get("有效位数"), "Contour bit count", 9, len(contours) * 8)
        points = _integer(ring.get("点数"), "Contour point count", 3, MAX_IMAGE_PIXELS)
        length = (bits + 7) // 8
        check(offset + length <= len(contours) and bits == points * 3, "Contour range is invalid")
        ring_points = unpack_ring(ring.get("起点XY"), points, contours[offset:offset + length], bits)
        check(canonicalize_ring(ring_points, bool(ring.get("内环"))) == ring_points, "Contour is not canonical")
        check(all(rect[0] <= x < rect[0] + rect[2] and rect[1] <= y < rect[1] + rect[3] for x, y in ring_points), "Contour is outside cluster bbox")
        parent = _integer(ring.get("父环索引"), "Contour parent", -1, len(rings) - 1)
        check(parent != index, "Contour cannot be its own parent")
        check(type(ring.get("内环")) is bool and type(ring.get("闭合区域边界")) is bool and ring.get("物理孔洞确认") is False, "Invalid contour topology flags")
        _integer(ring.get("边界原因位"), "Boundary reason bits", 0, 7)
        ring_topology.append((parent, ring["内环"]))
    for index, (_, inner) in enumerate(ring_topology):
        seen = {index}
        parent, depth = ring_topology[index][0], 0
        while parent != -1:
            check(parent not in seen, "Contour hierarchy cycle")
            seen.add(parent)
            depth += 1
            parent = ring_topology[parent][0]
        check(inner == bool(depth % 2), "Contour inner-ring parity is invalid")
    fingerprint = entry["形状指纹"]
    _only_keys(fingerprint, {"归一化规则", "层级"}, "Shape fingerprint")
    check(fingerprint.get("归一化规则") in {"filled-contour-center-square/1", "label-mask-center-square/1"}, "Unknown shape fingerprint normalization")
    levels = fingerprint.get("层级")
    check(isinstance(levels, list) and len(levels) == 3 and all(isinstance(level, dict) for level in levels), "Shape levels must be an array")
    check([level.get("边长") for level in levels] == [8, 16, 32], "Shape levels must be 8,16,32")
    for level in levels:
        _only_keys(level, {"边长", "位图SHA256", "前景像素数"}, "Shape level")
        _integer(level["边长"], "Shape side", 8, 32)
        _string(level["位图SHA256"], "Shape bitmap SHA256", 64, 64)
        _integer(level["前景像素数"], "Shape foreground count", 0, 32 * 32)
    color = entry["颜色摘要"]
    _only_keys(color, {"颜色空间", "RGB均值", "有效像素数", "可比性"}, "Color summary")
    check(color.get("颜色空间") == "sRGB", "Unsupported color space")
    check(isinstance(color.get("RGB均值"), list) and len(color["RGB均值"]) == 3, "RGB mean must be RGB")
    for value in color["RGB均值"]:
        _integer(value, "RGB mean", 0, 255)
    _integer(color.get("有效像素数"), "Color sample count", 1, width * height)
    check(color.get("可比性") in {"当前帧", "受光照变化限制", "未知"}, "Invalid color comparability")
    distance = entry["距离"]
    _only_keys(distance, {"模式", "值米", "区间米", "不确定度米", "依据"}, "Distance")
    check(distance.get("模式") in DISTANCE_MODES, "Unknown distance mode")
    for key in ("值米", "不确定度米"):
        if distance.get(key) is not None:
            _finite(distance[key], key, 0)
    interval = distance.get("区间米")
    if interval is not None:
        check(isinstance(interval, list) and len(interval) == 2, "Distance interval must have two values")
        low, high = (_finite(v, "Distance interval", 0) for v in interval)
        check(low <= high, "Distance interval is reversed")
    check(distance.get("依据") in {"当前可靠深度", "当前双目轮廓", "仅图像轮廓", "证据不足"}, "Invalid distance evidence")
    if distance["模式"] == "PreciseDepth3D":
        check(distance["值米"] is not None and distance["依据"] == "当前可靠深度", "Precise depth requires current reliable evidence")
    if distance["模式"] == "UnknownDistance":
        check(distance["值米"] is None and distance["区间米"] is None and distance["不确定度米"] is None and distance["依据"] == "证据不足",
              "Unknown distance must not carry invented metric values")
    evidence = entry["深度证据"]
    _only_keys(evidence, {"当前实测像素数", "历史候选像素数", "估算像素数", "缺失像素数", "范围外像素数"}, "Depth evidence")
    evidence_counts = [_integer(evidence.get(key), key, 0, width * height) for key in evidence]
    check(sum(evidence_counts) == entry["像素数"], "Depth evidence must partition cluster pixels")
    for key in ("三维中心米", "尺寸米", "运动", "关联证据", "详细材料句柄"):
        check(entry[key] is None or isinstance(entry[key], (dict, str, list)), f"{key} must be null or structured")
    occlusion = entry["遮挡"]
    _only_keys(occlusion, {"状态", "比例", "依据"}, "Occlusion")
    check(occlusion.get("状态") in {"Visible", "PartiallyOccluded", "Occluded", "Unknown"}, "Invalid occlusion state")
    if occlusion.get("比例") is not None:
        _finite(occlusion["比例"], "Occlusion ratio", 0)
        check(occlusion["比例"] <= 1, "Occlusion ratio exceeds one")
    _string(occlusion.get("依据"), "Occlusion evidence", 1, 256)
    freshness = entry["时效"]
    _only_keys(freshness, {"连续可见帧数", "连续缺失帧数", "证据年龄毫秒"}, "Freshness")
    _integer(freshness.get("连续可见帧数"), "Visible frame count", 0, 1_000_000)
    _integer(freshness.get("连续缺失帧数"), "Missing frame count", 0, 1_000_000)
    if freshness.get("证据年龄毫秒") is not None:
        _finite(freshness["证据年龄毫秒"], "Evidence age", 0)
    return {"frame_cluster_id": entry["帧内簇编号"], "track_id": track, "pixels": entry["像素数"]}


def validate_cluster_packet(path: Path) -> dict:
    path = path.resolve()
    raw = path.read_bytes()
    check(len(raw) <= MAX_PACKET_JSON_BYTES, "Cluster packet JSON exceeds limit")
    packet = json.loads(raw)
    width, height = _validate_header(packet)
    materials = packet["材料"]
    _only_keys(materials, {"精确轮廓链"}, "Cluster materials")
    check("精确轮廓链" in materials, "Exact contour material is required")
    contours = _read_material(path.parent, materials["精确轮廓链"])
    changes = packet["簇变化"]
    check(isinstance(changes, list) and len(changes) <= MAX_CLUSTERS, "Cluster change list exceeds limit")
    if packet["包类型"] == "Heartbeat":
        check(not changes, "Heartbeat cannot carry cluster changes")
    active_tracks: set[str] = set()
    records = [_validate_cluster(entry, width, height, contours, active_tracks) for entry in changes]
    frame_ids = [record["frame_cluster_id"] for record in records if record["frame_cluster_id"] is not None]
    check(len(frame_ids) == len(set(frame_ids)), "Duplicate frame-local cluster ID")
    return {"status": "pass", "format": packet["格式"], "packet_type": packet["包类型"], "intent": packet["任务意图"],
            "clusters": len(records), "contour_bytes": len(contours), "image_pixels": width * height}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("packet", type=Path)
    args = parser.parse_args()
    print(json.dumps(validate_cluster_packet(args.packet), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
