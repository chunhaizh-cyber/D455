"""Convert one PCS.Observation/1 material directory into a PCS.ClusterObservation/1 full snapshot.

This is the P1 bridge.  It is intentionally stateless: no cross-frame tracking,
no history reuse, and no upgrade from in-range depth to calibrated precision.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from cluster_protocol import MAX_CLUSTERS, MAX_IMAGE_PIXELS, pack_ring, validate_cluster_packet
from validate_packet import read_material, validate_packet


def sha256(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def canonical_json_hash(value: object) -> str:
    raw = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return "sha256:" + sha256(raw)


def packed_mask_hash(mask: np.ndarray, side: int) -> tuple[str, int]:
    """Deterministically normalize a label mask without claiming physical-hole semantics."""
    height, width = mask.shape
    square = max(height, width)
    canvas = np.zeros((square, square), dtype=np.uint8)
    top, left = (square - height) // 2, (square - width) // 2
    canvas[top:top + height, left:left + width] = mask.astype(np.uint8)
    reduced = np.zeros((side, side), dtype=np.uint8)
    for y in range(side):
        y0, y1 = y * square // side, (y + 1) * square // side
        for x in range(side):
            x0, x1 = x * square // side, (x + 1) * square // side
            reduced[y, x] = 1 if np.any(canvas[y0:y1, x0:x1]) else 0
    bits = np.packbits(reduced.reshape(-1), bitorder="little").tobytes()
    return sha256(bits), int(reduced.sum())


def shape_fingerprint(mask: np.ndarray) -> dict:
    return {
        "归一化规则": "label-mask-center-square/1",
        "层级": [{"边长": side, "位图SHA256": digest, "前景像素数": foreground}
                  for side, (digest, foreground) in ((side, packed_mask_hash(mask, side)) for side in (8, 16, 32))],
    }


def source_time(source: dict) -> dict:
    return {"值": source["彩图时间戳毫秒"], "单位": "ms", "时间域": source["彩图时间域"]}


def encode_valid_rings(source_rings: list[dict], points: np.ndarray) -> tuple[list[dict], bytes, int]:
    """Encode only closed, non-degenerate source rings.

    The producer can expose one- and two-pixel segmentation fragments.  They
    are useful source diagnostics but cannot be represented by the closed-ring
    protocol.  Do not invent geometry for them: discard a malformed ring, and
    let the caller downgrade a cluster with no remaining outer ring to Unknown.
    """
    accepted: list[tuple[int, dict, list[tuple[int, int]], bytes, int, int]] = []
    rejected = 0
    for index, source_ring in enumerate(source_rings):
        start, count = source_ring["起始点"], source_ring["点数"]
        xy = [(int(px), int(py)) for px, py in points[start:start + count, :2]]
        try:
            normalized, raw, bit_count = pack_ring(xy, bool(source_ring["内环"]))
        except ValueError:
            rejected += 1
            continue
        boundary_reasons = points[start:start + count, 2]
        accepted.append((index, source_ring, normalized, raw, bit_count,
                         int(np.bitwise_or.reduce(boundary_reasons, initial=0))))

    # A retained inner ring must retain its complete parent chain.  Otherwise
    # its topology would claim a hole in a contour that is no longer present.
    by_source_index = {item[0]: item for item in accepted}
    retained: list[tuple[int, dict, list[tuple[int, int]], bytes, int, int]] = []
    for item in accepted:
        parent = item[1]["父轮廓索引"]
        ancestors: set[int] = set()
        while parent != -1:
            if parent in ancestors or parent not in by_source_index:
                break
            ancestors.add(parent)
            parent = by_source_index[parent][1]["父轮廓索引"]
        else:
            retained.append(item)

    remapped = {item[0]: index for index, item in enumerate(retained)}
    encoded = bytearray()
    rings = []
    for source_index, source_ring, normalized, raw, bit_count, boundary_reason in retained:
        parent = source_ring["父轮廓索引"]
        offset = len(encoded)
        encoded.extend(raw)
        rings.append({
            "材料键": "精确轮廓链", "字节偏移": offset, "有效位数": bit_count,
            "起点XY": list(normalized[0]), "点数": len(normalized),
            "父环索引": -1 if parent == -1 else remapped[parent], "内环": source_ring["内环"],
            "闭合区域边界": source_ring["闭合区域边界"], "物理孔洞确认": False,
            "边界原因位": boundary_reason,
        })
    return rings, bytes(encoded), rejected + len(accepted) - len(retained)


def convert(source_path: Path, output: Path, minimum_cluster_pixels: int = 1) -> dict:
    """Write a published cluster snapshot to a new directory and return its validation report."""
    source_path = source_path.resolve()
    output = output.resolve()
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if output.parent.joinpath(output.name + ".pending").exists():
        raise ValueError(f"Pending output already exists: {output.name}.pending")
    validate_packet(source_path)
    manifest = json.loads(source_path.read_text(encoding="utf-8"))
    arrays = {key: read_material(source_path.parent, descriptor) for key, descriptor in manifest["材料"].items()}
    labels = arrays["簇归属图"]
    ownership = arrays["归属状态"]
    depth_state = arrays["当前深度状态"]
    fill_state = arrays["补全状态"]
    color = arrays["颜色图"]
    points = arrays["轮廓点"]
    height, width = labels.shape
    if width * height > MAX_IMAGE_PIXELS or len(manifest["簇目录"]) > MAX_CLUSTERS:
        raise ValueError("Source observation exceeds cluster package bounds")
    if minimum_cluster_pixels < 1:
        raise ValueError("minimum_cluster_pixels must be positive")
    contour_bytes = bytearray()
    entries = []
    downgraded_pixels = 0
    rejected_ring_count = 0
    below_minimum_cluster_count = 0
    for source_cluster in manifest["簇目录"]:
        cluster_id = source_cluster["本帧簇编号"]
        mask = labels == cluster_id
        y, x = np.where(mask)
        box = source_cluster["范围XYWH"]
        if int(mask.sum()) < minimum_cluster_pixels:
            downgraded_pixels += int(mask.sum())
            below_minimum_cluster_count += 1
            continue
        rings, encoded_rings, rejected = encode_valid_rings(source_cluster["轮廓"], points)
        rejected_ring_count += rejected
        if not any(not ring["内环"] for ring in rings):
            # A protocol cluster is a closed visible region.  A source fragment
            # without an outer ring is not silently promoted to a cluster.
            downgraded_pixels += int(mask.sum())
            continue
        base_offset = len(contour_bytes)
        contour_bytes.extend(encoded_rings)
        for ring in rings:
            ring["字节偏移"] += base_offset
        measured = int(np.count_nonzero(mask & (depth_state == 1)))
        out_of_range = int(np.count_nonzero(mask & (depth_state == 2)))
        estimated = int(np.count_nonzero(mask & (fill_state == 1)))
        missing = int(np.count_nonzero(mask & (depth_state == 0) & (fill_state == 0)))
        mean_rgb = [int(round(float(value))) for value in color[mask].mean(axis=0)]
        entries.append({
            "帧内簇编号": cluster_id,
            "相机跟踪候选编号": None,
            "自我绑定令牌": None,
            "变化类型": "Added",
            "跟踪状态": "Tentative",
            "范围XYWH": box,
            "图像中心XY": [box[0] + (box[2] - 1) / 2.0, box[1] + (box[3] - 1) / 2.0],
            "像素数": int(mask.sum()),
            "触及视野边界": bool(box[0] == 0 or box[1] == 0 or box[0] + box[2] == width or box[1] + box[3] == height),
            "轮廓": rings,
            "形状指纹": shape_fingerprint(mask[box[1]:box[1] + box[3], box[0]:box[0] + box[2]]),
            "颜色摘要": {"颜色空间": "sRGB", "RGB均值": mean_rgb, "有效像素数": int(mask.sum()), "可比性": "当前帧"},
            # PCS.Observation/1 has in-range samples, not a calibrated precision guarantee.  Do not upgrade it here.
            "距离": {"模式": "UnknownDistance", "值米": None, "区间米": None, "不确定度米": None, "依据": "证据不足"},
            "三维中心米": None,
            "尺寸米": None,
            "深度证据": {"当前实测像素数": measured, "历史候选像素数": 0, "估算像素数": estimated, "缺失像素数": missing, "范围外像素数": out_of_range},
            "运动": None,
            "遮挡": {"状态": "Unknown", "比例": None, "依据": "单帧无时序证据"},
            "关联证据": None,
            "时效": {"连续可见帧数": 1, "连续缺失帧数": 0, "证据年龄毫秒": None},
            "详细材料句柄": None,
        })
    source = manifest["源信息"]
    unknown = int(np.count_nonzero(ownership == 0)) + downgraded_pixels
    staging = output.parent / (output.name + ".pending")
    staging.mkdir(parents=True)
    try:
        raw_contours = bytes(contour_bytes)
        (staging / "contours.bin").write_bytes(raw_contours)
        packet = {
            "格式": "PCS.ClusterObservation/1", "发布状态": "完整", "包标识": "cluster-" + manifest["输出序号"],
            "会话标识": manifest["会话标识"], "跟踪时期": "1", "输出序号": manifest["输出序号"], "场景版本": manifest["输出序号"],
            "包类型": "FullSnapshot", "任务意图": "Scan", "依赖全量序号": None, "前置场景版本": None,
            "源时间": source_time(source), "发布Unix毫秒": str(manifest["输出Unix毫秒"]), "结果年龄毫秒": None,
            "配置版本": manifest["配置版本"], "标定版本": canonical_json_hash(manifest["标定"]), "坐标系": "D455_Color_Optical",
            "图像尺寸WH": [width, height], "相机姿态": None,
            "处理区域": {"高精度ROI": [], "外围扫描区": [[0, 0, width, height]], "未处理区": []},
            "输入质量": {
                "源缺口": bool(source.get("彩图源帧缺口", 0) or source.get("深度源帧缺口", 0)),
                "同步状态": "未知", "预热状态": source.get("预热状态", "未证明稳定"),
            },
            "全局覆盖摘要": {"已处理像素数": int(labels.size - unknown), "未知像素数": unknown, "无效像素数": 0, "遮挡像素数": 0, "未处理像素数": 0},
            "材料": {"精确轮廓链": {"文件": "contours.bin", "编码": "PCS.ContourChain8/1", "字节数": len(raw_contours), "SHA256": sha256(raw_contours)}},
            "簇变化": entries,
            "指标": {"转换来源格式": manifest["格式"], "转换来源清单SHA256": sha256(source_path.read_bytes()), "跟踪": "not_implemented", "精确深度晋级": "not_implemented",
                     "退化轮廓环数": rejected_ring_count, "低于最小簇像素数簇数": below_minimum_cluster_count,
                     "最小簇像素数": minimum_cluster_pixels, "降级未知像素数": downgraded_pixels},
        }
        packet_path = staging / "packet.json"
        packet_path.write_text(json.dumps(packet, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        report = validate_cluster_packet(packet_path)
        staging.rename(output)
        return {**report, "packet": str(output / "packet.json")}
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_packet", type=Path, help="PCS.Observation/1 frame.json")
    parser.add_argument("--output", required=True, type=Path, help="New PCS.ClusterObservation/1 directory")
    parser.add_argument("--minimum-cluster-pixels", type=int, default=1)
    args = parser.parse_args()
    print(json.dumps(convert(args.source_packet, args.output, args.minimum_cluster_pixels), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
