"""Independent package reader; does not call the producer or its segmentation code."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image


def check(condition, message):
    if not condition:
        raise ValueError(message)


def read_material(root: Path, descriptor: dict):
    path = (root / descriptor["文件"]).resolve()
    check(path.parent == root.resolve(), "Material must be a direct package child")
    check(path.stat().st_size <= 96 * 1024 * 1024, "Material exceeds limit")
    raw = path.read_bytes()
    check(len(raw) == descriptor["字节数"], "Byte length mismatch")
    check(hashlib.sha256(raw).hexdigest() == descriptor["SHA256"], "SHA256 mismatch")
    shape = descriptor["形状"]
    check(isinstance(shape, list) and all(type(x) is int and 0 <= x <= 1280 * 720 * 4 for x in shape), "Invalid shape")
    encoding = descriptor["编码"]
    if encoding.startswith("PNG_"):
        with Image.open(io.BytesIO(raw)) as image:
            check(image.size == (shape[1], shape[0]), "PNG dimensions mismatch")
            if encoding == "PNG_RGB8":
                check(image.mode == "RGB", "Color image is not RGB")
            array = np.array(image)
        check(list(array.shape) == shape, "PNG shape mismatch")
        if encoding == "PNG_U16":
            check(array.dtype.kind in "ui" and array.min() >= 0 and array.max() <= 65535, "Not depth16")
        return array
    dtype = {"uint8": "u1", "uint32_le": "<u4", "int32_le": "<i4", "float32_le": "<f4"}[encoding]
    check(int(np.prod(shape, dtype=np.int64)) * np.dtype(dtype).itemsize == len(raw), "Array size mismatch")
    return np.frombuffer(raw, dtype=dtype).reshape(shape)


def validate_packet(path: Path, *, reference=None, expected_color=None, expected_depth=None, output=None):
    manifest_bytes = path.read_bytes()
    if reference:
        check(hashlib.sha256(manifest_bytes).hexdigest() == reference["清单SHA256"], "Manifest checksum mismatch")
    manifest = json.loads(manifest_bytes)
    check(manifest["格式"] == "PCS.Observation/1" and manifest["发布状态"] == "完整", "Not a complete supported packet")
    check(manifest["体素"] is False and manifest["已确认存在身份"] is False, "Invalid evidence claim")
    arrays = {key: read_material(path.parent, value) for key, value in manifest["材料"].items()}
    color = arrays["颜色图"]
    labels = arrays["簇归属图"]
    own = arrays["归属状态"]
    depth = arrays["当前观测深度米"]
    state = arrays["当前深度状态"]
    source_index = arrays["源深度像素索引"]
    filled = arrays["补全深度米"]
    fill_state = arrays["补全状态"]
    raw_depth = arrays["原始深度"]
    points = arrays["轮廓点"]
    height, width = labels.shape
    check(manifest["图像尺寸WH"] == [width, height], "Frame dimensions mismatch")
    check(color.shape == (height, width, 3), "Color dimensions mismatch")
    for value in (own, depth, state, source_index, filled, fill_state):
        check(value.shape == labels.shape, "Shared image-plane mismatch")
    check(np.all(np.isin(state, [0, 1, 2])) and np.all(np.isin(own, [0, 1, 2, 3])), "Invalid state")
    check(np.all(np.isin(fill_state, [0, 1])), "Invalid interpolation state")
    check(np.all(np.isfinite(depth)) and np.all(np.isfinite(filled)), "Nonfinite depth")
    check(np.all(depth[state == 0] == 0) and np.all(depth[state != 0] > 0), "Observation value/state conflict")
    check(np.all(filled[fill_state == 0] == 0) and np.all(filled[fill_state == 1] > 0), "Interpolation value/state conflict")
    check(np.all(state[fill_state == 1] == 0), "Interpolation overwrote current evidence")
    check(np.all(source_index[state == 0] == -1), "Missing samples must not acquire provenance")
    check(np.all((source_index[state != 0] >= 0) & (source_index[state != 0] < raw_depth.size)), "Invalid sample source")
    selected = raw_depth.reshape(-1)[source_index[state != 0]]
    check(np.all((selected != 0) & (selected != 65535)), "Invalid raw depth used as evidence")
    check(np.all((labels == 0) == (own == 0)), "Ownership ledger conflict")
    check(np.all(state[own == 1] == 1), "Depth-supported ownership without usable depth")
    ids = [entry["本帧簇编号"] for entry in manifest["簇目录"]]
    check(ids == list(range(1, len(ids) + 1)), "IDs are not canonical")
    check(set(np.unique(labels)) - {0} == set(ids), "Labels and directory disagree")
    first_pixels = [int(np.flatnonzero(labels == cluster_id)[0]) for cluster_id in ids]
    check(first_pixels == sorted(first_pixels), "IDs not in first-pixel order")
    reconstructed = np.zeros_like(color)
    hit_count = np.zeros_like(labels, dtype=np.uint8)
    used_points = np.zeros(len(points), dtype=np.uint8)
    for entry in manifest["簇目录"]:
        cluster_id = entry["本帧簇编号"]
        mask = labels == cluster_id
        yy, xx = np.where(mask)
        check(len(xx) == entry["像素数"], "Cluster pixel count mismatch")
        check(entry["范围XYWH"] == [int(xx.min()), int(yy.min()), int(xx.max() - xx.min() + 1), int(yy.max() - yy.min() + 1)], "Cluster bbox mismatch")
        check(int(np.count_nonzero(mask & (state == 1))) == entry["当前可用深度像素数"], "Measured count mismatch")
        check(int(np.count_nonzero(mask & (state == 2))) == entry["当前范围外深度像素数"], "Rejected range count mismatch")
        check(int(np.count_nonzero(mask & (fill_state == 1))) == entry["补全像素数"], "Filled count mismatch")
        check(int(np.count_nonzero(mask & (state == 0) & (fill_state == 0))) == entry["未解决深度像素数"], "Unknown depth count mismatch")
        reconstructed[mask] = color[mask]
        hit_count[mask] += 1
        rings = entry["轮廓"]
        for i, ring in enumerate(rings):
            start, count = ring["起始点"], ring["点数"]
            check(type(start) is int and type(count) is int and count > 0 and 0 <= start <= len(points) - count, "Invalid contour span")
            poly = points[start:start + count]
            used_points[start:start + count] += 1
            check(np.all((poly[:, 0] >= 0) & (poly[:, 0] < width) & (poly[:, 1] >= 0) & (poly[:, 1] < height)), "Contour out of frame")
            check(np.all(labels[poly[:, 1], poly[:, 0]] == cluster_id), "Contour point outside own cluster")
            check(np.all((poly[:, 2] >= 0) & (poly[:, 2] <= 7)), "Boundary reason invalid")
            check(np.max(np.abs(poly[:, :2] - np.roll(poly[:, :2], 1, axis=0))) <= 1, "Contour is not an ordered pixel chain")
            visited = {i}
            parent = ring["父轮廓索引"]
            nesting = 0
            while parent != -1:
                check(0 <= parent < len(rings) and parent not in visited, "Contour hierarchy cycle")
                visited.add(parent)
                nesting += 1
                parent = rings[parent]["父轮廓索引"]
            check(ring["内环"] == bool(nesting % 2), "Hole hierarchy mismatch")
            check(ring["物理孔洞确认"] is False, "Topology alone cannot confirm a physical hole")
    check(np.all(used_points == 1), "Unused or overlapping contour spans")
    residual = labels == 0
    reconstructed[residual] = color[residual]
    hit_count[residual] += 1
    check(np.all(hit_count == 1), "Missing or repeated pixel ownership")
    check(np.array_equal(reconstructed, color), "Package reconstruction failed")
    if expected_color is not None:
        check(np.array_equal(reconstructed, expected_color), "Source RGB reconstruction mismatch")
    if expected_depth is not None:
        check(np.array_equal(raw_depth, expected_depth), "Original depth was changed")

    calibration = manifest["标定"]
    ci, di = calibration["彩图内参"], calibration["深度内参"]
    check([raw_depth.shape[1], raw_depth.shape[0]] == [di["宽"], di["高"]], "Raw depth calibration mismatch")
    check([width, height] == [ci["宽"], ci["高"]], "Color calibration mismatch")
    geometry_checked = ci["畸变模型"] in (0, 2, 4) and di["畸变模型"] in (0, 2, 4)
    if geometry_checked:
        yy, xx = np.where(state != 0)
        indices = source_index[yy, xx]
        z = raw_depth.reshape(-1)[indices].astype(np.float64) * calibration["源深度单位米"]
        sx, sy = indices % di["宽"], indices // di["宽"]
        x0, y0 = (sx - di["主点X"]) / di["焦距X"], (sy - di["主点Y"]) / di["焦距Y"]
        x, y = x0.copy(), y0.copy()
        if di["畸变模型"] in (2, 4):
            k1, k2, p1, p2, k3 = di["畸变系数"]
            for _ in range(10):
                r2 = x * x + y * y
                inverse_radial = 1 / (1 + ((k3 * r2 + k2) * r2 + k1) * r2)
                xq, yq = (x / inverse_radial, y / inverse_radial) if di["畸变模型"] == 2 else (x, y)
                tx = 2 * p1 * xq * yq + p2 * (r2 + 2 * xq * xq)
                ty = 2 * p2 * xq * yq + p1 * (r2 + 2 * yq * yq)
                x, y = (x0 - tx) * inverse_radial, (y0 - ty) * inverse_radial
        points3d = np.stack((x * z, y * z, z), axis=1)
        ex = calibration["深度到彩图外参"]
        rotation = np.array(ex["旋转列优先"]).reshape((3, 3), order="F")
        projected = points3d @ rotation.T + np.array(ex["平移米"])
        check(np.allclose(depth[yy, xx], projected[:, 2], atol=2e-6, rtol=2e-6), "Output Z is not color-camera Z")
        if len(projected):
            x, y = projected[:, 0] / projected[:, 2], projected[:, 1] / projected[:, 2]
            if ci["畸变模型"] in (2, 4):
                k1, k2, p1, p2, k3 = ci["畸变系数"]
                r2 = x * x + y * y
                radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
                xf, yf = x * radial, y * radial
                tx, ty = (xf, yf) if ci["畸变模型"] == 2 else (x, y)
                x, y = xf + 2 * p1 * tx * ty + p2 * (r2 + 2 * tx * tx), yf + 2 * p2 * tx * ty + p1 * (r2 + 2 * ty * ty)
            uv = np.stack((x, y), axis=1) * [ci["焦距X"], ci["焦距Y"]] + [ci["主点X"], ci["主点Y"]]
            check(np.all(np.abs(uv - np.stack((xx, yy), axis=1)) <= 0.5001), "Invalid source-to-color projection")

    config = manifest["处理配置"]
    for y, x in zip(*np.where(fill_state == 1)):
        y0, y1, x0, x1 = max(0, y - 1), min(height, y + 2), max(0, x - 1), min(width, x + 2)
        local = (state[y0:y1, x0:x1] == 1) & (labels[y0:y1, x0:x1] == labels[y, x])
        measured = depth[y0:y1, x0:x1][local]
        check(len(measured) >= config["补全最少样本"], "Insufficient measured support")
        check(measured.min() - 1e-6 <= filled[y, x] <= measured.max() + 1e-6, "Filled depth outside measured neighbors")
    report = {"status": "pass", "pixels": width * height, "clusters": len(ids),
              "reconstruction_mismatch_pixels": 0, "missing_or_duplicate_pixels": 0,
              "observed_usable_pixels": int(np.count_nonzero(state == 1)),
              "interpolated_pixels": int(np.count_nonzero(fill_state)),
              "independent_reprojection_checked": geometry_checked,
              "physical_segmentation_accuracy": "not_established",
              "absolute_metric_accuracy": "not_established"}
    if output:
        output.mkdir(parents=True, exist_ok=True)
        Image.fromarray(reconstructed).save(output / "reconstructed.png")
        palette = np.stack(((labels * 67) % 251, (labels * 131) % 251, (labels * 197) % 251), axis=-1).astype(np.uint8)
        Image.fromarray(palette).save(output / "cluster_labels.png")
        (output / "validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("packet", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    print(json.dumps(validate_packet(args.packet, output=args.output), indent=2))


if __name__ == "__main__":
    main()
