"""Offline window-change probe. A low-change window is not a sensor accuracy certificate."""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image


THRESHOLDS = {"window_pairs": 15, "min_window_ms": 400, "max_window_ms": 1000,
              "min_valid_percent": 20, "valid_span_pp": 2, "max_tile_valid_span_pp": 5,
              "valid_flip_percent": 2, "depth_delta_p95_m": 0.03,
              "color_mean_abs": 3, "color_changed_percent": 2,
              "color_change_code": 20, "min_common_valid_pixels": 1000}


def read_png(root, desc):
    path = (root / desc["文件"]).resolve()
    if not path.is_relative_to(root.resolve()) or path.stat().st_size > 16 * 1024 * 1024:
        raise ValueError("Raw PNG path/size invalid")
    data = path.read_bytes()
    if len(data) != desc["字节数"] or hashlib.sha256(data).hexdigest() != desc["SHA256"]:
        raise ValueError("Raw PNG checksum mismatch")
    with Image.open(io.BytesIO(data)) as image:
        if image.width * image.height > 1280 * 720:
            raise ValueError("Raw PNG dimensions exceed source budget")
        return np.array(image)


def measure(color, depth, previous, scale):
    if color.ndim != 3 or color.shape[2] != 3 or color.dtype != np.uint8 or depth.ndim != 2 or depth.dtype.kind not in "ui" or min(depth.shape) < 4:
        raise ValueError("Invalid raw image format")
    valid = (depth > 0) & (depth < 65535)
    tiles = [100 * float(np.mean(tile)) for ys in np.array_split(valid, 4, axis=0) for tile in np.array_split(ys, 4, axis=1)]
    row = {"valid_percent": 100 * float(valid.mean()), "tile_valid_percent": tiles,
           "valid_flip_percent": None, "depth_delta_p50_m": None, "depth_delta_p95_m": None,
           "common_valid_pixels": 0, "color_mean_abs": None, "color_changed_percent": None,
           "depth_bands": {}, "tile_depth_delta_p95_m": []}
    if previous is not None:
        pc, pd = previous
        if pc.shape != color.shape or pd.shape != depth.shape:
            raise ValueError("Dimensions changed within session")
        pv = (pd > 0) & (pd < 65535)
        common = pv & valid
        row["valid_flip_percent"] = 100 * float(np.mean(valid != pv))
        row["common_valid_pixels"] = int(common.sum())
        if common.any():
            delta = np.abs(depth[common].astype(np.float64) - pd[common]) * scale
            row["depth_delta_p50_m"], row["depth_delta_p95_m"] = map(float, np.percentile(delta, [50, 95]))
        full_delta = np.abs(depth.astype(np.float64) - pd) * scale
        previous_m = pd.astype(np.float64) * scale
        for name, lo, hi in (("below_0.3m", 0, 0.3), ("0.3_to_1m", 0.3, 1), ("1_to_2m", 1, 2),
                             ("2_to_3.5m", 2, 3.5), ("above_3.5m", 3.5, float("inf"))):
            mask = common & (previous_m >= lo) & (previous_m < hi)
            values = full_delta[mask]
            row["depth_bands"][name] = {"common_pixels": int(mask.sum()),
                "delta_p50_m": float(np.median(values)) if len(values) else None,
                "delta_p95_m": float(np.percentile(values, 95)) if len(values) else None}
        for yy in np.array_split(np.arange(depth.shape[0]), 4):
            for xx in np.array_split(np.arange(depth.shape[1]), 4):
                block = np.ix_(yy, xx)
                values = full_delta[block][common[block]]
                row["tile_depth_delta_p95_m"].append(float(np.percentile(values, 95)) if len(values) else None)
        diff = np.abs(color.astype(np.int16) - pc.astype(np.int16))
        row["color_mean_abs"] = float(diff.mean())
        row["color_changed_percent"] = 100 * float(np.mean(diff.max(axis=2) > THRESHOLDS["color_change_code"]))
    return row


def decide(rows):
    n = THRESHOLDS["window_pairs"]
    if len(rows) < n + 1:
        return "积累中", ["窗口帧数不足"]
    window = rows[-n - 1:]
    pairs = window[1:]
    if any(not r["continuous"] for r in pairs):
        return "序列中断", ["缺口、拒绝事件、时间域变化或非递增时间"]
    span = window[-1]["color_timestamp_ms"] - window[0]["color_timestamp_ms"]
    if not THRESHOLDS["min_window_ms"] <= span <= THRESHOLDS["max_window_ms"]:
        return "证据不足", ["时间窗不在范围内"]
    if min(r["valid_percent"] for r in window) < THRESHOLDS["min_valid_percent"] or any(
            r["common_valid_pixels"] < THRESHOLDS["min_common_valid_pixels"] for r in pairs):
        return "证据不足", ["有效深度不足；稳定缺测不能作为稳定观测"]
    reasons = []
    if np.ptp([r["valid_percent"] for r in window]) > THRESHOLDS["valid_span_pp"]:
        reasons.append("全帧有效比例变化")
    if np.ptp([r["tile_valid_percent"] for r in window], axis=0).max() > THRESHOLDS["max_tile_valid_span_pp"]:
        reasons.append("局部有效比例变化")
    for key in ("valid_flip_percent", "depth_delta_p95_m", "color_mean_abs", "color_changed_percent"):
        if any(r[key] is None or not np.isfinite(r[key]) or r[key] > THRESHOLDS[key] for r in pairs):
            reasons.append(key)
    if any(r.get("exposure_changed", False) for r in pairs):
        reasons.append("曝光或增益元数据变化")
    return ("变化中", reasons) if reasons else ("变化较小候选", [])


def metadata_changed(source, previous):
    if previous is None:
        return False
    for stream in ("彩图", "深度"):
        for key in ("Actual Exposure", "Gain Level", "Auto Exposure"):
            a = source.get("逐流元数据", {}).get(stream, {}).get(key, {})
            b = previous.get("逐流元数据", {}).get(stream, {}).get(key, {})
            if a.get("支持") != b.get("支持") or a.get("原始值") != b.get("原始值"):
                return True
    return False


def analyze(path, output):
    data = path.read_bytes()
    report = json.loads(data)
    if report["格式"] != "PCS.RawStartupProbe/1":
        raise ValueError("Not a raw startup probe")
    output.mkdir(parents=True, exist_ok=False)
    summary = {"format": "PCS.StartupAnalysis/1", "probe_sha256": hashlib.sha256(data).hexdigest(),
               "thresholds": THRESHOLDS, "sensor_warmup_confirmed": False,
               "absolute_accuracy_confirmed": False, "physical_scene_static_confirmed": False,
               "sessions": []}
    all_rows = []
    for session in report["会话"]:
        previous = None
        last_source = None
        rows = []
        event_indices = {e["已接受帧数"] for e in session["异常事件"]}
        scale = session["标定与源状态"]["深度单位米"]
        if not np.isfinite(scale) or not 0 < scale <= 1:
            raise ValueError("Invalid depth scale")
        for item in session["帧列表"]:
            color = read_png(path.parent, item["彩图"])
            depth = read_png(path.parent, item["深度"])
            ci = session["标定与源状态"]["彩图内参"]
            di = session["标定与源状态"]["深度内参"]
            if color.shape[:2] != (ci["高"], ci["宽"]) or depth.shape != (di["高"], di["宽"]):
                raise ValueError("Raw dimensions do not match calibration")
            s = item["源信息"]
            row = measure(color, depth, previous, scale)
            row.update({"session": session["编号"], "index": item["序号"],
                        "color_timestamp_ms": s["彩图时间戳毫秒"], "source": s,
                        "received_after_open_ms": item["打开后接收毫秒"],
                        "continuous": False, "exposure_changed": metadata_changed(s, last_source)})
            if last_source:
                row["continuous"] = item["序号"] not in event_indices and all(
                    int(s[f"{name}源帧号"]) == int(last_source[f"{name}源帧号"]) + 1 and
                    s[f"{name}时间域"] == last_source[f"{name}时间域"] and
                    s[f"{name}时间戳毫秒"] > last_source[f"{name}时间戳毫秒"] for name in ("彩图", "深度"))
            rows.append(row)
            row["window_state"], row["window_reasons"] = decide(rows)
            previous, last_source = (color, depth), s
        candidates = [r for r in rows if r["window_state"] == "变化较小候选"]
        tail = rows[-30:]
        states = {state: sum(r["window_state"] == state for r in rows) for state in sorted({r["window_state"] for r in rows})}
        metadata_support = rows[0]["source"].get("逐流元数据", {}) if rows else {}
        bands = {}
        for name in (rows[-1]["depth_bands"] if rows else {}):
            values = [r["depth_bands"].get(name, {}).get("delta_p95_m") for r in tail]
            available = [v for v in values if v is not None]
            bands[name] = {"pair_p95_m_median": float(np.median(available)) if available else None,
                          "common_pixels_median": float(np.median([r["depth_bands"].get(name, {}).get("common_pixels", 0) for r in tail]))}
        summary["sessions"].append({"session": session["编号"], "frames": len(rows),
            "capture_events": session["异常事件"], "state_counts": states, "first_frame_metadata": metadata_support,
            "first_candidate_index": candidates[0]["index"] if candidates else None,
            "first_candidate_received_ms": candidates[0]["received_after_open_ms"] if candidates else None,
            "first_invalid_percent": 100 - rows[0]["valid_percent"] if rows else None,
            "tail_invalid_percent_p50": float(np.median([100 - r["valid_percent"] for r in tail])) if tail else None,
            "tail_depth_delta_p95_m_median": float(np.median([r["depth_delta_p95_m"] for r in tail if r["depth_delta_p95_m"] is not None])) if any(r["depth_delta_p95_m"] is not None for r in tail) else None,
            "tail_depth_bands": bands,
            "window_rejection_counts": {reason: sum(reason in r["window_reasons"] for r in rows) for reason in sorted({x for r in rows for x in r["window_reasons"]})},
            "depth_only_rejected_windows": sum(r["window_reasons"] == ["depth_delta_p95_m"] for r in rows),
            "nonconsecutive_pairs": sum(not r["continuous"] for r in rows[1:])})
        all_rows.extend(rows)
    (output / "frame_metrics.json").write_text(json.dumps(all_rows, ensure_ascii=False, indent=2), encoding="utf-8")
    fields = ["session", "index", "received_after_open_ms", "valid_percent", "valid_flip_percent",
              "depth_delta_p50_m", "depth_delta_p95_m", "color_mean_abs", "color_changed_percent", "continuous", "window_state"]
    with (output / "frame_metrics.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(all_rows)
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probe", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(analyze(args.probe, args.output), ensure_ascii=False, indent=2))
