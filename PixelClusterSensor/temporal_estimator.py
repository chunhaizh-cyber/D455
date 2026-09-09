"""Offline temporal estimators. Original samples stay authoritative; no production wiring."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import warnings

import numpy as np

from analyze_startup import read_png


CONFIG = {"format": "PCS.TemporalEstimatorConfig/1", "ignore_first_frames": 30,
          "cell_frames": 18, "half_frames": 9, "windows": [3, 5, 9],
          "methods": ["mean", "median", "guarded_median"], "support_fraction": 0.8,
          "near_m": 0.3, "far_m": 3.5, "span_absolute_m": 0.04, "span_relative": 0.02,
          "color_mean_abs": 3, "color_change_code": 20, "color_changed_percent": 2,
          "minimum_pair_pixels": 100, "max_source_step_ms": 50}
REASONS = {"current_missing": 1, "current_out_of_range": 2, "insufficient_samples": 4,
           "temporal_span": 8, "color_change": 16, "history_out_of_range": 32}
BANDS = {"all": (0.3, 3.5), "0.3_to_1m": (0.3, 1), "1_to_2m": (1, 2), "2_to_3.5m": (2, 3.5)}


def estimate(depth, timestamps, scale, method, *, color_changed=False):
    if method not in CONFIG["methods"] or depth.ndim != 3 or depth.shape[0] not in (3, 5, 9):
        raise ValueError("Unsupported estimator/window")
    t = np.asarray(timestamps, dtype=np.float64)
    if t.shape != (len(depth),) or not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0) or np.any(np.diff(t) > CONFIG["max_source_step_ms"]):
        raise ValueError("Estimator requires consecutive source times")
    if not np.isfinite(scale) or not 0 < scale <= 1 or depth.dtype.kind not in "ui" or np.any(depth > 65535):
        raise ValueError("Invalid raw depth")
    valid = (depth > 0) & (depth < 65535)
    z = depth.astype(np.float64) * scale
    count = valid.sum(axis=0).astype(np.uint8)
    values = np.where(valid, z, np.nan)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        lo, hi = np.nanmin(values, axis=0), np.nanmax(values, axis=0)
        result = np.nanmean(values, axis=0) if method == "mean" else np.nanmedian(values, axis=0)
    reason = np.zeros(depth.shape[1:], np.uint8)
    reason[~valid[-1]] |= REASONS["current_missing"]
    reason[valid[-1] & ((z[-1] < CONFIG["near_m"]) | (z[-1] >= CONFIG["far_m"]))] |= REASONS["current_out_of_range"]
    reason[count < int(np.ceil(CONFIG["support_fraction"] * len(depth)))] |= REASONS["insufficient_samples"]
    reason[np.any(valid & ((z < CONFIG["near_m"]) | (z >= CONFIG["far_m"])), axis=0)] |= REASONS["history_out_of_range"]
    if method == "guarded_median":
        reason[hi - lo > CONFIG["span_absolute_m"] + CONFIG["span_relative"] * lo] |= REASONS["temporal_span"]
        if color_changed:
            reason[:] |= REASONS["color_change"]
    accepted = (reason == 0) & np.isfinite(result)
    ages = (t[-1] - t)[:, None, None]
    oldest = np.max(np.where(valid, ages, 0), axis=0)
    average_age = np.divide(np.sum(np.where(valid, ages, 0), axis=0), count,
                            out=np.zeros_like(result), where=count > 0)
    return {"raw_current": depth[-1].copy(), "estimate_m": np.where(accepted, result, 0).astype(np.float32),
            "estimate_state": accepted.astype(np.uint8), "support_count": count,
            "oldest_sample_age_ms": np.where(accepted, oldest, 0).astype(np.float32),
            "mean_sample_age_ms": np.where(accepted, average_age, 0).astype(np.float32),
            "reject_reason": reason}


def boundary_mask(raw, scale):
    valid = (raw > 0) & (raw < 65535)
    z = raw.astype(np.float64) * scale
    boundary = np.zeros_like(valid)
    for a, b in ((np.s_[:-1, :], np.s_[1:, :]), (np.s_[:, :-1], np.s_[:, 1:])):
        jump = (valid[a] != valid[b]) | (valid[a] & valid[b] &
                (np.abs(z[a] - z[b]) > CONFIG["span_absolute_m"] + CONFIG["span_relative"] * np.minimum(z[a], z[b])))
        boundary[a] |= jump
        boundary[b] |= jump
    padded = np.pad(boundary, 1)
    return np.logical_or.reduce([padded[y:y + raw.shape[0], x:x + raw.shape[1]] for y in range(3) for x in range(3)])


def color_motion(frames):
    last = frames[-1].astype(np.int16)
    for earlier in frames[:-1]:
        delta = np.abs(earlier.astype(np.int16) - last)
        if delta.mean() > CONFIG["color_mean_abs"] or 100 * np.mean(delta.max(axis=2) > CONFIG["color_change_code"]) > CONFIG["color_changed_percent"]:
            return True
    return False


def cell_starts(length):
    return range(CONFIG["ignore_first_frames"], length - CONFIG["cell_frames"] + 1, CONFIG["cell_frames"])


def continuous(items, events):
    for i, row in enumerate(items):
        s = row["源信息"]
        if s["彩图时间域"] != s["深度时间域"] or abs(s["彩图时间戳毫秒"] - s["深度时间戳毫秒"]) > 50:
            return False
        if not i:
            continue
        p = items[i - 1]["源信息"]
        if row["序号"] in events:
            return False
        for name in ("彩图", "深度"):
            if (int(s[f"{name}源帧号"]) != int(p[f"{name}源帧号"]) + 1 or
                    s[f"{name}时间域"] != p[f"{name}时间域"] or
                    not 0 < s[f"{name}时间戳毫秒"] - p[f"{name}时间戳毫秒"] <= CONFIG["max_source_step_ms"]):
                return False
    return True


def evaluate(path, output):
    manifest_data = path.read_bytes()
    manifest = json.loads(manifest_data)
    if manifest["格式"] != "PCS.RawStartupProbe/1" or not manifest["会话"]:
        raise ValueError("Expected a nonempty raw startup probe")
    output.mkdir(parents=True, exist_ok=False)
    (output / "samples").mkdir()
    config_data = (json.dumps(CONFIG, sort_keys=True, indent=2) + "\n").encode()
    (output / "config_snapshot.json").write_bytes(config_data)
    report = {"format": "PCS.TemporalEstimatorAnalysis/1", "status": "running",
              "probe_sha256": hashlib.sha256(manifest_data).hexdigest(),
              "config_sha256": hashlib.sha256(config_data).hexdigest(),
              "code_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "production_promotion": False, "absolute_accuracy_verified": False,
              "raw_data_mutated": False, "sessions": [], "samples": [], "aggregate": []}
    rows = []
    try:
        for session in manifest["会话"]:
            frames = session["帧列表"]
            if len(frames) > 300:
                raise ValueError("Session exceeds raw probe frame budget")
            info = session["标定与源状态"]
            scale = info["深度单位米"]
            di, ci = info["深度内参"], info["彩图内参"]
            if not np.isfinite(scale) or not 0 < scale <= 1:
                raise ValueError("Invalid scale")
            events = {e["已接受帧数"] for e in session["异常事件"]}
            accepted_cells = skipped_cells = 0
            for start in cell_starts(len(frames)):
                items = frames[start:start + CONFIG["cell_frames"]]
                if not continuous(items, events):
                    skipped_cells += 1
                    continue
                depth = np.stack([read_png(path.parent, r["深度"]) for r in items])
                color = np.stack([read_png(path.parent, r["彩图"]) for r in items])
                if depth.shape[1:] != (di["高"], di["宽"]) or color.shape[1:] != (ci["高"], ci["宽"], 3):
                    raise ValueError("Image/calibration mismatch")
                timestamps = [r["源信息"]["深度时间戳毫秒"] for r in items]
                accepted_cells += 1
                # Identical output times and common raw support, independent of window/method selection.
                valid = (depth > 0) & (depth < 65535)
                fair = valid.all(axis=0)
                raw_a, raw_b = depth[8].astype(np.float64) * scale, depth[17].astype(np.float64) * scale
                edge = boundary_mask(depth[8], scale) | boundary_mask(depth[17], scale)
                for n in CONFIG["windows"]:
                    cuts = (slice(9 - n, 9), slice(18 - n, 18))
                    changed = [color_motion(color[cut]) for cut in cuts]
                    for method in CONFIG["methods"]:
                        estimates = [estimate(depth[cut], timestamps[cut], scale, method, color_changed=motion)
                                     for cut, motion in zip(cuts, changed)]
                        if accepted_cells == 1:
                            filename = f"session_{session['编号']}_{method}_{n}.npz"
                            sample_path = output / "samples" / filename
                            np.savez_compressed(sample_path, **estimates[-1])
                            report["samples"].append({"file": f"samples/{filename}", "sha256": hashlib.sha256(sample_path.read_bytes()).hexdigest(),
                                "sources": [r["源信息"] for r in items[cuts[-1]]], "window": n, "method": method})
                        a, b = estimates
                        kept = (a["estimate_state"] != 0) & (b["estimate_state"] != 0)
                        for band, (lo, hi) in BANDS.items():
                            base = fair & (raw_a >= lo) & (raw_a < hi)
                            for region, region_mask in (("all", np.ones_like(fair)), ("boundary", edge), ("interior", ~edge)):
                                eligible = base & region_mask
                                mask = eligible & kept
                                count = int(mask.sum())
                                row = {"session": session["编号"], "cell_start": start, "window": n, "method": method,
                                       "band": band, "region": region, "frame_pixels": fair.size,
                                       "fair_pixels": int(eligible.sum()), "evaluated_pixels": count,
                                       "endpoint_gap_ms": timestamps[-1] - timestamps[8],
                                       "color_veto_a": changed[0], "color_veto_b": changed[1],
                                       "raw_delta_p95_m": None, "estimate_delta_p95_m": None,
                                       "oldest_sample_age_ms_p95": None, "mean_sample_age_ms_p95": None}
                                for name, bit in REASONS.items():
                                    row[f"rejected_b_{name}"] = int(np.count_nonzero(b["reject_reason"] & bit))
                                row["accepted_current_b_pixels"] = int(np.count_nonzero(b["estimate_state"]))
                                if count >= CONFIG["minimum_pair_pixels"]:
                                    row["raw_delta_p95_m"] = float(np.percentile(np.abs(raw_b - raw_a)[mask], 95))
                                    row["estimate_delta_p95_m"] = float(np.percentile(np.abs(b["estimate_m"] - a["estimate_m"])[mask], 95))
                                    for key in ("oldest_sample_age_ms", "mean_sample_age_ms"):
                                        row[f"{key}_p95"] = float(np.percentile(b[key][mask], 95))
                                rows.append(row)
            report["sessions"].append({"session": session["编号"], "accepted_cells": accepted_cells,
                "skipped_nonconsecutive_cells": skipped_cells, "ignored_first_frames": min(len(frames), 30),
                "unused_tail_frames": max(0, len(frames) - 30) % 18})
        for n in CONFIG["windows"]:
            for method in CONFIG["methods"]:
                for band in BANDS:
                    for region in ("all", "boundary", "interior"):
                        group = [r for r in rows if (r["window"], r["method"], r["band"], r["region"]) == (n, method, band, region)]
                        good = [r for r in group if r["estimate_delta_p95_m"] is not None]
                        eligible = sum(r["fair_pixels"] for r in group)
                        entry = {"window": n, "method": method, "band": band, "region": region,
                                 "paired_cells": len(good), "eligible_pixel_observations": eligible,
                                 "fair_pixel_percent": 100 * eligible / sum(r["frame_pixels"] for r in group) if group else None,
                                 "retained_percent": 100 * sum(r["evaluated_pixels"] for r in group) / eligible if eligible else None}
                        for key in ("raw_delta_p95_m", "estimate_delta_p95_m", "oldest_sample_age_ms_p95", "mean_sample_age_ms_p95"):
                            entry[f"{key}_median"] = float(np.median([r[key] for r in good])) if good else None
                        report["aggregate"].append(entry)
        fields = list(rows[0]) if rows else ["session"]
        with (output / "paired_metrics.csv").open("w", newline="", encoding="utf-8-sig") as file:
            writer = csv.DictWriter(file, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        report["status"] = "measured" if rows else "no_evaluable_cells"
    except Exception as exc:
        report["status"] = "error"
        report["error"] = str(exc)
        raise
    finally:
        (output / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probe", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = evaluate(args.probe, args.output)
    print(json.dumps({"status": result["status"], "sessions": result["sessions"], "summary": str(args.output / "summary.json")}, indent=2))
