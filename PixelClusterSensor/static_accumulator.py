"""Bounded static-scene accumulation probe; never labels history as a current measurement."""
from __future__ import annotations

import argparse
from collections import deque
import csv
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from analyze_startup import metadata_changed, read_png
from temporal_estimator import continuous, estimate


CONFIG = {"format": "PCS.StaticAccumulationConfig/1", "near_m": 0.3, "far_m": 3.5,
          "history_max_age_ms": 200, "required_observations": 3,
          "depth_change_absolute_m": 0.04, "depth_change_relative": 0.02,
          "color_mean_abs": 3, "color_change_code": 20, "color_changed_percent": 2,
          "max_step_ms": 50, "ignore_first_frames": 30}
STATES = {"missing": 0, "current_usable": 1, "historical_candidate": 2,
          "current_out_of_range": 3, "current_saturated": 4}


def expand(mask):
    padded = np.pad(mask, 1)
    h, w = mask.shape
    return np.logical_or.reduce([padded[y:y + h, x:x + w] for y in range(3) for x in range(3)])


class StaticAccumulator:
    def __init__(self, scale):
        if not np.isfinite(scale) or not 0 < scale <= 1:
            raise ValueError("Invalid depth unit")
        self.scale = scale
        self.cache = None
        self.timestamp = None
        self.index = None

    def state_bytes(self):
        if self.cache is None:
            return 0
        return sum(a.nbytes for a in (self.cache, self.seen_ms, self.source_index,
                                      self.support, self.color_anchor))

    def update(self, depth, color, timestamp_ms, index, *, source_continuous=True):
        if (depth.ndim != 2 or depth.dtype != np.uint16 or depth.size > 921600 or
                min(depth.shape) < 2 or color.ndim != 3 or color.shape[2] != 3 or
                color.dtype != np.uint8 or color.size > 3 * 921600 or min(color.shape[:2]) < 2 or
                not np.isfinite(timestamp_ms) or not isinstance(index, int) or not 0 <= index < 2**31):
            raise ValueError("Invalid raw arrays, timestamp or source index")
        if self.cache is not None and (depth.shape != self.cache.shape or color.shape != self.color_anchor.shape):
            raise ValueError("Dimensions changed: create a new accumulator for the new session")
        reset = "startup" if self.cache is None else "none"
        color_mean = color_percent = 0.0
        if self.cache is None:
            self.cache = np.zeros_like(depth)
            self.seen_ms = np.zeros(depth.shape, np.float64)
            self.source_index = np.full(depth.shape, -1, np.int32)
            self.support = np.zeros(depth.shape, np.uint8)
            self.color_anchor = color.copy()
        else:
            if (not source_continuous or not 0 < timestamp_ms - self.timestamp <= CONFIG["max_step_ms"] or
                    index != self.index + 1):
                reset = "source_discontinuity"
            # Compare to the static epoch anchor, not just the immediately previous frame.
            # Raw color and depth are unregistered; color can veto globally, not per depth pixel.
            delta = np.abs(color.astype(np.int16) - self.color_anchor.astype(np.int16))
            color_mean = float(delta.mean())
            color_percent = 100 * float(np.mean(delta.max(axis=2) > CONFIG["color_change_code"]))
            if reset == "none" and (color_mean > CONFIG["color_mean_abs"] or
                                     color_percent > CONFIG["color_changed_percent"]):
                reset = "color_change"
        reset_pixels = int(np.count_nonzero(self.cache)) if reset != "none" else 0
        if reset != "none":
            self.cache.fill(0)
            self.support.fill(0)
            self.color_anchor = color.copy()
        raw_valid = (depth > 0) & (depth < 65535)
        z = depth.astype(np.float32) * self.scale
        usable = raw_valid & (z >= CONFIG["near_m"]) & (z < CONFIG["far_m"])
        expired = (self.cache != 0) & (timestamp_ms - self.seen_ms > CONFIG["history_max_age_ms"])
        self.cache[expired] = 0
        self.support[expired] = 0
        old_z = self.cache.astype(np.float32) * self.scale
        conflict = ((self.cache != 0) & raw_valid &
                    (np.abs(z - old_z) > CONFIG["depth_change_absolute_m"] +
                     CONFIG["depth_change_relative"] * np.minimum(z, old_z)))
        # A changed boundary also invalidates neighboring holes before current samples seed new state.
        invalidated = expand(conflict) | (depth == 65535) | (raw_valid & ~usable)
        invalidated_pixels = int(np.count_nonzero(invalidated & (self.cache != 0)))
        self.cache[invalidated] = 0
        self.support[invalidated] = 0
        self.cache[usable] = depth[usable]
        self.seen_ms[usable] = timestamp_ms
        self.source_index[usable] = index
        self.support[usable] = np.minimum(self.support[usable] + 1, CONFIG["required_observations"])
        history = ((depth == 0) & (self.cache != 0) &
                   (self.support >= CONFIG["required_observations"]))
        combined = depth.copy()
        combined[history] = self.cache[history]
        state = np.zeros(depth.shape, np.uint8)
        state[usable] = STATES["current_usable"]
        state[history] = STATES["historical_candidate"]
        state[raw_valid & ~usable] = STATES["current_out_of_range"]
        state[depth == 65535] = STATES["current_saturated"]
        age = np.zeros(depth.shape, np.float32)
        age[history] = timestamp_ms - self.seen_ms[history]
        sources = np.full(depth.shape, -1, np.int32)
        sources[raw_valid | (depth == 65535)] = index
        sources[history] = self.source_index[history]
        self.timestamp, self.index = timestamp_ms, index
        return {"raw_depth16": depth.copy(), "color_rgb": color.copy(), "combined_depth16": combined,
                "evidence_state": state, "last_observation_age_ms": age, "source_index": sources,
                "support_count": self.support.copy()}, {
                    "reset_reason": reset, "reset_pixels": reset_pixels,
                    "expired_pixels": int(expired.sum()), "depth_conflict_pixels": int(conflict.sum()),
                    "invalidated_pixels": invalidated_pixels, "color_anchor_mean_abs": color_mean,
                    "color_anchor_changed_percent": color_percent, "persistent_state_bytes": self.state_bytes()}


def distribution(values):
    values = [float(v) for v in values if v is not None]
    if not values:
        return {"count": 0, "p50": None, "p95": None, "max": None}
    return {"count": len(values), "p50": float(np.median(values)),
            "p95": float(np.percentile(values, 95)), "max": max(values)}


def evaluate(path, output, repeats=3, save_every_n=30):
    if not 1 <= repeats <= 5 or save_every_n < 1:
        raise ValueError("Invalid repeat/export budget")
    data = path.read_bytes()
    manifest = json.loads(data)
    if manifest["格式"] != "PCS.RawStartupProbe/1" or not 1 <= len(manifest["会话"]) <= 5:
        raise ValueError("Expected bounded raw startup probe")
    output.mkdir(parents=True, exist_ok=False)
    (output / "samples").mkdir()
    snapshot = {"config": CONFIG, "repeats": repeats, "save_every_n": save_every_n,
                "reference": "temporal_estimator.estimate mean3, same endpoint; no quality ranking"}
    config_data = (json.dumps(snapshot, sort_keys=True, indent=2) + "\n").encode()
    (output / "config_snapshot.json").write_bytes(config_data)
    report = {"format": "PCS.StaticAccumulationAnalysis/1", "status": "running",
              "probe_sha256": hashlib.sha256(data).hexdigest(),
              "config_sha256": hashlib.sha256(config_data).hexdigest(),
              "code_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "reference_code_sha256": hashlib.sha256(Path(__file__).with_name("temporal_estimator.py").read_bytes()).hexdigest(),
              "promotion": False, "absolute_accuracy_verified": False, "sessions": [], "samples": [],
              "states": STATES, "coordinate_system": "raw depth image; current color remains in its own image coordinates",
              "timing_scope": "sequential Python update including gates and output copies; excludes PNG IO/hash, source manifest checks, metrics and export; not C++ frame latency"}
    rows = []
    try:
        for session in manifest["会话"]:
            items = session["帧列表"]
            if not 2 <= len(items) <= 300:
                raise ValueError("Invalid raw session frame budget")
            info = session["标定与源状态"]
            scale = info["深度单位米"]
            if type(session["编号"]) is not int or session["编号"] < 0:
                raise ValueError("Invalid session identifier")
            ci, di = info["彩图内参"], info["深度内参"]
            events = {e["已接受帧数"] for e in session["异常事件"]}
            decoded = []
            # Preload only the bounded benchmark session; the accumulator itself stores no frame queue.
            for item in items:
                depth, color = read_png(path.parent, item["深度"]), read_png(path.parent, item["彩图"])
                if depth.shape != (di["高"], di["宽"]) or color.shape != (ci["高"], ci["宽"], 3):
                    raise ValueError("Image/calibration mismatch")
                if depth.dtype.kind not in "ui" or np.any(depth > 65535):
                    raise ValueError("Invalid raw depth codes")
                decoded.append((depth.astype(np.uint16), color))
                if sum(d.nbytes + c.nbytes for d, c in decoded) > 512 * 1024**2:
                    raise ValueError("Benchmark source buffer budget exceeded")
            checksum = hashlib.sha256()
            for d, c in decoded:
                checksum.update(d.tobytes())
                checksum.update(c.tobytes())
            before = checksum.hexdigest()
            # Alternating whole sequential passes avoids concurrent resource competition and fixed order bias.
            for repeat in range(repeats):
                pass_rows = [{} for _ in items]
                methods = ("accumulator", "mean3") if repeat % 2 == 0 else ("mean3", "accumulator")
                for method in methods:
                    accumulator = StaticAccumulator(scale)
                    history = deque(maxlen=3)
                    times = deque(maxlen=3)
                    previous = None
                    for index, (item, (depth, color)) in enumerate(zip(items, decoded)):
                        source = item["源信息"]
                        if (not all(np.isfinite(source[f"{s}时间戳毫秒"]) for s in ("彩图", "深度")) or
                                not continuous([item], set())):
                            raise ValueError("Unpaired source frame")
                        contiguous = (index > 0 and continuous(items[index - 1:index + 1], events) and
                                      not metadata_changed(source, items[index - 1]["源信息"]))
                        timestamp = source["深度时间戳毫秒"]
                        row = pass_rows[index]
                        if method == "mean3":
                            if not contiguous:
                                history.clear()
                                times.clear()
                            history.append(depth)
                            times.append(timestamp)
                            row["mean3_update_ms"] = None
                            if len(history) == 3:
                                start = time.perf_counter()
                                estimate(np.stack(history), list(times), scale, "mean")
                                row["mean3_update_ms"] = 1000 * (time.perf_counter() - start)
                            continue
                        start = time.perf_counter()
                        arrays, diagnostics = accumulator.update(depth, color, timestamp, index, source_continuous=contiguous)
                        duration = 1000 * (time.perf_counter() - start)
                        state = arrays["evidence_state"]
                        current, retained = state == 1, state == 2
                        output_available = current | retained
                        row.update({"session": session["编号"], "repeat": repeat, "index": index,
                            "accumulator_update_ms": duration, "current_usable_percent": 100 * float(current.mean()),
                            "historical_candidate_percent": 100 * float(retained.mean()),
                            "combined_available_percent": 100 * float(output_available.mean()),
                            "history_age_ms_p95": float(np.percentile(arrays["last_observation_age_ms"][retained], 95)) if retained.any() else None,
                            "raw_availability_flip_percent": None, "combined_availability_flip_percent": None,
                            "current_raw_delta_p95_m": None, "current_output_delta_p95_m": None,
                            "reobserved_history_pixels": 0, "reobserved_history_delta_p95_m": None,
                            **diagnostics})
                        if previous is not None and contiguous:
                            pd, pa = previous
                            old_state = pa["evidence_state"]
                            row["raw_availability_flip_percent"] = 100 * float(np.mean(current != (old_state == 1)))
                            row["combined_availability_flip_percent"] = 100 * float(np.mean(output_available != ((old_state == 1) | (old_state == 2))))
                            common = current & (old_state == 1)
                            if common.any():
                                row["current_raw_delta_p95_m"] = float(np.percentile(np.abs(depth.astype(np.float32) - pd)[common] * scale, 95))
                                row["current_output_delta_p95_m"] = float(np.percentile(np.abs(arrays["combined_depth16"].astype(np.float32) - pa["combined_depth16"])[common] * scale, 95))
                            reobserved = current & (old_state == 2)
                            row["reobserved_history_pixels"] = int(reobserved.sum())
                            if reobserved.any():
                                row["reobserved_history_delta_p95_m"] = float(np.percentile(np.abs(depth.astype(np.float32) - pa["combined_depth16"])[reobserved] * scale, 95))
                        previous = (depth, arrays)
                        if repeat == 0 and (index % save_every_n == 0 or index == len(items) - 1):
                            filename = f"session_{session['编号']}_frame_{index}.npz"
                            sample_path = output / "samples" / filename
                            np.savez_compressed(sample_path, **arrays)
                            report["samples"].append({"file": f"samples/{filename}", "session": session["编号"], "index": index,
                                "depth_unit_m": scale, "history_source_lookup": "source_index indexes this session's original frame list",
                                "source": source, "sha256": hashlib.sha256(sample_path.read_bytes()).hexdigest()})
                rows.extend(pass_rows)
            checksum = hashlib.sha256()
            for d, c in decoded:
                checksum.update(d.tobytes())
                checksum.update(c.tobytes())
            if checksum.hexdigest() != before:
                raise AssertionError("Raw input was mutated")
            report["sessions"].append({"session": session["编号"], "frames": len(items), "raw_array_hash_unchanged": True})
        with (output / "frame_metrics.csv").open("w", newline="", encoding="utf-8-sig") as file:
            fields = sorted(set().union(*(r.keys() for r in rows)))
            writer = csv.DictWriter(file, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        evaluated = [r for r in rows if r["index"] >= CONFIG["ignore_first_frames"]]
        first = [r for r in evaluated if r["repeat"] == 0]
        quality_fields = ("current_usable_percent", "historical_candidate_percent", "combined_available_percent",
                          "history_age_ms_p95", "raw_availability_flip_percent", "combined_availability_flip_percent",
                          "current_raw_delta_p95_m", "current_output_delta_p95_m", "reobserved_history_delta_p95_m")
        report["quality"] = {key: distribution(r[key] for r in first) for key in quality_fields}
        report["timing"] = []
        for repeat in range(repeats):
            paired = [r for r in evaluated if r["repeat"] == repeat and r["mean3_update_ms"] is not None]
            report["timing"].append({"repeat": repeat, **{key: distribution(r[key] for r in paired)
                                    for key in ("accumulator_update_ms", "mean3_update_ms")}})
        report["reset_counts"] = {key: sum(r["reset_reason"] == key for r in first)
                                  for key in ("none", "startup", "source_discontinuity", "color_change")}
        report["persistent_state_bytes_max"] = max(r["persistent_state_bytes"] for r in rows)
        report["status"] = "measured" if first else "no_evaluable_frames"
    except Exception as exc:
        report["status"], report["error"] = "error", str(exc)
        raise
    finally:
        (output / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probe", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--save-every-n", type=int, default=30)
    args = parser.parse_args()
    result = evaluate(args.probe, args.output, args.repeats, args.save_every_n)
    print(json.dumps({"status": result["status"], "quality": result.get("quality"), "timing": result.get("timing")}, indent=2))
