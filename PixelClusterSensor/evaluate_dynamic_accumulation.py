"""Evaluate the frozen accumulator on legacy aligned RGBD recordings, without inventing raw metadata."""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image

from static_accumulator import CONFIG, StaticAccumulator, distribution


def read_case(root, nominal_fps=None):
    manifest_path = root / "case_manifest.json"
    manifest_bytes = manifest_path.read_bytes()
    manifest = json.loads(manifest_bytes)
    if manifest.get("format") != "d455_directory_replay_v1" or manifest.get("depth_unit") != "millimeter_uint16":
        raise ValueError("Expected legacy millimeter replay, not a processed AVI")
    count = manifest["frame_count"]
    if type(count) is not int or not 2 <= count <= 300:
        raise ValueError("Case must contain 2..300 frames")
    paths = sorted((root / "frames").glob("*_color.png"))
    if [p.name for p in paths] != [f"{i:06}_color.png" for i in range(count)]:
        raise ValueError("Frame count/index mismatch")
    stamps_path = root / "source_timestamps.csv"
    if stamps_path.exists():
        timestamp_data = stamps_path.read_bytes()
        stamps = list(csv.DictReader(io.StringIO(timestamp_data.decode("utf-8-sig"))))
        if len(stamps) != count or [int(r["sample_index"]) for r in stamps] != list(range(count)):
            raise ValueError("Timestamp rows do not match image indices")
        timestamps = [float(r["depth_timestamp_ms"]) for r in stamps]
        pairs = [float(r["color_timestamp_ms"]) for r in stamps]
        if not np.isfinite(timestamps + pairs).all() or any(abs(a - b) > 50 for a, b in zip(timestamps, pairs)):
            raise ValueError("Invalid or numerically unpaired timestamps")
        continuity = [False] + [all(int(stamps[i][k]) == int(stamps[i - 1][k]) + 1 for k in
            ("source_frameset_index", "color_frame_number", "depth_frame_number")) and
            all(0 < float(stamps[i][k]) - float(stamps[i - 1][k]) <= 50 for k in
                ("color_timestamp_ms", "depth_timestamp_ms")) for i in range(1, count)]
        timing = {"mode": "recorded_numeric_timestamps", "timestamp_domains_recorded": False,
                  "sha256": hashlib.sha256(timestamp_data).hexdigest(), "nominal_fps": None}
    else:
        if nominal_fps is None or not np.isfinite(nominal_fps) or not 20 <= nominal_fps <= 120:
            raise ValueError("Missing timestamps: explicit --nominal-fps is required for a conditional test")
        timestamps = [i * 1000 / nominal_fps for i in range(count)]
        continuity = [False] + [True] * (count - 1)
        timing = {"mode": "assumed_nominal_timeline", "nominal_fps": nominal_fps,
                  "timestamp_domains_recorded": False, "physical_age_verified": False}
    frames, inventory, total = [], [], 0
    digest = hashlib.sha256(manifest_bytes)
    for i in range(count):
        decoded = []
        for suffix in ("depth16", "color"):
            path = (root / "frames" / f"{i:06}_{suffix}.png").resolve()
            if not path.is_relative_to(root.resolve()) or path.stat().st_size > 16 * 1024**2:
                raise ValueError("Invalid image path/budget")
            data = path.read_bytes()
            with Image.open(io.BytesIO(data)) as image:
                if image.width * image.height > 921600:
                    raise ValueError("Image pixel budget exceeded")
                array = np.array(image)
            if suffix == "depth16":
                if array.ndim != 2 or array.dtype.kind not in "ui" or np.any(array > 65535):
                    raise ValueError("Invalid depth PNG")
                array = array.astype(np.uint16)
            elif array.ndim != 3 or array.shape[2] != 3 or array.dtype != np.uint8:
                raise ValueError("Invalid color PNG")
            sha = hashlib.sha256(data).hexdigest()
            inventory.append({"file": str(path.relative_to(root.resolve())), "bytes": len(data), "sha256": sha})
            digest.update(path.name.encode())
            digest.update(bytes.fromhex(sha))
            total += array.nbytes
            if total > 512 * 1024**2:
                raise ValueError("Preload budget exceeded")
            decoded.append(array)
        depth, color = decoded
        if list(depth.shape[::-1]) != manifest["depth_resolution"] or list(color.shape[1::-1]) != manifest["color_resolution"]:
            raise ValueError("Resolution differs from manifest")
        if depth.shape != color.shape[:2]:
            raise ValueError("This legacy diagnostic requires aligned color/depth dimensions")
        frames.append((depth, color))
    if stamps_path.exists():
        digest.update(timestamp_data)
    return frames, timestamps, continuity, {"case_id": manifest["case_id"], "frame_count": count,
        "manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(), "input_digest": digest.hexdigest(),
        "timing": timing, "inventory": inventory, "source_directory": str(root.resolve()),
        "source_gap_count": sum(not c for c in continuity[1:]),
        "coordinate_basis": "legacy color-aligned millimeter depth, based on capture/conversion implementation",
        "reviewed_field": manifest.get("reviewed"), "independent_object_ground_truth": False}


def inspect_output(arrays, frames, timestamps, index, previous):
    raw, color = frames[index]
    state, combined = arrays["evidence_state"], arrays["combined_depth16"]
    history = state == 2
    valid = (raw > 0) & (raw < 65535)
    current = state == 1
    if not np.array_equal(raw, arrays["raw_depth16"]) or not np.array_equal(color, arrays["color_rgb"]):
        raise AssertionError("Current input was changed")
    if not np.array_equal(combined[~history], raw[~history]) or np.any(raw[history] != 0):
        raise AssertionError("History overwrote current evidence")
    local_conflict = np.zeros(raw.shape, bool)
    sources = arrays["source_index"]
    for source in np.unique(sources[history]):
        if not 0 <= source < index:
            raise AssertionError("Invalid historical source")
        mask = history & (sources == source)
        old_depth, old_color = frames[int(source)]
        if not np.array_equal(combined[mask], old_depth[mask]):
            raise AssertionError("Historical value differs from source")
        np.testing.assert_allclose(arrays["last_observation_age_ms"][mask], timestamps[index] - timestamps[int(source)], atol=.001)
        local_conflict[mask] = np.max(np.abs(color[mask].astype(np.int16) - old_color[mask].astype(np.int16)), axis=1) > CONFIG["color_change_code"]
    age = arrays["last_observation_age_ms"][history]
    if age.size and (np.any(age <= 0) or np.any(age > CONFIG["history_max_age_ms"])):
        raise AssertionError("Historical age limit exceeded")
    row = {"index": index, "current_usable_percent": 100 * float(current.mean()),
           "historical_percent": 100 * float(history.mean()), "history_pixels": int(history.sum()),
           "combined_available_percent": 100 * float(np.mean(current | history)),
           "history_age_p95_ms": float(np.percentile(age, 95)) if age.size else None,
           "history_age_max_ms": float(age.max()) if age.size else None,
           "local_color_conflict_pixels": int(local_conflict.sum()),
           "reobserved_pixels": 0, "reobserved_conflict_pixels": 0, "reobserved_delta_p95_m": None,
           "raw_flip_percent": None, "combined_flip_percent": None}
    if previous is not None:
        old_state, old_combined = previous
        row["raw_flip_percent"] = 100 * float(np.mean(current != (old_state == 1)))
        row["combined_flip_percent"] = 100 * float(np.mean((current | history) != ((old_state == 1) | (old_state == 2))))
        reobserved = valid & (old_state == 2)
        row["reobserved_pixels"] = int(reobserved.sum())
        if reobserved.any():
            delta = np.abs(raw.astype(np.float32) - old_combined) * .001
            threshold = CONFIG["depth_change_absolute_m"] + CONFIG["depth_change_relative"] * np.minimum(raw, old_combined) * .001
            row["reobserved_conflict_pixels"] = int(np.count_nonzero(reobserved & (delta > threshold)))
            row["reobserved_delta_p95_m"] = float(np.percentile(delta[reobserved], 95))
    return row, local_conflict


def summarize(rows):
    total_history = sum(r["history_pixels"] for r in rows)
    total_reobserved = sum(r["reobserved_pixels"] for r in rows)
    totals = {key: sum(r[key] for r in rows) for key in ("history_pixels", "local_color_conflict_pixels", "reobserved_pixels", "reobserved_conflict_pixels")}
    return {"frames": len(rows), "frames_with_history": sum(r["history_pixels"] > 0 for r in rows),
        "frames_with_local_color_conflict": sum(r["local_color_conflict_pixels"] > 0 for r in rows),
        "reset_counts": {key: sum(r["reset_reason"] == key for r in rows) for key in ("startup", "none", "color_change", "source_discontinuity")},
        "totals": totals,
        "local_color_conflict_percent_of_history": 100 * totals["local_color_conflict_pixels"] / total_history if total_history else None,
        "reobserved_conflict_percent": 100 * totals["reobserved_conflict_pixels"] / total_reobserved if total_reobserved else None,
        "metrics": {key: distribution(r[key] for r in rows) for key in (
            "current_usable_percent", "historical_percent", "combined_available_percent", "raw_flip_percent", "combined_flip_percent",
            "history_age_p95_ms", "history_age_max_ms", "reobserved_delta_p95_m")}}


def evaluate(root, output, nominal_fps=None, repeats=3):
    if not 1 <= repeats <= 5:
        raise ValueError("Invalid repeat count")
    frames, timestamps, continuity, source = read_case(root, nominal_fps)
    output.mkdir(parents=True, exist_ok=False)
    (output / "source_inventory.json").write_text(json.dumps(source, indent=2), encoding="utf-8")
    snapshot = {"accumulator": CONFIG, "repeats": repeats, "timing": source["timing"],
                "scale_m": .001, "diagnostics_do_not_control_accumulator": True}
    config_data = (json.dumps(snapshot, sort_keys=True, indent=2) + "\n").encode()
    (output / "config_snapshot.json").write_bytes(config_data)
    report = {"format": "PCS.LegacyDynamicAccumulationAnalysis/1", "status": "running", "source": {k: v for k, v in source.items() if k != "inventory"},
        "config_sha256": hashlib.sha256(config_data).hexdigest(),
        "accumulator_sha256": hashlib.sha256(Path(__file__).with_name("static_accumulator.py").read_bytes()).hexdigest(),
        "runner_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "physical_motion_accuracy_verified": False, "promotion": False,
        "timing_scope": "Python update and return copies only, excludes PNG IO, inspection and exports",
        "timing": [], "source_value_age_checks": "not_completed"}
    rows, best, best_rank = [], None, (-1, -1)
    try:
        digests = []
        for repeat in range(repeats):
            accumulator = StaticAccumulator(.001)
            elapsed, digest, previous = [], hashlib.sha256(), None
            for index, (depth, color) in enumerate(frames):
                started = time.perf_counter()
                arrays, diag = accumulator.update(depth, color, timestamps[index], index, source_continuous=continuity[index])
                duration = 1000 * (time.perf_counter() - started)
                elapsed.append(duration)
                for key in sorted(arrays):
                    digest.update(arrays[key].tobytes())
                if repeat == 0:
                    row, mask = inspect_output(arrays, frames, timestamps, index, previous if continuity[index] else None)
                    row.update(diag)
                    row["update_ms"] = duration
                    rows.append(row)
                    previous = arrays["evidence_state"], arrays["combined_depth16"]
                    rank = (row["local_color_conflict_pixels"], row["history_pixels"])
                    if rank > best_rank:
                        best, best_rank = (index, arrays, mask), rank
            digests.append(digest.hexdigest())
            report["timing"].append({"repeat": repeat, "all_frames_ms": distribution(elapsed),
                                    "after_30_ms": distribution(elapsed[30:])})
        if len(set(digests)) != 1:
            raise AssertionError("Accumulation output is not deterministic across repeated playback")
        report["output_digest"] = digests[0]
        report["repeat_deterministic"] = True
        report["source_value_age_checks"] = "pass_all_frames"
        report["all_frames"] = summarize(rows)
        report["after_30"] = summarize(rows[30:])
        with (output / "frame_metrics.csv").open("w", newline="", encoding="utf-8-sig") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        index, arrays, mask = best
        np.savez_compressed(output / "risk_sample.npz", **arrays, local_color_conflict=mask)
        Image.fromarray(arrays["color_rgb"]).save(output / "source_color.png")
        palette = np.array([[0, 0, 0], [180, 180, 180], [0, 160, 210], [80, 80, 80], [220, 100, 0]], np.uint8)
        state_rgb = palette[arrays["evidence_state"]]
        state_rgb[mask] = [230, 30, 65]
        Image.fromarray(state_rgb).save(output / "evidence_state.png")
        report["sample"] = {"index": index, "local_color_conflict_pixels": best_rank[0], "history_pixels": best_rank[1],
                            "selection": "maximum local color conflict count, tie-break maximum history count; all frames eligible",
                            "palette": "black missing, light gray current, cyan history, dark gray out-of-range, orange saturation, red history with local color conflict"}
        report["status"] = "measured_with_source_limits"
    except Exception as exc:
        report["status"], report["error"] = "error", str(exc)
        raise
    finally:
        (output / "summary.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--nominal-fps", type=float)
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args()
    result = evaluate(args.case, args.output, args.nominal_fps, args.repeats)
    print(json.dumps({"status": result["status"], "case": result["source"]["case_id"], "after_30": result["after_30"], "timing": result["timing"]}, indent=2))
