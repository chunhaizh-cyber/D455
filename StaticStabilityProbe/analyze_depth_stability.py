#!/usr/bin/env python3
import argparse
import csv
import json
import math
import warnings
from pathlib import Path

import numpy as np
from PIL import Image


WINDOW_SIZES = (1, 2, 3, 4, 5, 8, 16, 32)


def finite_percentile(values, percentile):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return 0.0
    return float(np.percentile(values, percentile))


def safe_nan_stat(function, values, axis=0):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        return function(values, axis=axis)


def save_u8_map(path, values, minimum, maximum):
    clipped = np.clip((values - minimum) / max(maximum - minimum, 1e-6), 0.0, 1.0)
    Image.fromarray(np.rint(clipped * 255.0).astype(np.uint8)).save(path)


def save_u16_map(path, values):
    Image.fromarray(np.clip(np.rint(values), 0, 65535).astype(np.uint16)).save(path)


def load_depth_stack(frames_dir, output_dir, frame_count):
    depth_paths = sorted(frames_dir.glob("*_depth16.png"))
    if not depth_paths:
        raise RuntimeError(f"no depth16 frames found in {frames_dir}")
    if frame_count > 0:
        depth_paths = depth_paths[:frame_count]
    first = np.asarray(Image.open(depth_paths[0]), dtype=np.uint16)
    if first.ndim != 2:
        raise RuntimeError("depth image must be single channel")
    stack_path = output_dir / "depth_stack.npy"
    stack = np.lib.format.open_memmap(
        stack_path,
        mode="w+",
        dtype=np.uint16,
        shape=(len(depth_paths), first.shape[0], first.shape[1]),
    )
    stack[0] = first
    for index, path in enumerate(depth_paths[1:], start=1):
        image = np.asarray(Image.open(path), dtype=np.uint16)
        if image.shape != first.shape:
            raise RuntimeError(f"shape mismatch in {path}")
        stack[index] = image
    stack.flush()
    del stack
    return np.load(stack_path, mmap_mode="r"), stack_path


def compute_distribution_maps(stack, tile_rows, reference_start, saturated_depth_value):
    frame_count, height, width = stack.shape
    valid_count = np.zeros((height, width), dtype=np.uint16)
    median = np.zeros((height, width), dtype=np.float32)
    mad = np.zeros((height, width), dtype=np.float32)
    standard_deviation = np.zeros((height, width), dtype=np.float32)
    p10 = np.zeros((height, width), dtype=np.float32)
    p90 = np.zeros((height, width), dtype=np.float32)
    reference_median = np.zeros((height, width), dtype=np.float32)
    reference_valid_count = np.zeros((height, width), dtype=np.uint16)

    for row_start in range(0, height, tile_rows):
        row_end = min(height, row_start + tile_rows)
        block = np.asarray(stack[:, row_start:row_end, :], dtype=np.float32)
        valid = (block > 0) & (block < saturated_depth_value)
        valid_count[row_start:row_end] = np.sum(valid, axis=0, dtype=np.uint16)
        block[~valid] = np.nan
        percentiles = safe_nan_stat(
            lambda value, axis: np.nanpercentile(value, (10, 50, 90), axis=axis),
            block,
        )
        p10[row_start:row_end] = np.nan_to_num(percentiles[0], nan=0.0)
        median_tile = np.nan_to_num(percentiles[1], nan=0.0)
        median[row_start:row_end] = median_tile
        p90[row_start:row_end] = np.nan_to_num(percentiles[2], nan=0.0)
        standard_deviation[row_start:row_end] = np.nan_to_num(
            safe_nan_stat(np.nanstd, block), nan=0.0
        )
        absolute_deviation = np.abs(block - median_tile[None, :, :])
        mad[row_start:row_end] = np.nan_to_num(
            safe_nan_stat(np.nanmedian, absolute_deviation), nan=0.0
        )

        reference = block[reference_start:]
        reference_valid_count[row_start:row_end] = np.sum(
            np.isfinite(reference), axis=0, dtype=np.uint16
        )
        reference_median[row_start:row_end] = np.nan_to_num(
            safe_nan_stat(np.nanmedian, reference), nan=0.0
        )

    return {
        "valid_count": valid_count,
        "valid_percent": valid_count.astype(np.float32) * 100.0 / frame_count,
        "median": median,
        "mad": mad,
        "standard_deviation": standard_deviation,
        "p10": p10,
        "p90": p90,
        "temporal_spread": p90 - p10,
        "reference_median": reference_median,
        "reference_valid_count": reference_valid_count,
    }


def compute_depth_edge_mask(median_depth, threshold_mm):
    valid = median_depth > 0
    edge = np.zeros(median_depth.shape, dtype=bool)
    horizontal = (
        valid[:, 1:]
        & valid[:, :-1]
        & (np.abs(median_depth[:, 1:] - median_depth[:, :-1]) > threshold_mm)
    )
    vertical = (
        valid[1:, :]
        & valid[:-1, :]
        & (np.abs(median_depth[1:, :] - median_depth[:-1, :]) > threshold_mm)
    )
    edge[:, 1:] |= horizontal
    edge[:, :-1] |= horizontal
    edge[1:, :] |= vertical
    edge[:-1, :] |= vertical
    return edge


def compute_hole_runs(stack, saturated_depth_value):
    frame_count, height, width = stack.shape
    current = np.zeros(height * width, dtype=np.uint16)
    maximum = np.zeros(height * width, dtype=np.uint16)
    histogram = np.zeros(frame_count + 1, dtype=np.int64)
    for frame_index in range(frame_count):
        values = np.asarray(stack[frame_index]).reshape(-1)
        invalid = (values == 0) | (values >= saturated_depth_value)
        current[invalid] += 1
        ended = (~invalid) & (current > 0)
        if np.any(ended):
            lengths = current[ended]
            maximum[ended] = np.maximum(maximum[ended], lengths)
            histogram[: len(np.bincount(lengths, minlength=frame_count + 1))] += np.bincount(
                lengths, minlength=frame_count + 1
            )
            current[ended] = 0
    ongoing = current > 0
    maximum[ongoing] = np.maximum(maximum[ongoing], current[ongoing])
    return maximum.reshape(height, width), histogram, int(np.count_nonzero(ongoing))


def region_row(name, mask, maps):
    count = int(np.count_nonzero(mask))
    total = mask.size
    return {
        "region": name,
        "pixel_count": count,
        "pixel_percent": 100.0 * count / total,
        "valid_percent_p50": finite_percentile(maps["valid_percent"][mask], 50),
        "valid_percent_p95": finite_percentile(maps["valid_percent"][mask], 95),
        "median_depth_mm_p50": finite_percentile(maps["median"][mask], 50),
        "temporal_std_mm_p50": finite_percentile(maps["standard_deviation"][mask], 50),
        "temporal_std_mm_p95": finite_percentile(maps["standard_deviation"][mask], 95),
        "mad_mm_p50": finite_percentile(maps["mad"][mask], 50),
        "mad_mm_p95": finite_percentile(maps["mad"][mask], 95),
        "p90_p10_spread_mm_p50": finite_percentile(maps["temporal_spread"][mask], 50),
        "p90_p10_spread_mm_p95": finite_percentile(maps["temporal_spread"][mask], 95),
        "max_hole_run_p50": finite_percentile(maps["max_hole_run"][mask], 50),
        "max_hole_run_p95": finite_percentile(maps["max_hole_run"][mask], 95),
    }


def estimate_window(block, method, saturated_depth_value):
    values = block.astype(np.float32)
    values[(values <= 0) | (values >= saturated_depth_value)] = np.nan
    support = np.sum(np.isfinite(values), axis=0)
    if method == "mean":
        estimate = np.divide(
            np.nansum(values, axis=0),
            support,
            out=np.zeros(support.shape, dtype=np.float32),
            where=support > 0,
        )
    else:
        median = np.nan_to_num(safe_nan_stat(np.nanmedian, values), nan=0.0)
        if method == "median":
            estimate = median
        elif method == "mad_filtered_mean":
            absolute_deviation = np.abs(values - median[None, :, :])
            mad = np.nan_to_num(safe_nan_stat(np.nanmedian, absolute_deviation), nan=0.0)
            threshold = np.maximum(20.0, 3.0 * 1.4826 * mad)
            filtered = np.where(absolute_deviation <= threshold[None, :, :], values, np.nan)
            filtered_support = np.sum(np.isfinite(filtered), axis=0)
            estimate = np.divide(
                np.nansum(filtered, axis=0),
                filtered_support,
                out=np.zeros(filtered_support.shape, dtype=np.float32),
                where=filtered_support > 0,
            )
            support = filtered_support
        else:
            raise ValueError(method)
    return estimate, support


def convergence_rows(
    stack,
    maps,
    interior_mask,
    intermittent_mask,
    edge_jump_mask,
    reference_start,
    samples,
    saturated_depth_value,
):
    reference_count = stack.shape[0] - reference_start
    reliable_reference = (
        (maps["reference_median"] > 0)
        & (maps["reference_valid_count"] >= math.ceil(reference_count * 0.9))
    )
    intermittent_reference = (
        intermittent_mask
        & (maps["reference_median"] > 0)
        & (maps["reference_valid_count"] >= math.ceil(reference_count * 0.1))
    )
    edge_jump_reference = (
        edge_jump_mask
        & (maps["reference_median"] > 0)
        & (maps["reference_valid_count"] >= math.ceil(reference_count * 0.1))
    )
    scopes = {
        "reliable_pixels": (reliable_reference, 0.5),
        "persistent_valid_interior": (reliable_reference & interior_mask, 0.5),
        "intermittent_any_valid": (intermittent_reference, 0.0),
        "intermittent_majority_valid": (intermittent_reference, 0.5),
        "edge_jump_any_valid": (edge_jump_reference, 0.0),
        "edge_jump_majority_valid": (edge_jump_reference, 0.5),
    }
    rows = []
    candidate_frame_count = reference_start
    for window_size in WINDOW_SIZES:
        maximum_start = max(0, candidate_frame_count - window_size)
        starts = np.unique(np.linspace(0, maximum_start, num=samples, dtype=int))
        for method in ("mean", "median", "mad_filtered_mean"):
            per_scope = {
                name: {"p50": [], "p95": [], "coverage": [], "minimum_support": 0}
                for name in scopes
            }
            for start in starts:
                block = np.asarray(stack[start : start + window_size])
                estimate, support = estimate_window(block, method, saturated_depth_value)
                for scope_name, (scope_mask, support_fraction) in scopes.items():
                    minimum_support = max(1, math.ceil(window_size * support_fraction))
                    per_scope[scope_name]["minimum_support"] = minimum_support
                    usable = scope_mask & (support >= minimum_support) & (estimate > 0)
                    scope_count = max(1, int(np.count_nonzero(scope_mask)))
                    error = np.abs(estimate[usable] - maps["reference_median"][usable])
                    per_scope[scope_name]["p50"].append(finite_percentile(error, 50))
                    per_scope[scope_name]["p95"].append(finite_percentile(error, 95))
                    per_scope[scope_name]["coverage"].append(
                        100.0 * np.count_nonzero(usable) / scope_count
                    )
            for scope_name, values in per_scope.items():
                rows.append(
                    {
                        "scope": scope_name,
                        "method": method,
                        "window_frames": window_size,
                        "sample_window_count": len(starts),
                        "minimum_support_frames": values["minimum_support"],
                        "coverage_percent_mean": float(np.mean(values["coverage"])),
                        "error_mm_p50_mean": float(np.mean(values["p50"])),
                        "error_mm_p95_mean": float(np.mean(values["p95"])),
                    }
                )
    return rows


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description="Analyze static per-pixel D455 depth stability")
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--reference-start", type=int, default=300)
    parser.add_argument("--tile-rows", type=int, default=40)
    parser.add_argument("--edge-threshold-mm", type=float, default=50.0)
    parser.add_argument("--convergence-samples", type=int, default=4)
    parser.add_argument("--saturated-depth-value", type=int, default=65535)
    args = parser.parse_args()

    dataset = Path(args.dataset)
    output_dir = Path(args.out_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stack, stack_path = load_depth_stack(dataset / "frames", output_dir, args.frames)
    if args.reference_start <= 0 or args.reference_start >= stack.shape[0]:
        raise RuntimeError("--reference-start must split the captured sequence")

    maps = compute_distribution_maps(
        stack, args.tile_rows, args.reference_start, args.saturated_depth_value
    )
    edge_mask = compute_depth_edge_mask(maps["median"], args.edge_threshold_mm)
    max_hole_run, hole_histogram, ongoing_invalid_pixels = compute_hole_runs(
        stack, args.saturated_depth_value
    )
    maps["max_hole_run"] = max_hole_run

    persistent_valid = maps["valid_percent"] >= 95.0
    intermittent = (maps["valid_percent"] > 5.0) & (maps["valid_percent"] < 95.0)
    persistent_invalid = maps["valid_percent"] <= 5.0
    interior = persistent_valid & ~edge_mask
    edge_jump_candidate = edge_mask & (maps["temporal_spread"] > 100.0)

    regions = {
        "all_pixels": np.ones(maps["median"].shape, dtype=bool),
        "persistent_valid_interior": interior,
        "persistent_valid_depth_edge": persistent_valid & edge_mask,
        "intermittent_depth": intermittent,
        "persistent_invalid": persistent_invalid,
        "edge_jump_candidate": edge_jump_candidate,
    }
    for lower, upper in ((0, 1000), (1000, 2000), (2000, 3000), (3000, 5000), (5000, 65536)):
        regions[f"interior_{lower}_{upper}mm"] = (
            interior & (maps["median"] >= lower) & (maps["median"] < upper)
        )
    region_rows = [region_row(name, mask, maps) for name, mask in regions.items()]
    write_csv(output_dir / "spatial_region_metrics.csv", region_rows)

    convergence = convergence_rows(
        stack,
        maps,
        interior,
        intermittent,
        edge_jump_candidate,
        args.reference_start,
        max(1, args.convergence_samples),
        args.saturated_depth_value,
    )
    write_csv(output_dir / "window_convergence.csv", convergence)

    recovered_runs = int(np.sum(hole_histogram[1:]))
    hole_rows = []
    for length, count in enumerate(hole_histogram):
        if length == 0 or count == 0:
            continue
        hole_rows.append(
            {
                "invalid_run_frames": length,
                "recovered_run_count": int(count),
                "recovered_run_percent": 100.0 * int(count) / max(1, recovered_runs),
            }
        )
    write_csv(output_dir / "hole_run_lengths.csv", hole_rows)

    save_u8_map(output_dir / "valid_percent_map.png", maps["valid_percent"], 0.0, 100.0)
    save_u16_map(output_dir / "median_depth_mm_map.png", maps["median"])
    save_u8_map(output_dir / "mad_0_100mm_map.png", maps["mad"], 0.0, 100.0)
    save_u8_map(output_dir / "temporal_std_0_200mm_map.png", maps["standard_deviation"], 0.0, 200.0)
    save_u8_map(output_dir / "max_hole_run_0_60_map.png", max_hole_run, 0.0, 60.0)
    Image.fromarray(edge_mask.astype(np.uint8) * 255).save(
        output_dir / "depth_edge_mask.png"
    )
    Image.fromarray(edge_jump_candidate.astype(np.uint8) * 255).save(
        output_dir / "edge_jump_candidate_mask.png"
    )

    hole_recovery = {}
    for frames in (1, 2, 3, 5, 10):
        hole_recovery[f"within_{frames}_frames_percent"] = (
            100.0 * int(np.sum(hole_histogram[1 : frames + 1])) / max(1, recovered_runs)
        )
    summary = {
        "schema_version": 1,
        "dataset": str(dataset),
        "frame_count": int(stack.shape[0]),
        "height": int(stack.shape[1]),
        "width": int(stack.shape[2]),
        "reference_frames": {
            "candidate_start": 0,
            "candidate_end_exclusive": args.reference_start,
            "reference_start": args.reference_start,
            "reference_end_exclusive": int(stack.shape[0]),
            "note": "The second-half median is a repeatability reference, not ground truth accuracy.",
        },
        "thresholds": {
            "persistent_valid_percent": 95.0,
            "persistent_invalid_percent": 5.0,
            "depth_edge_threshold_mm": args.edge_threshold_mm,
            "edge_jump_temporal_spread_mm": 100.0,
            "mad_filter_minimum_mm": 20.0,
            "mad_filter_multiplier": 3.0 * 1.4826,
            "saturated_depth_value": args.saturated_depth_value,
        },
        "pixel_classes": {
            name: {
                "pixel_count": int(np.count_nonzero(mask)),
                "pixel_percent": 100.0 * np.count_nonzero(mask) / mask.size,
            }
            for name, mask in regions.items()
            if name in (
                "persistent_valid_interior",
                "persistent_valid_depth_edge",
                "intermittent_depth",
                "persistent_invalid",
                "edge_jump_candidate",
            )
        },
        "hole_recovery": {
            "recovered_invalid_run_count": recovered_runs,
            "sequence_end_invalid_pixel_count": ongoing_invalid_pixels,
            **hole_recovery,
            "max_hole_run_frames_p50": finite_percentile(max_hole_run, 50),
            "max_hole_run_frames_p95": finite_percentile(max_hole_run, 95),
            "max_hole_run_frames_max": int(np.max(max_hole_run)),
        },
        "artifacts": {
            "depth_stack": stack_path.name,
            "spatial_region_metrics": "spatial_region_metrics.csv",
            "window_convergence": "window_convergence.csv",
            "hole_run_lengths": "hole_run_lengths.csv",
        },
    }
    with (output_dir / "depth_stability_summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    print(f"Depth stability analysis complete: frames={stack.shape[0]} output={output_dir}")


if __name__ == "__main__":
    main()
