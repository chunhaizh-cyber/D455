#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path


DEFAULT_SCORE_CONFIG = {
    "total_score": {
        "pixel_clustering": 0.20,
        "contour_quality": 0.20,
        "spatial_quality": 0.20,
        "temporal_stability": 0.15,
        "far_retention": 0.10,
        "performance": 0.10,
        "diagnostics": 0.05,
    },
    "hard_fail": {
        "cluster_coverage_percent_p50_min": 95.0,
        "unknown_percent_p50_max": 15.0,
        "total_frame_ms_p95_max": 100.0,
        "contour_lost_event_max": 30,
    },
    "performance_budget": {
        "realtime_30fps_ms": 33.3,
        "soft_budget_ms": 66.6,
        "hard_budget_ms": 100.0,
    },
}


def parse_scalar(value):
    value = value.strip()
    if value.lower() in ("true", "false"):
        return value.lower() == "true"
    try:
        if "." in value:
            return float(value)
        return int(value)
    except ValueError:
        return value.strip("\"'")


def read_simple_yaml(path):
    if not path.exists():
        return {}
    root = {}
    stack = [(-1, root)]
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        key, sep, value = line.strip().partition(":")
        if not sep:
            continue
        while stack and stack[-1][0] >= indent:
            stack.pop()
        parent = stack[-1][1]
        if value.strip() == "":
            child = {}
            parent[key] = child
            stack.append((indent, child))
        else:
            parent[key] = parse_scalar(value)
    return root


def merge_dict(defaults, overrides):
    result = dict(defaults)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge_dict(result[key], value)
        else:
            result[key] = value
    return result


def load_score_config(path):
    return merge_dict(DEFAULT_SCORE_CONFIG, read_simple_yaml(Path(path)))


def score_caps(config):
    raw = config.get("total_score", {})
    caps = {}
    for key, weight in raw.items():
        score_key = "diagnostic_completeness" if key == "diagnostics" else key
        caps[score_key] = float(weight) * 100.0
    return caps


def percentile(values, percent):
    clean = sorted(float(v) for v in values if v not in (None, ""))
    if not clean:
        return None
    if len(clean) == 1:
        return clean[0]
    pos = (len(clean) - 1) * percent / 100.0
    lo = int(pos)
    hi = min(lo + 1, len(clean) - 1)
    frac = pos - lo
    return clean[lo] * (1.0 - frac) + clean[hi] * frac


def read_frame_metrics(path):
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def read_events(path):
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def read_json(path):
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8-sig") as f:
        return json.load(f)


def row_float(row, key):
    try:
        return float(row.get(key) or 0.0)
    except (TypeError, ValueError):
        return 0.0


def row_sum(row, *keys):
    return sum(row_float(row, key) for key in keys)


def event_count(events, event_type):
    return sum(1 for e in events if e.get("event_type") == event_type)


def score_run(run_dir, config):
    manifest = read_json(run_dir / "run_manifest.json")
    frame_rows = read_frame_metrics(run_dir / "frame_metrics.csv")
    events = read_events(run_dir / "events.csv")
    cluster_metrics_exists = (run_dir / "cluster_metrics.jsonl").exists()
    caps = score_caps(config)
    hard_fail = config.get("hard_fail", {})
    performance_budget = config.get("performance_budget", {})

    coverage_p50 = percentile([r.get("cluster_coverage_percent") for r in frame_rows], 50)
    unknown_p50 = percentile([r.get("unknown_percent") for r in frame_rows], 50)
    frame_ms_p95 = percentile([r.get("total_frame_ms") for r in frame_rows], 95)
    far_cluster_p50 = percentile([r.get("far_cluster_count") for r in frame_rows], 50)
    total_pixels_p50 = percentile([r.get("total_pixels") for r in frame_rows], 50)
    far_retained_pixels_p50 = percentile([
        row_sum(r, "approx_stereo_contour_pixels", "image_only_contour_pixels", "depth_hole_candidate_pixels")
        for r in frame_rows
    ], 50)
    stereo_matched_p50 = percentile([r.get("stereo_matched_cluster_count") for r in frame_rows], 50)
    stereo_failed_p50 = percentile([r.get("stereo_failed_cluster_count") for r in frame_rows], 50)

    coverage_min = float(hard_fail.get("cluster_coverage_percent_p50_min", 95.0))
    unknown_max = float(hard_fail.get("unknown_percent_p50_max", 15.0))
    frame_ms_max = float(hard_fail.get("total_frame_ms_p95_max", 100.0))
    realtime_ms = float(performance_budget.get("realtime_30fps_ms", 33.3))
    soft_budget_ms = float(performance_budget.get("soft_budget_ms", 66.6))
    contour_lost_max = int(hard_fail.get("contour_lost_event_max", 30))

    coverage_score = 0.0 if coverage_p50 is None else max(0.0, min(1.0, (coverage_p50 - coverage_min) / max(1.0, 100.0 - coverage_min)))
    unknown_score = 0.0 if unknown_p50 is None else max(0.0, min(1.0, (unknown_max - unknown_p50) / max(1.0, unknown_max)))
    perf_score = 0.0 if frame_ms_p95 is None else max(0.0, min(1.0, (frame_ms_max - frame_ms_p95) / max(1.0, frame_ms_max - realtime_ms)))
    far_retained_ratio = 0.0
    if total_pixels_p50 and far_retained_pixels_p50 is not None:
        far_retained_ratio = max(0.0, far_retained_pixels_p50 / total_pixels_p50)
    far_pixel_score = min(1.0, far_retained_ratio / 0.05)
    stereo_presence_score = 1.0 if stereo_matched_p50 is not None and stereo_matched_p50 > 0 else 0.0

    merge_events = event_count(events, "cluster_merge")
    split_events = event_count(events, "cluster_split")
    contour_lost_events = event_count(events, "contour_lost")
    far_failed_events = event_count(events, "far_stereo_failed")
    unknown_spikes = event_count(events, "unknown_spike")

    pixel_cap = caps.get("pixel_clustering", 20.0)
    contour_cap = caps.get("contour_quality", 20.0)
    spatial_cap = caps.get("spatial_quality", 20.0)
    temporal_cap = caps.get("temporal_stability", 15.0)
    far_cap = caps.get("far_retention", 10.0)
    performance_cap = caps.get("performance", 10.0)
    diagnostic_cap = caps.get("diagnostic_completeness", 5.0)

    pixel_score = pixel_cap * (0.40 * coverage_score + 0.20 * unknown_score + (0.40 if frame_rows else 0.0))
    pixel_score = min(pixel_cap, pixel_score)

    contour_score = max(0.0, contour_cap - 0.10 * contour_cap * contour_lost_events - 0.05 * contour_cap * merge_events - 0.025 * contour_cap * split_events)
    spatial_score = max(0.0, spatial_cap - 0.10 * spatial_cap * far_failed_events)
    temporal_score = max(0.0, temporal_cap - 0.10 * temporal_cap * contour_lost_events - (1.0 / 15.0) * temporal_cap * merge_events - (0.5 / 15.0) * temporal_cap * split_events)
    far_score = far_cap * (0.70 * far_pixel_score + 0.30 * stereo_presence_score)
    far_score = max(0.0, far_score - 0.20 * far_cap * far_failed_events - 0.10 * far_cap * contour_lost_events)
    performance_score = performance_cap * perf_score

    diag_parts = [
        (run_dir / "run_manifest.json").exists(),
        (run_dir / "config_snapshot.json").exists(),
        (run_dir / "frame_metrics.csv").exists(),
        cluster_metrics_exists,
        (run_dir / "events.csv").exists() and (run_dir / "sample_frames").exists(),
    ]
    diag_score = diagnostic_cap * sum(1.0 for x in diag_parts if x) / len(diag_parts)

    scores = {
        "pixel_clustering": round(pixel_score, 3),
        "contour_quality": round(contour_score, 3),
        "spatial_quality": round(spatial_score, 3),
        "temporal_stability": round(temporal_score, 3),
        "far_retention": round(far_score, 3),
        "performance": round(performance_score, 3),
        "diagnostic_completeness": round(diag_score, 3),
    }
    total = sum(scores.values())

    hard_fail_reasons = []
    if coverage_p50 is None:
        hard_fail_reasons.append("frame_metrics.csv missing or empty")
    elif coverage_p50 < coverage_min:
        hard_fail_reasons.append(f"cluster_coverage_percent_p50 < {coverage_min:g}")
    if unknown_p50 is not None and unknown_p50 > unknown_max:
        hard_fail_reasons.append(f"unknown_percent_p50 > {unknown_max:g}")
    if frame_ms_p95 is not None and frame_ms_p95 > frame_ms_max:
        hard_fail_reasons.append(f"total_frame_ms_p95 > {frame_ms_max:g}")
    if contour_lost_events > contour_lost_max:
        hard_fail_reasons.append(f"contour_lost_event_count > {contour_lost_max}")
    if not cluster_metrics_exists:
        hard_fail_reasons.append("cluster_metrics.jsonl missing")

    top_failures = []
    if unknown_spikes:
        top_failures.append(f"unknown_spike events: {unknown_spikes}")
    if contour_lost_events:
        top_failures.append(f"contour_lost events: {contour_lost_events}")
    if merge_events:
        top_failures.append(f"cluster_merge events: {merge_events}")
    if far_failed_events:
        top_failures.append(f"far_stereo_failed events: {far_failed_events}")

    recommended = []
    if unknown_p50 is not None and unknown_p50 > 10:
        recommended.append("increase visual contour/background fallback before depth gating")
    if merge_events:
        recommended.append("inspect PCL vote ratio and split boundary signals for merged near clusters")
    if far_failed_events or (far_cluster_p50 is not None and far_cluster_p50 <= 0):
        recommended.append("inspect far contour retention and stereo confidence downgrade")
    if frame_ms_p95 is not None and frame_ms_p95 > soft_budget_ms:
        recommended.append("switch to lighter feature profile or sample heavy metrics")

    return {
        "run_id": manifest.get("run_id", run_dir.name),
        "candidate_id": manifest.get("candidate_id"),
        "case_id": manifest.get("case_id"),
        "pass": not hard_fail_reasons,
        "total_score": round(total, 3),
        "scores": scores,
        "metrics": {
            "cluster_coverage_percent_p50": coverage_p50,
            "unknown_percent_p50": unknown_p50,
            "far_cluster_count_p50": far_cluster_p50,
            "far_retained_pixels_p50": far_retained_pixels_p50,
            "stereo_matched_cluster_count_p50": stereo_matched_p50,
            "stereo_failed_cluster_count_p50": stereo_failed_p50,
            "contour_lost_event_count": contour_lost_events,
            "merge_event_count": merge_events,
            "split_event_count": split_events,
            "far_stereo_failed_event_count": far_failed_events,
            "total_frame_ms_p95": frame_ms_p95,
        },
        "hard_fail_reasons": hard_fail_reasons,
        "top_failures": top_failures,
        "recommended_next_actions": recommended,
        "score_config": {
            "weights": config.get("total_score", {}),
            "hard_fail": hard_fail,
            "performance_budget": performance_budget,
        },
    }


def write_markdown(run_dir, score):
    summary = run_dir / "run_summary.md"
    summary.write_text(
        "# Run Summary\n\n"
        f"- run_id: {score['run_id']}\n"
        f"- pass: {score['pass']}\n"
        f"- total_score: {score['total_score']}\n\n"
        "## Top Failures\n\n"
        + "\n".join(f"- {x}" for x in score["top_failures"] or ["None"])
        + "\n\n## Recommended Next Actions\n\n"
        + "\n".join(f"- {x}" for x in score["recommended_next_actions"] or ["None"])
        + "\n",
        encoding="utf-8",
    )

    failure = run_dir / "failure_report.md"
    failure.write_text(
        "# Failure Report\n\n"
        "## Hard Fail Reasons\n\n"
        + "\n".join(f"- {x}" for x in score["hard_fail_reasons"] or ["None"])
        + "\n\n## Top Failures\n\n"
        + "\n".join(f"- {x}" for x in score["top_failures"] or ["None"])
        + "\n",
        encoding="utf-8",
    )


def iter_run_dirs(root):
    root = Path(root)
    if (root / "run_manifest.json").exists() or (root / "frame_metrics.csv").exists():
        yield root
        return
    for path in sorted(root.glob("**/run_manifest.json")):
        if "_template" not in path.parts:
            yield path.parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", default="analysis_runs")
    parser.add_argument("--weights", default="eval/score_weights.yaml")
    args = parser.parse_args()

    config = load_score_config(args.weights)
    count = 0
    for run_dir in iter_run_dirs(args.runs):
        score = score_run(run_dir, config)
        (run_dir / "run_score.json").write_text(
            json.dumps(score, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        write_markdown(run_dir, score)
        print(f"scored {run_dir}: {score['total_score']} pass={score['pass']}")
        count += 1
    if count == 0:
        print("No run directories found.")


if __name__ == "__main__":
    main()
