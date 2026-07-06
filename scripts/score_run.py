#!/usr/bin/env python3
import argparse
import csv
import json
import statistics
from pathlib import Path


SCORE_KEYS = {
    "pixel_clustering": 20.0,
    "contour_quality": 20.0,
    "spatial_quality": 20.0,
    "temporal_stability": 15.0,
    "far_retention": 10.0,
    "performance": 10.0,
    "diagnostic_completeness": 5.0,
}


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


def event_count(events, event_type):
    return sum(1 for e in events if e.get("event_type") == event_type)


def score_run(run_dir):
    manifest = read_json(run_dir / "run_manifest.json")
    frame_rows = read_frame_metrics(run_dir / "frame_metrics.csv")
    events = read_events(run_dir / "events.csv")
    cluster_metrics_exists = (run_dir / "cluster_metrics.jsonl").exists()

    coverage_p50 = percentile([r.get("cluster_coverage_percent") for r in frame_rows], 50)
    unknown_p50 = percentile([r.get("unknown_percent") for r in frame_rows], 50)
    frame_ms_p95 = percentile([r.get("total_frame_ms") for r in frame_rows], 95)
    far_cluster_p50 = percentile([r.get("far_cluster_count") for r in frame_rows], 50)

    coverage_score = 0.0 if coverage_p50 is None else max(0.0, min(1.0, (coverage_p50 - 95.0) / 4.0))
    unknown_score = 0.0 if unknown_p50 is None else max(0.0, min(1.0, (15.0 - unknown_p50) / 15.0))
    perf_score = 0.0 if frame_ms_p95 is None else max(0.0, min(1.0, (100.0 - frame_ms_p95) / (100.0 - 33.3)))

    merge_events = event_count(events, "cluster_merge")
    split_events = event_count(events, "cluster_split")
    contour_lost_events = event_count(events, "contour_lost")
    far_failed_events = event_count(events, "far_stereo_failed")
    unknown_spikes = event_count(events, "unknown_spike")

    pixel_score = 8.0 * coverage_score + 4.0 * unknown_score
    pixel_score += 8.0 if frame_rows else 0.0
    pixel_score = min(20.0, pixel_score)

    contour_score = max(0.0, 20.0 - 2.0 * contour_lost_events - 1.0 * merge_events - 0.5 * split_events)
    spatial_score = max(0.0, 20.0 - 2.0 * far_failed_events)
    temporal_score = max(0.0, 15.0 - 1.5 * contour_lost_events - 1.0 * merge_events - 0.5 * split_events)
    far_score = max(0.0, 10.0 - 2.0 * far_failed_events - 1.0 * contour_lost_events)
    performance_score = 10.0 * perf_score

    diag_parts = [
        (run_dir / "run_manifest.json").exists(),
        (run_dir / "config_snapshot.json").exists(),
        (run_dir / "frame_metrics.csv").exists(),
        cluster_metrics_exists,
        (run_dir / "events.csv").exists() and (run_dir / "sample_frames").exists(),
    ]
    diag_score = sum(1.0 for x in diag_parts if x)

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
    elif coverage_p50 < 95:
        hard_fail_reasons.append("cluster_coverage_percent_p50 < 95")
    if unknown_p50 is not None and unknown_p50 > 15:
        hard_fail_reasons.append("unknown_percent_p50 > 15")
    if frame_ms_p95 is not None and frame_ms_p95 > 100:
        hard_fail_reasons.append("total_frame_ms_p95 > 100")
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
    if frame_ms_p95 is not None and frame_ms_p95 > 66.6:
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
            "contour_lost_event_count": contour_lost_events,
            "merge_event_count": merge_events,
            "split_event_count": split_events,
            "far_stereo_failed_event_count": far_failed_events,
            "total_frame_ms_p95": frame_ms_p95,
        },
        "hard_fail_reasons": hard_fail_reasons,
        "top_failures": top_failures,
        "recommended_next_actions": recommended,
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
    args = parser.parse_args()

    count = 0
    for run_dir in iter_run_dirs(args.runs):
        score = score_run(run_dir)
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
