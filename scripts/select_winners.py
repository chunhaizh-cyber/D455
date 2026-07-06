#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path


def iter_scores(root):
    root = Path(root)
    for path in sorted(root.glob("**/run_score.json")):
        if "_template" in path.parts:
            continue
        with path.open("r", encoding="utf-8") as f:
            score = json.load(f)
        score["_path"] = path
        yield score


def metric(score, name, default=""):
    return score.get("metrics", {}).get(name, default)


def is_passed(score):
    return bool(score.get("pass"))


def write_no_pass_candidate(writer, slot):
    writer.writerow([
        slot,
        "no_pass_candidate",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
    ])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", default="analysis_runs")
    parser.add_argument("--out", default="leaderboards")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    scores = list(iter_scores(args.runs))
    scores.sort(key=lambda s: s.get("total_score", 0), reverse=True)
    passed_scores = [s for s in scores if is_passed(s)]

    leaderboard = out / "leaderboard.csv"
    with leaderboard.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "candidate_id", "case_id", "total_score", "pass",
            "cluster_coverage_p50", "unknown_p50", "near_score", "far_score",
            "temporal_score", "frame_ms_p95", "frame_ms_max",
            "frame_over_100ms_count", "sampled_frame_ms_p95", "timing_source",
            "merge_events", "split_events",
            "contour_lost_events", "color_refresh_count", "color_refresh_motion_count",
            "color_refresh_unknown_spike_count", "color_refresh_far_loss_count",
            "color_cache_reuse_count", "color_stereo_reuse_p50",
            "color_async_submitted_count", "color_async_applied_count",
            "color_async_dropped_count", "color_cache_age_p95",
            "color_async_worker_ms_p95", "run_id"
        ])
        for s in scores:
            writer.writerow([
                s.get("candidate_id") or "",
                s.get("case_id") or "",
                s.get("total_score", 0),
                int(bool(s.get("pass"))),
                metric(s, "cluster_coverage_percent_p50"),
                metric(s, "unknown_percent_p50"),
                s.get("scores", {}).get("spatial_quality", ""),
                s.get("scores", {}).get("far_retention", ""),
                s.get("scores", {}).get("temporal_stability", ""),
                metric(s, "total_frame_ms_p95"),
                metric(s, "total_frame_ms_max"),
                metric(s, "frame_time_over_hard_budget_count"),
                metric(s, "sampled_total_frame_ms_p95"),
                metric(s, "scored_timing_source"),
                metric(s, "merge_event_count"),
                metric(s, "split_event_count"),
                metric(s, "contour_lost_event_count"),
                metric(s, "color_contour_refresh_count"),
                metric(s, "color_contour_refresh_motion_count"),
                metric(s, "color_contour_refresh_unknown_spike_count"),
                metric(s, "color_contour_refresh_far_loss_count"),
                metric(s, "color_contour_cache_reuse_count"),
                metric(s, "color_contour_stereo_reuse_count_p50"),
                metric(s, "color_contour_async_submitted_count"),
                metric(s, "color_contour_async_applied_count"),
                metric(s, "color_contour_async_dropped_count"),
                metric(s, "color_contour_cache_age_frames_p95"),
                metric(s, "color_contour_async_worker_ms_p95"),
                s.get("run_id", ""),
            ])

    slots = [
        ("best_overall", lambda s: s.get("total_score", 0)),
        ("best_far_retention", lambda s: s.get("scores", {}).get("far_retention", 0)),
        ("best_realtime", lambda s: s.get("scores", {}).get("performance", 0)),
        ("best_low_unknown", lambda s: -(metric(s, "unknown_percent_p50", 999) or 999)),
        ("best_low_merge", lambda s: -(metric(s, "merge_event_count", 999) or 999)),
        ("best_low_split", lambda s: -(metric(s, "split_event_count", 999) or 999)),
    ]
    pareto = out / "pareto_front.csv"
    with pareto.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["slot", "candidate_id", "total_score", "near_score", "far_score", "performance_score", "unknown_p50", "frame_ms_p95", "run_id"])
        for slot, key in slots:
            if not passed_scores:
                write_no_pass_candidate(writer, slot)
                continue
            best = max(passed_scores, key=key)
            writer.writerow([
                slot,
                best.get("candidate_id") or "",
                best.get("total_score", 0),
                best.get("scores", {}).get("spatial_quality", ""),
                best.get("scores", {}).get("far_retention", ""),
                best.get("scores", {}).get("performance", ""),
                metric(best, "unknown_percent_p50"),
                metric(best, "total_frame_ms_p95"),
                best.get("run_id", ""),
            ])

    print(f"wrote {leaderboard}")
    print(f"wrote {pareto}")
    print(f"passed runs available for winner selection: {len(passed_scores)}/{len(scores)}")


if __name__ == "__main__":
    main()
