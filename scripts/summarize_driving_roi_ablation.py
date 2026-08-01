#!/usr/bin/env python3
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


def iter_scores(root):
    for path in sorted(Path(root).rglob("run_score.json")):
        score = json.loads(path.read_text(encoding="utf-8"))
        score["_path"] = str(path)
        yield score


def metric(score, name, default=0.0):
    return score.get("metrics", {}).get(name, default)


def aggregate(candidate_id, scores):
    frame_count = sum(max(1, int(metric(score, "profile_frame_count", 0))) for score in scores)

    def weighted_mean(name):
        total = 0.0
        for score in scores:
            weight = max(1, int(metric(score, "profile_frame_count", 0)))
            total += float(metric(score, name, 0.0)) * weight
        return total / max(1, frame_count)

    coverage_min = min((float(metric(score, "cluster_coverage_percent_p50", 0.0)) for score in scores), default=0.0)
    unknown_max = max((float(metric(score, "unknown_percent_p50", 100.0)) for score in scores), default=100.0)
    contour_lost = sum(int(metric(score, "contour_lost_event_count", 0)) for score in scores)
    merge_events = sum(int(metric(score, "merge_event_count", 0)) for score in scores)
    split_events = sum(int(metric(score, "split_event_count", 0)) for score in scores)
    return {
        "candidate_id": candidate_id,
        "run_count": len(scores),
        "hard_pass_count": sum(1 for score in scores if score.get("pass")),
        "coverage_p50_min": round(coverage_min, 3),
        "unknown_p50_max": round(unknown_max, 3),
        "contour_lost_events": contour_lost,
        "merge_events": merge_events,
        "split_events": split_events,
        "frame_ms_p95_mean": round(sum(float(metric(score, "total_frame_ms_p95", 0.0)) for score in scores) / max(1, len(scores)), 3),
        "semantic_processed_percent_mean": round(weighted_mean("semantic_processed_percent_mean"), 3),
        "semantic_full_frame_equivalent": round(sum(float(metric(score, "semantic_full_frame_equivalent", 0.0)) for score in scores), 3),
        "semantic_processing_frame_count": sum(int(metric(score, "semantic_processing_frame_count", 0)) for score in scores),
        "driving_risk_roi_percent_p50": round(weighted_mean("driving_risk_roi_percent_p50"), 3),
        "driving_risk_clipped_all_dirty_count": sum(int(metric(score, "driving_risk_roi_clipped_all_dirty_count", 0)) for score in scores),
        "ledger_proxy_pass": coverage_min >= 95.0 and unknown_max <= 15.0,
        "temporal_proxy_pass": contour_lost == 0 and merge_events == 0 and split_events == 0,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--baseline-id", default="candidate_0050")
    parser.add_argument("--min-workload-reduction-percent", type=float, default=10.0)
    args = parser.parse_args()

    grouped = defaultdict(list)
    for score in iter_scores(args.runs):
        grouped[score.get("candidate_id") or "unknown"].append(score)
    if not grouped:
        raise SystemExit(f"No run_score.json found under {args.runs}")

    rows = [aggregate(candidate_id, scores) for candidate_id, scores in sorted(grouped.items())]
    baseline = next((row for row in rows if row["candidate_id"] == args.baseline_id), None)
    if baseline is None:
        raise SystemExit(f"Baseline candidate not found: {args.baseline_id}")
    baseline_workload = float(baseline["semantic_processed_percent_mean"])
    for row in rows:
        row["semantic_workload_reduction_vs_baseline_percent"] = round(
            baseline_workload - float(row["semantic_processed_percent_mean"]), 3
        )
        row["workload_proxy_pass"] = (
            row["candidate_id"] == args.baseline_id or
            row["semantic_workload_reduction_vs_baseline_percent"] >=
                args.min_workload_reduction_percent
        )
        row["mechanism_proxy_pass"] = bool(
            row["ledger_proxy_pass"] and
            row["temporal_proxy_pass"] and
            row["workload_proxy_pass"]
        )

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    csv_path = out / "ablation_summary.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    decision = {
        "scope": "offline mechanism proxy; not autonomous-driving safety validation",
        "baseline_id": args.baseline_id,
        "absolute_performance_is_reported_not_gated": True,
        "missing_hard_evidence": [
            "synchronized vehicle speed, steering and planned trajectory",
            "trajectory occupancy and false-free-space ground truth",
            "independent repeated speed-stratified captures",
        ],
        "candidates": rows,
    }
    json_path = out / "ablation_decision.json"
    json_path.write_text(json.dumps(decision, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {csv_path}")
    print(f"wrote {json_path}")


if __name__ == "__main__":
    main()
