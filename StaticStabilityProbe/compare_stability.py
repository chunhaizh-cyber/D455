#!/usr/bin/env python3
"""Compare two StaticStabilityProbe summaries without declaring a quality winner."""

import argparse
import csv
import json
from pathlib import Path


def load_summary(path):
    summary_path = Path(path)
    try:
        payload = json.loads(summary_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read {summary_path}: {exc}") from exc
    if payload.get("schema_version") != 1 or not isinstance(payload.get("metrics"), dict):
        raise ValueError(f"unsupported stability summary: {summary_path}")
    return payload


def compare_metric(name, baseline, candidate):
    row = {"metric": name}
    for percentile in ("p50", "p95", "max"):
        baseline_value = float(baseline.get(percentile, 0.0))
        candidate_value = float(candidate.get(percentile, 0.0))
        delta = candidate_value - baseline_value
        improvement_percent = (
            100.0 * (baseline_value - candidate_value) / baseline_value
            if baseline_value != 0.0
            else 0.0
        )
        row[f"baseline_{percentile}"] = baseline_value
        row[f"candidate_{percentile}"] = candidate_value
        row[f"delta_{percentile}"] = delta
        row[f"improvement_percent_{percentile}"] = improvement_percent
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, help="Auto-control stability_summary.json")
    parser.add_argument("--candidate", required=True, help="Locked-control stability_summary.json")
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()

    try:
        baseline = load_summary(args.baseline)
        candidate = load_summary(args.candidate)
    except ValueError as exc:
        parser.error(str(exc))

    metric_names = sorted(set(baseline["metrics"]) & set(candidate["metrics"]))
    rows = [
        compare_metric(name, baseline["metrics"][name], candidate["metrics"][name])
        for name in metric_names
    ]
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "stability_comparison.csv"
    fieldnames = [
        "metric",
        "baseline_p50", "candidate_p50", "delta_p50", "improvement_percent_p50",
        "baseline_p95", "candidate_p95", "delta_p95", "improvement_percent_p95",
        "baseline_max", "candidate_max", "delta_max", "improvement_percent_max",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    json_path = out_dir / "stability_comparison.json"
    output = {
        "schema_version": 1,
        "baseline": str(Path(args.baseline)),
        "candidate": str(Path(args.candidate)),
        "metric_count": len(rows),
        "decision": "descriptive_only",
        "interpretation": (
            "Positive improvement_percent means the candidate has lower variation. "
            "The first round does not apply an automatic sensor pass/fail threshold."
        ),
        "metrics": rows,
    }
    json_path.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {csv_path}")
    print(f"wrote {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
