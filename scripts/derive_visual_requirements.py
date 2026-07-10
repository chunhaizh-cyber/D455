#!/usr/bin/env python3
"""Derive deduplicated visual requirement candidates from analysis runs."""

import argparse
import csv
import json
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path


HARD_LIMITS = {
    "cluster_coverage_percent_p50_min": 95.0,
    "unknown_percent_p50_max": 15.0,
    "total_frame_ms_p95_max": 100.0,
}

EVENT_TO_FAILURE = {
    "frame_time_spike": "frame_time_spike",
    "contour_lost_event": "contour_lost",
    "unknown_spike": "unknown_spike",
    "far_stereo_failed": "far_stereo_failed",
    "roi_rejected_large": "roi_rejected_large",
}


def number(value, default=0.0):
    try:
        if value in (None, ""):
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def integer(value, default=0):
    return int(round(number(value, default)))


def slug(value):
    chars = []
    for char in str(value).lower():
        chars.append(char if char.isalnum() else "-")
    return "".join(chars).strip("-") or "unknown"


def read_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read JSON {path}: {exc}") from exc


def find_run_score_paths(inputs):
    paths = []
    seen = set()
    for raw in inputs:
        path = Path(raw)
        if path.is_file() and path.name == "run_score.json":
            candidates = [path]
        elif path.is_dir():
            candidates = [path / "run_score.json"] if (path / "run_score.json").exists() else sorted(path.rglob("run_score.json"))
        else:
            raise ValueError(f"run input does not exist: {path}")
        for candidate in candidates:
            resolved = candidate.resolve()
            if candidate.exists() and resolved not in seen:
                paths.append(candidate)
                seen.add(resolved)
    if not paths:
        raise ValueError("no run_score.json found")
    return paths


def read_frame_summary(run_dir):
    path = run_dir / "frame_metrics.csv"
    values = []
    if not path.exists():
        return {"total_pixels": 0.0, "frame_count": 0}
    with path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            total_pixels = number(row.get("total_pixels"))
            if total_pixels > 0:
                values.append(total_pixels)
    return {
        "total_pixels": max(values) if values else 0.0,
        "frame_count": len(values),
    }


def read_event_counts(run_dir):
    counts = defaultdict(int)
    path = run_dir / "events.csv"
    if not path.exists():
        return counts
    with path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            event_type = str(row.get("event_type", "")).strip()
            failure = EVENT_TO_FAILURE.get(event_type)
            if failure:
                counts[failure] += 1
    return counts


def load_run(score_path):
    score = read_json(score_path)
    if not isinstance(score, dict) or not score.get("run_id"):
        raise ValueError(f"{score_path}: run_score.json needs run_id")
    run_dir = score_path.parent
    manifest_path = run_dir / "run_manifest.json"
    manifest = read_json(manifest_path) if manifest_path.exists() else {}
    metrics = score.get("metrics", {})
    return {
        "score_path": score_path,
        "run_dir": run_dir,
        "run_id": score.get("run_id"),
        "case_id": score.get("case_id") or manifest.get("case_id") or "unknown_case",
        "candidate_id": score.get("candidate_id") or manifest.get("candidate_id") or "unknown_method",
        "score": score,
        "metrics": metrics,
        "frame_summary": read_frame_summary(run_dir),
        "event_counts": read_event_counts(run_dir),
    }


def metric(run, name):
    return number(run["metrics"].get(name))


def detect_findings(run):
    metrics = run["metrics"]
    events = run["event_counts"]
    findings = []
    coverage = metric(run, "cluster_coverage_percent_p50")
    unknown = metric(run, "unknown_percent_p50")
    frame_p95 = metric(run, "total_frame_ms_p95")
    over_budget = integer(metrics.get("frame_time_over_hard_budget_count"))
    contour_lost = integer(metrics.get("contour_lost_event_count"))
    far_failed = integer(metrics.get("far_stereo_failed_event_count"))

    if coverage < HARD_LIMITS["cluster_coverage_percent_p50_min"] or events["coverage_low"] > 0:
        findings.append({
            "failure_type": "coverage_low",
            "target_feature": "证据解决率",
            "goal": "提高非未知像素的证据解决率",
            "current_value": {"cluster_coverage_percent_p50": coverage},
            "target_value": {"cluster_coverage_percent_p50_min": HARD_LIMITS["cluster_coverage_percent_p50_min"]},
        })
    if unknown > HARD_LIMITS["unknown_percent_p50_max"] or events["unknown_spike"] > 0:
        findings.append({
            "failure_type": "unknown_spike",
            "target_feature": "未知率",
            "goal": "降低无证据未知像素，同时不伪造背景归属",
            "current_value": {"unknown_percent_p50": unknown},
            "target_value": {"unknown_percent_p50_max": HARD_LIMITS["unknown_percent_p50_max"]},
        })
    if frame_p95 > HARD_LIMITS["total_frame_ms_p95_max"] or over_budget > 0 or events["frame_time_spike"] > 0:
        findings.append({
            "failure_type": "frame_time_spike",
            "target_feature": "帧耗时稳定性",
            "goal": "降低高成本刷新造成的最坏帧耗时",
            "current_value": {
                "total_frame_ms_p95": frame_p95,
                "frame_time_over_hard_budget_count": over_budget,
                "total_frame_ms_max": metric(run, "total_frame_ms_max"),
            },
            "target_value": {
                "total_frame_ms_p95_max": HARD_LIMITS["total_frame_ms_p95_max"],
                "frame_time_over_hard_budget_count": 0,
            },
        })
    if contour_lost > 0 or events["contour_lost"] > 0:
        findings.append({
            "failure_type": "contour_lost",
            "target_feature": "轮廓归属连续性",
            "goal": "减少遮挡、重现或运动中的轮廓丢失",
            "current_value": {"contour_lost_event_count": contour_lost},
            "target_value": {"contour_lost_event_count_max": 0},
        })
    if far_failed > 0 or events["far_stereo_failed"] > 0:
        findings.append({
            "failure_type": "far_stereo_failed",
            "target_feature": "远场证据保持",
            "goal": "远场双目失败时保留可信轮廓并正确降级",
            "current_value": {"far_stereo_failed_event_count": far_failed},
            "target_value": {"far_stereo_failed_event_count_max": 0},
        })

    roi_candidate = metric(run, "color_contour_refresh_roi_candidate_pixels_p95")
    roi_limit = metric(run, "color_contour_refresh_roi_max_pixels_p50")
    roi_rejected = integer(metrics.get("color_contour_refresh_roi_rejected_large_count"))
    if roi_rejected > 0 or (roi_candidate > 0 and roi_limit > 0 and roi_candidate > roi_limit):
        frame_pixels = run["frame_summary"]["total_pixels"]
        target_roi = frame_pixels * 0.25 if frame_pixels > 0 else None
        findings.append({
            "failure_type": "roi_rejected_large",
            "target_feature": "局部刷新有效性",
            "goal": "让运动刷新区域保持局部，避免稀疏运动退化为整帧刷新",
            "current_value": {
                "roi_candidate_pixels_p95": roi_candidate,
                "roi_max_pixels_p50": roi_limit,
                "roi_rejected_large_count": roi_rejected,
                "roi_motion_bbox_pixels_p95": metric(run, "color_contour_refresh_roi_motion_bbox_pixels_p95"),
                "frame_time_over_hard_budget_count": over_budget,
            },
            "target_value": {
                "roi_candidate_pixels_p95_max": target_roi if target_roi else "within_declared_area_budget",
                "roi_rejected_large_count": 0,
                "frame_time_over_hard_budget_count": 0,
            },
        })
    return findings


def merge_findings(runs, min_occurrences):
    grouped = {}
    for run in runs:
        for finding in detect_findings(run):
            method = run["candidate_id"]
            dedupe_key = "+".join([
                run["case_id"],
                finding["target_feature"],
                finding["failure_type"],
                method,
            ])
            record_id = "-".join([
                "visual-need", slug(run["case_id"]), slug(finding["failure_type"]), slug(method),
            ])
            item = grouped.setdefault(record_id, {
                "record_type": "requirement",
                "schema_version": 1,
                "record_id": record_id,
                "status": "candidate",
                "created_at": datetime.now(timezone.utc).date().isoformat(),
                "goal": finding["goal"],
                "target_feature": finding["target_feature"],
                "current_value": dict(finding["current_value"]),
                "target_value": dict(finding["target_value"]),
                "dedupe_key": dedupe_key,
                "source_evidence": [],
                "protection_conditions": {
                    "cluster_coverage_percent_p50_min": HARD_LIMITS["cluster_coverage_percent_p50_min"],
                    "unknown_percent_p50_max": HARD_LIMITS["unknown_percent_p50_max"],
                    "near_score_delta_min": -1,
                    "far_score_delta_min": -1,
                    "contour_lost_event_count_max": 0,
                },
                "occurrence_count": 0,
                "source_method_id": method,
            })
            item["occurrence_count"] += 1
            for key, value in finding["current_value"].items():
                if isinstance(value, (int, float)):
                    item["current_value"][key] = max(number(item["current_value"].get(key)), number(value))
            evidence = {
                "run_id": run["run_id"],
                "case_id": run["case_id"],
                "split": "historical",
                "score_path": run["score_path"].as_posix(),
            }
            if not any(existing["run_id"] == evidence["run_id"] for existing in item["source_evidence"]):
                item["source_evidence"].append(evidence)
    for item in grouped.values():
        if item["occurrence_count"] >= min_occurrences:
            item["status"] = "confirmed"
    return list(sorted(grouped.values(), key=lambda item: item["record_id"]))


def load_existing_ids(path):
    if not path.exists():
        return set()
    ids = set()
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        try:
            record = json.loads(raw)
        except json.JSONDecodeError:
            continue
        if record.get("record_id"):
            ids.add(record["record_id"])
    return ids


def register_candidates(candidates, registry, min_occurrences):
    eligible = [item for item in candidates if item["occurrence_count"] >= min_occurrences]
    existing = load_existing_ids(registry)
    new_items = [item for item in eligible if item["record_id"] not in existing]
    if new_items:
        registry.parent.mkdir(parents=True, exist_ok=True)
        with registry.open("a", encoding="utf-8", newline="\n") as handle:
            for item in new_items:
                handle.write(json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n")
    return len(new_items), len(eligible), len(existing)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", action="append", required=True, help="Run directory or run_score.json; repeatable")
    parser.add_argument("--baseline", default="", help="Optional baseline config reference")
    parser.add_argument("--out", required=True, help="JSON output for requirement candidates")
    parser.add_argument("--min-occurrences", type=int, default=2)
    parser.add_argument("--register", action="store_true")
    parser.add_argument("--registry", default="eval/visual_evolution/requirements.jsonl")
    args = parser.parse_args()
    if args.min_occurrences < 1:
        parser.error("--min-occurrences must be >= 1")

    try:
        score_paths = find_run_score_paths(args.runs)
        runs = [load_run(path) for path in score_paths]
        candidates = merge_findings(runs, args.min_occurrences)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    output = {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "baseline": args.baseline,
        "min_occurrences": args.min_occurrences,
        "source_runs": [run["run_id"] for run in runs],
        "candidates": candidates,
        "registration": None,
    }
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    if args.register:
        added, eligible, existing = register_candidates(candidates, Path(args.registry), args.min_occurrences)
        output["registration"] = {
            "registry": args.registry,
            "eligible_count": eligible,
            "added_count": added,
            "existing_count": existing,
        }
        out_path.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print(f"wrote {out_path}: candidates={len(candidates)}")
    if args.register:
        print(f"registered eligible candidates: added={added}, eligible={eligible}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
