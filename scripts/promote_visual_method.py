#!/usr/bin/env python3
"""Promote a method only after explicit replay, holdout and shadow evidence."""

import argparse
import json
import sys
from datetime import date
from pathlib import Path

from compare_runs import evaluate_regression, load_score
from score_run import load_score_config


def load_jsonl(path, record_type):
    if not path.exists():
        raise ValueError(f"registry does not exist: {path}")
    records = {}
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        text = raw.strip()
        if not text or text.startswith("#"):
            continue
        try:
            record = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path}:{line_number}: invalid JSON: {exc.msg}") from exc
        if record.get("record_type") == record_type:
            records[record["record_id"]] = record
    return records


def read_score(path, split):
    score = load_score(path)
    if not score.get("run_id"):
        raise ValueError(f"{path}: missing run_id")
    if not score.get("pass"):
        raise ValueError(f"{path}: pass=false cannot be used for promotion")
    return {
        "run_id": score["run_id"],
        "case_id": score.get("case_id"),
        "split": split,
        "score_path": str(Path(path)),
    }, score


def evaluate_method_run(baseline, score, config, method_version, split):
    if score.get("candidate_id") != method_version:
        raise ValueError(
            f"{split} run candidate_id does not match method version: "
            f"{score.get('candidate_id')} != {method_version}"
        )
    if baseline.get("case_id") and score.get("case_id") and baseline.get("case_id") != score.get("case_id"):
        raise ValueError(
            f"{split} baseline/candidate case mismatch: "
            f"{baseline.get('case_id')} != {score.get('case_id')}"
        )
    return evaluate_regression(baseline, score, config)


def append_record(path, record):
    existing = load_jsonl(path, record["record_type"]) if path.exists() else {}
    if record["record_id"] in existing:
        raise ValueError(f"record already exists: {record['record_id']}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8", newline="\n") as handle:
        handle.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")


def replace_record(path, record_type, record_id, replacement):
    lines = path.read_text(encoding="utf-8").splitlines()
    found = False
    output = []
    for raw in lines:
        if not raw.strip() or raw.lstrip().startswith("#"):
            output.append(raw)
            continue
        record = json.loads(raw)
        if record.get("record_type") == record_type and record.get("record_id") == record_id:
            output.append(json.dumps(replacement, ensure_ascii=False, separators=(",", ":")))
            found = True
        else:
            output.append(raw)
    if not found:
        raise ValueError(f"record not found: {record_id}")
    path.write_text("\n".join(output) + "\n", encoding="utf-8")


def copy_promoted_config(method, bucket, target, force):
    source = Path(method["configuration_ref"])
    if not source.exists():
        raise ValueError(f"method configuration does not exist: {source}")
    if target.exists() and not force:
        raise ValueError(f"best config exists; pass --force-best-config to replace: {target}")
    config = json.loads(source.read_text(encoding="utf-8"))
    config["applicability_bucket"] = bucket
    config["promotion_status"] = "approved"
    config["source_method_id"] = method["method_id"]
    config["source_method_version"] = method["method_version"]
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--method-id", required=True)
    parser.add_argument("--method-version", required=True)
    parser.add_argument("--applicability-bucket", required=True)
    parser.add_argument("--requirement-id", required=True)
    parser.add_argument("--task-id", required=True)
    parser.add_argument("--baseline-run", required=True)
    parser.add_argument("--fixed-run", action="append", required=True)
    parser.add_argument("--holdout-baseline-run", default="")
    parser.add_argument("--holdout-run", action="append", default=[])
    parser.add_argument("--shadow-baseline-run", default="")
    parser.add_argument("--shadow-run", action="append", default=[])
    parser.add_argument("--replay-only", action="store_true")
    parser.add_argument("--methods", default="eval/visual_evolution/methods.jsonl")
    parser.add_argument("--promotions", default="eval/visual_evolution/promotions.jsonl")
    parser.add_argument("--requirements", default="eval/visual_evolution/requirements.jsonl")
    parser.add_argument("--tasks", default="eval/visual_evolution/tasks.jsonl")
    parser.add_argument("--weights", default="eval/score_weights.yaml")
    parser.add_argument("--best-config", default="")
    parser.add_argument("--force-best-config", action="store_true")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    try:
        methods = load_jsonl(Path(args.methods), "method")
        requirements = load_jsonl(Path(args.requirements), "requirement")
        tasks = load_jsonl(Path(args.tasks), "task")
        method = methods.get(args.method_id)
        if method is None:
            raise ValueError(f"unknown method: {args.method_id}")
        if method.get("method_version") != args.method_version:
            raise ValueError("method version does not match registry")
        if args.applicability_bucket not in method.get("applicability_buckets", []):
            raise ValueError("method does not declare the selected applicability bucket")
        if args.requirement_id not in requirements:
            raise ValueError(f"unknown requirement: {args.requirement_id}")
        if args.replay_only and method.get("status") == "probe_only":
            raise ValueError("probe_only method cannot be registered as replay_only")
        task = tasks.get(args.task_id)
        if task is None:
            raise ValueError(f"unknown task: {args.task_id}")
        if task.get("requirement_id") != args.requirement_id:
            raise ValueError("task does not belong to requirement")
        if args.method_id not in task.get("candidate_method_ids", []):
            raise ValueError("method is not registered as a task candidate")
        if not args.replay_only and task.get("status") in {"blocked_missing_holdout", "failed"}:
            raise ValueError("blocked or failed task cannot be promoted")
        baseline_ref, baseline_score = read_score(args.baseline_run, "historical")
        holdout_baseline_ref = None
        holdout_baseline_score = None
        if args.holdout_baseline_run:
            holdout_baseline_ref, holdout_baseline_score = read_score(args.holdout_baseline_run, "test")
        shadow_baseline_ref = None
        shadow_baseline_score = None
        if args.shadow_baseline_run:
            shadow_baseline_ref, shadow_baseline_score = read_score(args.shadow_baseline_run, "shadow")
        config = load_score_config(Path(args.weights))
        fixed_refs = []
        all_fixed_results = []
        for path in args.fixed_run:
            ref, score = read_score(path, "fixed")
            fixed_refs.append(ref)
            all_fixed_results.append(
                evaluate_method_run(baseline_score, score, config, args.method_version, "fixed")
            )
        holdout_refs = []
        holdout_scores = []
        for path in args.holdout_run:
            if holdout_baseline_score is None:
                raise ValueError("holdout runs require --holdout-baseline-run")
            ref, score = read_score(path, "holdout")
            holdout_refs.append(ref)
            holdout_scores.append((
                ref,
                score,
                evaluate_method_run(holdout_baseline_score, score, config, args.method_version, "holdout"),
            ))
        shadow_refs = []
        shadow_scores = []
        for path in args.shadow_run:
            if shadow_baseline_score is None:
                raise ValueError("shadow runs require --shadow-baseline-run")
            ref, score = read_score(path, "shadow")
            shadow_refs.append(ref)
            shadow_scores.append((
                ref,
                score,
                evaluate_method_run(shadow_baseline_score, score, config, args.method_version, "shadow"),
            ))
        baseline_refs = [baseline_ref]
        if holdout_baseline_ref:
            baseline_refs.append(holdout_baseline_ref)
        if shadow_baseline_ref:
            baseline_refs.append(shadow_baseline_ref)
        all_run_ids = [item["run_id"] for item in baseline_refs + fixed_refs + holdout_refs + shadow_refs]
        if len(all_run_ids) != len(set(all_run_ids)):
            raise ValueError("fixed, holdout and shadow runs must have distinct run_id values")
        if any(result.get("status") != "pass" for result in all_fixed_results):
            raise ValueError("fixed replay regression gate failed")
        if not args.replay_only:
            if not holdout_refs or not shadow_refs or not holdout_baseline_ref or not shadow_baseline_ref:
                raise ValueError("approved promotion requires split-specific holdout and shadow baselines/runs")
            if any(result.get("status") != "pass" for _, _, result in holdout_scores + shadow_scores):
                raise ValueError("holdout or shadow regression gate failed")
            if method.get("status") == "probe_only":
                raise ValueError("probe_only method cannot be approved directly")
            if not args.best_config:
                raise ValueError("approved promotion requires --best-config")

        status = "replay_only" if args.replay_only else "approved"
        promotion = {
            "record_type": "promotion",
            "schema_version": 1,
            "record_id": f"promotion-{args.method_id}-{args.applicability_bucket}",
            "status": status,
            "created_at": date.today().isoformat(),
            "method_id": args.method_id,
            "method_version": args.method_version,
            "applicability_bucket": args.applicability_bucket,
            "requirement_id": args.requirement_id,
            "task_id": args.task_id,
            "baseline_runs": {
                "fixed": baseline_ref,
                "holdout": holdout_baseline_ref,
                "shadow": shadow_baseline_ref,
            },
            "fixed_runs": fixed_refs,
            "holdout_runs": holdout_refs,
            "shadow_runs": shadow_refs,
            "guard_conditions": {
                "replay_only": args.replay_only,
                "hard_gate_required": True,
                "independent_runs_required": True,
                "near_far_protection_required": True,
            },
        }
        promotions_path = Path(args.promotions)
        existing_promotions = load_jsonl(promotions_path, "promotion") if promotions_path.exists() else {}
        if promotion["record_id"] in existing_promotions:
            raise ValueError(f"promotion already exists: {promotion['record_id']}")
        if not args.replay_only:
            promoted_method = dict(method)
            promoted_method["status"] = "promoted"
            replace_record(Path(args.methods), "method", args.method_id, promoted_method)
            copy_promoted_config(method, args.applicability_bucket, Path(args.best_config), args.force_best_config)
        append_record(promotions_path, promotion)
        Path(args.out).write_text(json.dumps(promotion, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"wrote {args.out}: status={status}")
        return 0
    except (ValueError, OSError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
