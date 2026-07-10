#!/usr/bin/env python3
"""Evaluate a prepared visual task without promoting a method."""

import argparse
import json
import sys
from pathlib import Path

from compare_runs import evaluate_regression, load_score
from score_run import load_score_config


def read_task(path):
    try:
        task = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read task: {exc}") from exc
    if task.get("record_type") != "task":
        raise ValueError("task record_type must be 'task'")
    return task


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True)
    parser.add_argument("--baseline", required=True, help="Baseline run_score.json")
    parser.add_argument("--candidate", action="append", required=True, help="Candidate run_score.json; repeatable")
    parser.add_argument("--holdout", action="append", default=[], help="Holdout run_score.json; repeatable")
    parser.add_argument("--shadow", action="append", default=[], help="Shadow run_score.json; repeatable")
    parser.add_argument("--weights", default="eval/score_weights.yaml")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    try:
        task = read_task(args.task)
        baseline = load_score(args.baseline)
        config = load_score_config(Path(args.weights))
        candidate_scores = [load_score(path) for path in args.candidate]
        holdout_scores = [load_score(path) for path in args.holdout]
        shadow_scores = [load_score(path) for path in args.shadow]
    except (ValueError, OSError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    candidate_results = []
    for candidate in candidate_scores:
        result = evaluate_regression(baseline, candidate, config)
        candidate_results.append({
            "run_id": candidate.get("run_id"),
            "candidate_id": candidate.get("candidate_id"),
            "result": result,
        })
    fixed_pass = any(item["result"].get("status") == "pass" for item in candidate_results)
    holdout_pass = bool(holdout_scores) and all(score.get("pass") for score in holdout_scores)
    shadow_pass = bool(shadow_scores) and all(score.get("pass") for score in shadow_scores)
    fixed_run_ids = {baseline.get("run_id")} | {score.get("run_id") for score in candidate_scores}
    holdout_run_ids = {score.get("run_id") for score in holdout_scores}
    shadow_run_ids = {score.get("run_id") for score in shadow_scores}
    leakage = bool(
        (fixed_run_ids & holdout_run_ids)
        or (fixed_run_ids & shadow_run_ids)
        or (holdout_run_ids & shadow_run_ids)
    )
    independent_holdout_required = bool(task.get("validation_contract", {}).get("independent_holdout_required", True))
    shadow_required = bool(task.get("validation_contract", {}).get("shadow_required_before_production", True))

    if leakage:
        status = "insufficient_evidence"
    elif not fixed_pass:
        status = "fail"
    elif independent_holdout_required and not holdout_pass:
        status = "insufficient_evidence"
    elif shadow_required and not shadow_pass:
        status = "insufficient_evidence"
    else:
        status = "pass"

    output = {
        "record_type": "task_evaluation",
        "schema_version": 1,
        "task_id": task.get("record_id"),
        "requirement_id": task.get("requirement_id"),
        "status": status,
        "promotion_eligible": status == "pass",
        "fixed_replay_pass": fixed_pass,
        "holdout_pass": holdout_pass,
        "shadow_pass": shadow_pass,
        "data_leakage": leakage,
        "candidate_results": candidate_results,
        "holdout_run_ids": [score.get("run_id") for score in holdout_scores],
        "shadow_run_ids": [score.get("run_id") for score in shadow_scores],
    }
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {out_path}: status={status} promotion_eligible={output['promotion_eligible']}")
    return 0 if status in {"pass", "insufficient_evidence"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
