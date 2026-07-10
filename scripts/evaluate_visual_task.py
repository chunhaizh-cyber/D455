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


def evaluate_candidates(baseline, scores, config, split_name):
    results = []
    seen_candidate_ids = set()
    baseline_case_id = baseline.get("case_id")
    for score in scores:
        candidate_id = score.get("candidate_id")
        if not candidate_id:
            raise ValueError(f"{split_name} score is missing candidate_id")
        if candidate_id in seen_candidate_ids:
            raise ValueError(f"duplicate {split_name} candidate_id: {candidate_id}")
        seen_candidate_ids.add(candidate_id)
        if baseline_case_id and score.get("case_id") and score.get("case_id") != baseline_case_id:
            raise ValueError(
                f"{split_name} baseline/candidate case mismatch: "
                f"{baseline_case_id} != {score.get('case_id')}"
            )
        results.append({
            "run_id": score.get("run_id"),
            "candidate_id": candidate_id,
            "result": evaluate_regression(baseline, score, config),
        })
    return results


def passing_candidate_ids(results):
    return {
        item["candidate_id"]
        for item in results
        if item["result"].get("status") == "pass"
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True)
    parser.add_argument("--baseline", required=True, help="Baseline run_score.json")
    parser.add_argument("--candidate", action="append", required=True, help="Candidate run_score.json; repeatable")
    parser.add_argument("--holdout-baseline", help="Baseline run_score.json for the holdout case")
    parser.add_argument("--holdout", action="append", default=[], help="Holdout run_score.json; repeatable")
    parser.add_argument("--shadow-baseline", help="Baseline run_score.json for the shadow case")
    parser.add_argument("--shadow", action="append", default=[], help="Shadow run_score.json; repeatable")
    parser.add_argument("--weights", default="eval/score_weights.yaml")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    try:
        task = read_task(args.task)
        baseline = load_score(args.baseline)
        config = load_score_config(Path(args.weights))
        candidate_scores = [load_score(path) for path in args.candidate]
        holdout_baseline = load_score(args.holdout_baseline) if args.holdout_baseline else None
        holdout_scores = [load_score(path) for path in args.holdout]
        shadow_baseline = load_score(args.shadow_baseline) if args.shadow_baseline else None
        shadow_scores = [load_score(path) for path in args.shadow]
        candidate_results = evaluate_candidates(baseline, candidate_scores, config, "fixed")
        holdout_results = (
            evaluate_candidates(holdout_baseline, holdout_scores, config, "holdout")
            if holdout_baseline is not None
            else []
        )
        shadow_results = (
            evaluate_candidates(shadow_baseline, shadow_scores, config, "shadow")
            if shadow_baseline is not None
            else []
        )
    except (ValueError, OSError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    independent_holdout_required = bool(task.get("validation_contract", {}).get("independent_holdout_required", True))
    shadow_required = bool(task.get("validation_contract", {}).get("shadow_required_before_production", True))
    fixed_pass_ids = passing_candidate_ids(candidate_results)
    holdout_pass_ids = passing_candidate_ids(holdout_results)
    shadow_pass_ids = passing_candidate_ids(shadow_results)
    fixed_pass = bool(fixed_pass_ids)
    holdout_evidence_complete = holdout_baseline is not None and bool(holdout_scores)
    shadow_evidence_complete = shadow_baseline is not None and bool(shadow_scores)
    fixed_and_holdout_pass_ids = fixed_pass_ids & holdout_pass_ids
    holdout_pass = holdout_evidence_complete and bool(fixed_and_holdout_pass_ids)
    pre_shadow_pass_ids = fixed_and_holdout_pass_ids if independent_holdout_required else fixed_pass_ids
    shadow_pass = shadow_evidence_complete and bool(pre_shadow_pass_ids & shadow_pass_ids)
    fixed_run_id_list = [baseline.get("run_id")] + [score.get("run_id") for score in candidate_scores]
    holdout_run_id_list = [score.get("run_id") for score in holdout_scores]
    if holdout_baseline is not None:
        holdout_run_id_list.append(holdout_baseline.get("run_id"))
    shadow_run_id_list = [score.get("run_id") for score in shadow_scores]
    if shadow_baseline is not None:
        shadow_run_id_list.append(shadow_baseline.get("run_id"))
    fixed_run_ids = set(fixed_run_id_list)
    holdout_run_ids = set(holdout_run_id_list)
    shadow_run_ids = set(shadow_run_id_list)
    leakage = bool(
        len(fixed_run_id_list) != len(fixed_run_ids)
        or len(holdout_run_id_list) != len(holdout_run_ids)
        or len(shadow_run_id_list) != len(shadow_run_ids)
        or (fixed_run_ids & holdout_run_ids)
        or (fixed_run_ids & shadow_run_ids)
        or (holdout_run_ids & shadow_run_ids)
    )
    eligible_candidate_ids = set(fixed_pass_ids)
    if leakage:
        status = "insufficient_evidence"
    elif not fixed_pass:
        status = "fail"
    elif independent_holdout_required and not holdout_evidence_complete:
        status = "insufficient_evidence"
    else:
        if independent_holdout_required:
            eligible_candidate_ids &= holdout_pass_ids
        if shadow_required and not shadow_evidence_complete:
            status = "insufficient_evidence"
        else:
            if shadow_required:
                eligible_candidate_ids &= shadow_pass_ids
            status = "pass" if eligible_candidate_ids else "fail"

    if status == "insufficient_evidence":
        eligible_candidate_ids.clear()

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
        "holdout_results": holdout_results,
        "shadow_results": shadow_results,
        "eligible_candidate_ids": sorted(eligible_candidate_ids),
        "fixed_baseline_run_id": baseline.get("run_id"),
        "holdout_baseline_run_id": holdout_baseline.get("run_id") if holdout_baseline else None,
        "shadow_baseline_run_id": shadow_baseline.get("run_id") if shadow_baseline else None,
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
