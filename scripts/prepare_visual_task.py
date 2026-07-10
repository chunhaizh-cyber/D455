#!/usr/bin/env python3
"""Prepare a single-goal visual task from a requirement and method registry."""

import argparse
import json
import sys
from datetime import date
from pathlib import Path


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


def append_record(path, record):
    existing = load_jsonl(path, record["record_type"]) if path.exists() else {}
    if record["record_id"] in existing:
        raise ValueError(f"record already exists: {record['record_id']}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8", newline="\n") as handle:
        handle.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")


def method_priority(method):
    return {
        "promoted": 0,
        "replay_only": 1,
        "shadow": 2,
        "candidate": 3,
        "probe_only": 4,
    }.get(method.get("status"), 50)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--requirement-id", required=True)
    parser.add_argument("--requirements", default="eval/visual_evolution/requirements.jsonl")
    parser.add_argument("--methods", default="eval/visual_evolution/methods.jsonl")
    parser.add_argument("--applicability-bucket", required=True)
    parser.add_argument("--baseline-method-id", required=True)
    parser.add_argument("--candidate-method-id", action="append", default=[])
    parser.add_argument("--case-id", action="append", required=True)
    parser.add_argument("--holdout-case-id", action="append", default=[])
    parser.add_argument("--max-frames", type=int, default=120)
    parser.add_argument("--task-id", default="")
    parser.add_argument("--out", required=True)
    parser.add_argument("--register", action="store_true")
    parser.add_argument("--task-registry", default="eval/visual_evolution/tasks.jsonl")
    args = parser.parse_args()

    try:
        requirements = load_jsonl(Path(args.requirements), "requirement")
        methods = load_jsonl(Path(args.methods), "method")
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    requirement = requirements.get(args.requirement_id)
    if requirement is None:
        print(f"ERROR: unknown requirement: {args.requirement_id}", file=sys.stderr)
        return 1
    baseline = methods.get(args.baseline_method_id)
    if baseline is None:
        print(f"ERROR: unknown baseline method: {args.baseline_method_id}", file=sys.stderr)
        return 1

    warnings = []
    if args.applicability_bucket not in baseline.get("applicability_buckets", []):
        warnings.append("baseline method is being used outside its registered applicability bucket")

    candidate_ids = list(dict.fromkeys(args.candidate_method_id))
    if not candidate_ids:
        eligible = [
            method for method in methods.values()
            if args.applicability_bucket in method.get("applicability_buckets", [])
            and method.get("status") not in {"paused", "rolled_back"}
        ]
        eligible.sort(key=lambda item: (method_priority(item), item.get("method_id", "")))
        candidate_ids = [method["method_id"] for method in eligible[:3]]
    if args.baseline_method_id not in candidate_ids:
        candidate_ids.insert(0, args.baseline_method_id)
    if len(candidate_ids) > 3:
        print("ERROR: a task can contain at most three candidate methods", file=sys.stderr)
        return 1
    for method_id in candidate_ids:
        if method_id not in methods:
            print(f"ERROR: unknown candidate method: {method_id}", file=sys.stderr)
            return 1
        if args.applicability_bucket not in methods[method_id].get("applicability_buckets", []):
            warnings.append(f"candidate method {method_id} does not declare the selected bucket")

    task_id = args.task_id or f"task-{args.requirement_id}-{args.applicability_bucket}"
    input_cases = [
        {"case_id": case_id, "split": "historical_selection", "replay_required": True}
        for case_id in args.case_id
    ]
    input_cases.extend(
        {"case_id": case_id, "split": "holdout", "replay_required": True}
        for case_id in args.holdout_case_id
    )
    has_holdout = bool(args.holdout_case_id)
    record = {
        "record_type": "task",
        "schema_version": 1,
        "record_id": task_id,
        "status": "prepared" if has_holdout else "blocked_missing_holdout",
        "created_at": date.today().isoformat(),
        "requirement_id": args.requirement_id,
        "primary_goal": requirement["goal"],
        "baseline_method_id": args.baseline_method_id,
        "candidate_method_ids": candidate_ids,
        "input_cases": input_cases,
        "resource_budget": {
            "max_candidates": len(candidate_ids),
            "max_frames": args.max_frames,
            "sequential": True,
        },
        "validation_contract": {
            "fixed_replay_required": True,
            "independent_holdout_required": True,
            "holdout_provided": has_holdout,
            "shadow_required_before_production": True,
            "g3_required": True,
            "hard_gate_required": True,
        },
        "fallback_conditions": [
            "硬门槛失败",
            "近场或远场保护失败",
            "轮廓丢失增加",
            "G3红线失败",
            "运行证据不完整",
        ],
        "selection_warnings": warnings,
    }
    if not has_holdout:
        record["blocked_reason"] = "未提供未参与候选选择的同类留出回放"

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if args.register:
        append_record(Path(args.task_registry), record)
    print(f"wrote {out_path}: status={record['status']} candidates={len(candidate_ids)}")
    for warning in warnings:
        print(f"warning: {warning}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
