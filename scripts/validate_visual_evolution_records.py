#!/usr/bin/env python3
"""Validate D455 visual capability evolution JSONL records."""

import argparse
import json
import sys
from pathlib import Path


RECORD_FILES = {
    "requirement": "requirements.jsonl",
    "task": "tasks.jsonl",
    "method": "methods.jsonl",
    "promotion": "promotions.jsonl",
    "rollback": "rollbacks.jsonl",
}

REQUIRED_FIELDS = {
    "requirement": [
        "goal", "target_feature", "current_value", "target_value",
        "dedupe_key", "source_evidence", "protection_conditions",
    ],
    "task": [
        "requirement_id", "primary_goal", "baseline_method_id",
        "candidate_method_ids", "input_cases", "resource_budget",
        "validation_contract", "fallback_conditions",
    ],
    "method": [
        "method_id", "method_version", "method_type",
        "applicability_buckets", "applicability_conditions", "input_evidence",
        "output_transitions", "failure_receipt", "fallback_path", "cost",
        "configuration_ref",
    ],
    "promotion": [
        "method_id", "method_version", "applicability_bucket", "requirement_id",
        "task_id", "fixed_runs", "holdout_runs", "shadow_runs",
        "guard_conditions",
    ],
    "rollback": [
        "method_id", "method_version", "applicability_bucket",
        "replacement_method_id", "triggered_by", "failure_metrics", "reason",
    ],
}

ALLOWED_STATUS = {
    "requirement": {"candidate", "confirmed", "satisfied", "blocked", "closed"},
    "task": {
        "prepared", "practicing", "trying", "directed_learning", "evaluating",
        "blocked_missing_holdout", "completed", "failed",
    },
    "method": {
        "probe_only", "candidate", "replay_only", "shadow", "promoted",
        "paused", "rolled_back",
    },
    "promotion": {"replay_only", "approved", "rolled_back"},
    "rollback": {"recorded", "resolved"},
}


def is_non_empty_string(value):
    return isinstance(value, str) and bool(value.strip())


def read_jsonl(path, record_type, errors):
    records = []
    if not path.exists():
        errors.append(f"{path}: missing record file")
        return records
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        text = raw.strip()
        if not text or text.startswith("#"):
            continue
        try:
            record = json.loads(text)
        except json.JSONDecodeError as exc:
            errors.append(f"{path}:{line_number}: invalid JSON: {exc.msg}")
            continue
        if not isinstance(record, dict):
            errors.append(f"{path}:{line_number}: record must be an object")
            continue
        record["_source"] = f"{path}:{line_number}"
        record["_expected_type"] = record_type
        records.append(record)
    return records


def require_fields(record, fields, errors):
    for field in fields:
        if field not in record:
            errors.append(f"{record['_source']}: missing field '{field}'")


def validate_common(record, record_type, errors):
    require_fields(record, ["record_type", "schema_version", "record_id", "status", "created_at"], errors)
    if record.get("record_type") != record_type:
        errors.append(
            f"{record['_source']}: record_type={record.get('record_type')!r} "
            f"does not match {record_type!r}"
        )
    if record.get("schema_version") != 1:
        errors.append(f"{record['_source']}: schema_version must be 1")
    if not is_non_empty_string(record.get("record_id")):
        errors.append(f"{record['_source']}: record_id must be a non-empty string")
    if record.get("status") not in ALLOWED_STATUS[record_type]:
        errors.append(f"{record['_source']}: invalid {record_type} status {record.get('status')!r}")
    if not is_non_empty_string(record.get("created_at")):
        errors.append(f"{record['_source']}: created_at must be a non-empty string")
    require_fields(record, REQUIRED_FIELDS[record_type], errors)


def validate_evidence(value, location, errors):
    if not isinstance(value, list):
        errors.append(f"{location}: evidence must be an array")
        return
    for index, item in enumerate(value):
        if not isinstance(item, dict) or not is_non_empty_string(item.get("run_id")):
            errors.append(f"{location}[{index}]: must contain a non-empty run_id")


def validate_record_shape(record, record_type, errors):
    validate_common(record, record_type, errors)
    source = record["_source"]
    if record_type == "requirement":
        if not isinstance(record.get("current_value"), dict):
            errors.append(f"{source}: current_value must be an object")
        if not isinstance(record.get("target_value"), dict):
            errors.append(f"{source}: target_value must be an object")
        validate_evidence(record.get("source_evidence"), f"{source}.source_evidence", errors)
    elif record_type == "task":
        candidates = record.get("candidate_method_ids")
        if not isinstance(candidates, list) or not 1 <= len(candidates) <= 3:
            errors.append(f"{source}: candidate_method_ids must contain 1 to 3 methods")
        if not isinstance(record.get("input_cases"), list) or not record.get("input_cases"):
            errors.append(f"{source}: input_cases must be a non-empty array")
        if not isinstance(record.get("fallback_conditions"), list):
            errors.append(f"{source}: fallback_conditions must be an array")
    elif record_type == "method":
        if record.get("record_id") != record.get("method_id"):
            errors.append(f"{source}: record_id must equal method_id")
        for field in ["applicability_buckets", "input_evidence", "output_transitions", "failure_receipt", "fallback_path"]:
            if not isinstance(record.get(field), list) or not record.get(field):
                errors.append(f"{source}: {field} must be a non-empty array")
        if not is_non_empty_string(record.get("configuration_ref")):
            errors.append(f"{source}: configuration_ref must be a non-empty string")
        validate_evidence(record.get("evidence", []), f"{source}.evidence", errors)
    elif record_type == "promotion":
        for field in ["fixed_runs", "holdout_runs", "shadow_runs"]:
            validate_evidence(record.get(field), f"{source}.{field}", errors)
        if record.get("status") == "approved":
            if not record.get("holdout_runs") or not record.get("shadow_runs"):
                errors.append(f"{source}: approved promotion requires holdout_runs and shadow_runs")
    elif record_type == "rollback":
        validate_evidence(record.get("triggered_by"), f"{source}.triggered_by", errors)


def index_records(records, errors):
    indexes = {record_type: {} for record_type in RECORD_FILES}
    global_ids = {}
    for record_type, items in records.items():
        for record in items:
            record_id = record.get("record_id")
            if not is_non_empty_string(record_id):
                continue
            if record_id in indexes[record_type]:
                errors.append(f"{record['_source']}: duplicate {record_type} record_id {record_id!r}")
            indexes[record_type][record_id] = record
            previous = global_ids.get(record_id)
            if previous and previous != record_type:
                errors.append(
                    f"{record['_source']}: record_id {record_id!r} already belongs to {previous}"
                )
            global_ids[record_id] = record_type
    return indexes


def validate_references(records, indexes, errors):
    requirements = indexes["requirement"]
    tasks = indexes["task"]
    methods = indexes["method"]

    for record in records["task"]:
        source = record["_source"]
        if record.get("requirement_id") not in requirements:
            errors.append(f"{source}: unknown requirement_id {record.get('requirement_id')!r}")
        method_ids = [record.get("baseline_method_id"), *record.get("candidate_method_ids", [])]
        for method_id in method_ids:
            if method_id not in methods:
                errors.append(f"{source}: unknown method_id {method_id!r}")

    for record in records["promotion"]:
        source = record["_source"]
        method = methods.get(record.get("method_id"))
        if method is None:
            errors.append(f"{source}: unknown promotion method_id {record.get('method_id')!r}")
        elif record.get("method_version") != method.get("method_version"):
            errors.append(f"{source}: method_version does not match registered method")
        if record.get("requirement_id") not in requirements:
            errors.append(f"{source}: unknown requirement_id {record.get('requirement_id')!r}")
        if record.get("task_id") not in tasks:
            errors.append(f"{source}: unknown task_id {record.get('task_id')!r}")
        if method and record.get("applicability_bucket") not in method.get("applicability_buckets", []):
            errors.append(f"{source}: applicability bucket is not declared by method")

    for record in records["rollback"]:
        source = record["_source"]
        if record.get("method_id") not in methods:
            errors.append(f"{source}: unknown rollback method_id {record.get('method_id')!r}")
        if record.get("replacement_method_id") not in methods:
            errors.append(f"{source}: unknown replacement_method_id {record.get('replacement_method_id')!r}")


def collect_evidence_refs(records):
    refs = []
    for record_type, items in records.items():
        for record in items:
            if record_type == "requirement":
                refs.extend(record.get("source_evidence", []))
            elif record_type == "method":
                refs.extend(record.get("evidence", []))
            elif record_type == "promotion":
                refs.extend(record.get("fixed_runs", []))
                refs.extend(record.get("holdout_runs", []))
                refs.extend(record.get("shadow_runs", []))
            elif record_type == "rollback":
                refs.extend(record.get("triggered_by", []))
    return refs


def find_run_score(run_id, analysis_root, cache):
    if run_id in cache:
        return cache[run_id]
    for path in Path(analysis_root).rglob("run_score.json"):
        try:
            score = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if score.get("run_id") == run_id:
            cache[run_id] = (path, score)
            return cache[run_id]
    cache[run_id] = None
    return None


def validate_local_evidence(records, analysis_root, errors):
    cache = {}
    for evidence in collect_evidence_refs(records):
        run_id = evidence.get("run_id")
        found = find_run_score(run_id, analysis_root, cache)
        if found is None:
            errors.append(f"evidence {run_id!r}: run_score.json not found under {analysis_root}")

    for record in records["method"]:
        config_path = Path(record["configuration_ref"])
        if not config_path.is_absolute():
            config_path = Path.cwd() / config_path
        if not config_path.exists():
            errors.append(f"{record['_source']}: configuration_ref does not exist: {record['configuration_ref']}")

    for record in records["promotion"]:
        for evidence in record.get("fixed_runs", []):
            found = find_run_score(evidence.get("run_id"), analysis_root, cache)
            if found is not None and record.get("status") != "rolled_back" and not found[1].get("pass"):
                errors.append(f"{record['_source']}: fixed run {evidence.get('run_id')!r} has pass=false")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default="eval/visual_evolution")
    parser.add_argument("--analysis-runs", default="analysis_runs")
    parser.add_argument("--check-evidence", action="store_true")
    args = parser.parse_args()

    root = Path(args.root)
    errors = []
    schema_path = root / "schema.json"
    if not schema_path.exists():
        errors.append(f"{schema_path}: missing schema")
    else:
        try:
            schema = json.loads(schema_path.read_text(encoding="utf-8"))
            if not isinstance(schema.get("$defs"), dict):
                errors.append(f"{schema_path}: missing $defs")
            for name in RECORD_FILES:
                if name not in schema.get("$defs", {}):
                    errors.append(f"{schema_path}: missing $defs.{name}")
        except json.JSONDecodeError as exc:
            errors.append(f"{schema_path}: invalid JSON: {exc.msg}")

    records = {
        record_type: read_jsonl(root / filename, record_type, errors)
        for record_type, filename in RECORD_FILES.items()
    }
    for record_type, items in records.items():
        for record in items:
            validate_record_shape(record, record_type, errors)
    indexes = index_records(records, errors)
    validate_references(records, indexes, errors)
    if args.check_evidence:
        validate_local_evidence(records, args.analysis_runs, errors)

    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1

    counts = ", ".join(f"{record_type}={len(items)}" for record_type, items in records.items())
    mode = " with local evidence" if args.check_evidence else ""
    print(f"visual evolution records valid{mode}: {counts}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
