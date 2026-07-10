#!/usr/bin/env python3
"""Record a method rollback without deleting evidence."""

import argparse
import json
import sys
from datetime import date
from pathlib import Path


def load_jsonl(path, record_type):
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


def append_record(path, record):
    existing = load_jsonl(path, record["record_type"]) if path.exists() else {}
    if record["record_id"] in existing:
        raise ValueError(f"record already exists: {record['record_id']}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8", newline="\n") as handle:
        handle.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--method-id", required=True)
    parser.add_argument("--method-version", required=True)
    parser.add_argument("--applicability-bucket", required=True)
    parser.add_argument("--replacement-method-id", required=True)
    parser.add_argument("--triggered-by", action="append", required=True)
    parser.add_argument("--reason", required=True)
    parser.add_argument("--failure-metric", action="append", default=[])
    parser.add_argument("--methods", default="eval/visual_evolution/methods.jsonl")
    parser.add_argument("--rollbacks", default="eval/visual_evolution/rollbacks.jsonl")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    try:
        methods_path = Path(args.methods)
        methods = load_jsonl(methods_path, "method")
        method = methods.get(args.method_id)
        if method is None:
            raise ValueError(f"unknown method: {args.method_id}")
        replacement = methods.get(args.replacement_method_id)
        if replacement is None:
            raise ValueError(f"unknown replacement method: {args.replacement_method_id}")
        if args.method_version != method.get("method_version"):
            raise ValueError("method version does not match registry")
        if args.applicability_bucket not in method.get("applicability_buckets", []):
            raise ValueError("method does not declare the selected applicability bucket")
        triggered = [{"run_id": run_id, "split": "failure"} for run_id in args.triggered_by]
        failure_metrics = {}
        for item in args.failure_metric:
            if "=" not in item:
                raise ValueError(f"failure metric must be name=value: {item}")
            key, value = item.split("=", 1)
            failure_metrics[key] = value
        rollback = {
            "record_type": "rollback",
            "schema_version": 1,
            "record_id": f"rollback-{args.method_id}-{args.applicability_bucket}",
            "status": "recorded",
            "created_at": date.today().isoformat(),
            "method_id": args.method_id,
            "method_version": args.method_version,
            "applicability_bucket": args.applicability_bucket,
            "replacement_method_id": args.replacement_method_id,
            "triggered_by": triggered,
            "failure_metrics": failure_metrics,
            "reason": args.reason,
        }
        rollback_path = Path(args.rollbacks)
        existing_rollbacks = load_jsonl(rollback_path, "rollback") if rollback_path.exists() else {}
        if rollback["record_id"] in existing_rollbacks:
            raise ValueError(f"rollback already exists: {rollback['record_id']}")
        updated_method = dict(method)
        updated_method["status"] = "rolled_back"
        replace_record(methods_path, "method", args.method_id, updated_method)
        append_record(rollback_path, rollback)
        Path(args.out).write_text(json.dumps(rollback, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"wrote {args.out}: status=recorded")
        return 0
    except (ValueError, OSError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
