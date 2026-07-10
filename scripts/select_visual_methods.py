#!/usr/bin/env python3
"""Select registered methods applicable to one visual scenario bucket."""

import argparse
import json
import sys
from pathlib import Path


STATUS_PRIORITY = {
    "promoted": 0,
    "replay_only": 1,
    "shadow": 2,
    "candidate": 3,
    "probe_only": 4,
    "paused": 5,
    "rolled_back": 99,
}


def load_methods(path):
    if not path.exists():
        raise ValueError(f"method registry does not exist: {path}")
    methods = []
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        text = raw.strip()
        if not text or text.startswith("#"):
            continue
        try:
            record = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path}:{line_number}: invalid JSON: {exc.msg}") from exc
        if record.get("record_type") == "method":
            methods.append(record)
    return methods


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--methods", default="eval/visual_evolution/methods.jsonl")
    parser.add_argument("--applicability-bucket", required=True)
    parser.add_argument("--method-id", action="append", default=[])
    parser.add_argument("--include-probes", action="store_true")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    try:
        methods = load_methods(Path(args.methods))
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    selected = []
    warnings = []
    requested = set(args.method_id)
    allowed_statuses = {"promoted", "replay_only", "shadow", "candidate"}
    if args.include_probes:
        allowed_statuses.add("probe_only")
    for method in methods:
        if requested and method.get("method_id") not in requested:
            continue
        if args.applicability_bucket not in method.get("applicability_buckets", []):
            if method.get("method_id") in requested:
                warnings.append(
                    f"{method.get('method_id')} does not declare bucket {args.applicability_bucket}"
                )
            continue
        if method.get("status") not in allowed_statuses:
            continue
        selected.append(method)

    selected.sort(key=lambda item: (
        STATUS_PRIORITY.get(item.get("status"), 50),
        item.get("method_id", ""),
    ))
    result = {
        "schema_version": 1,
        "applicability_bucket": args.applicability_bucket,
        "include_probes": args.include_probes,
        "baseline_method_ids": [
            item["method_id"] for item in selected
            if item.get("status") in {"promoted", "replay_only"}
        ],
        "probe_method_ids": [
            item["method_id"] for item in selected
            if item.get("status") in {"candidate", "probe_only"}
        ],
        "eligible_method_ids": [item["method_id"] for item in selected],
        "warnings": warnings,
        "methods": selected,
    }
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {out_path}: eligible={len(selected)}")
    if warnings:
        for warning in warnings:
            print(f"warning: {warning}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
