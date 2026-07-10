#!/usr/bin/env python3
"""Select methods that are valid for a scenario and evaluation split."""

import argparse
import json
import sys
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


def priority(method):
    return {"promoted": 0, "replay_only": 1, "shadow": 2, "candidate": 3, "probe_only": 4}.get(
        method.get("status"), 50
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--methods", default="eval/visual_evolution/methods.jsonl")
    parser.add_argument("--promotions", default="eval/visual_evolution/promotions.jsonl")
    parser.add_argument("--applicability-bucket", required=True)
    parser.add_argument("--evaluation-split", choices=["production", "replay", "probe"], default="production")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    try:
        methods = load_jsonl(Path(args.methods), "method")
        promotions = load_jsonl(Path(args.promotions), "promotion")
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    approved_by_method = {
        promotion.get("method_id"): promotion
        for promotion in promotions.values()
        if promotion.get("applicability_bucket") == args.applicability_bucket
        and promotion.get("status") in {"approved", "replay_only"}
    }
    selected = []
    for method in methods.values():
        if args.applicability_bucket not in method.get("applicability_buckets", []):
            continue
        method_status = method.get("status")
        promotion = approved_by_method.get(method.get("method_id"))
        if args.evaluation_split == "production":
            if method_status != "promoted" or not promotion or promotion.get("status") != "approved":
                continue
        elif args.evaluation_split == "replay":
            if method_status not in {"promoted", "replay_only"} or not promotion:
                continue
        else:
            if method_status not in {"promoted", "replay_only", "candidate", "probe_only"}:
                continue
        selected.append({
            "method": method,
            "promotion": promotion,
        })
    selected.sort(key=lambda item: (priority(item["method"]), item["method"].get("method_id", "")))
    result = {
        "schema_version": 1,
        "applicability_bucket": args.applicability_bucket,
        "evaluation_split": args.evaluation_split,
        "method_ids": [item["method"]["method_id"] for item in selected],
        "methods": selected,
        "no_applicable_method": not selected,
    }
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {out_path}: methods={len(selected)} no_applicable_method={result['no_applicable_method']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
