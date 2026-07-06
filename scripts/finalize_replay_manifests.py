#!/usr/bin/env python3
import argparse
import json
import re
from pathlib import Path


def parse_scalar(value):
    value = value.strip()
    if value == "true":
        return True
    if value == "false":
        return False
    if re.fullmatch(r"-?\d+", value):
        return int(value)
    if re.fullmatch(r"-?\d+\.\d+", value):
        return float(value)
    return value.strip("\"'")


def parse_tags(value):
    value = value.strip()
    if not (value.startswith("[") and value.endswith("]")):
        return []
    inner = value[1:-1].strip()
    if not inner:
        return []
    return [item.strip() for item in inner.split(",")]


def parse_cases(path):
    cases = {}
    current = None
    in_expected = False
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        case_match = re.match(r"^\s*-\s+case_id:\s*([A-Za-z0-9_]+)\s*$", line)
        if case_match:
            current = {"case_id": case_match.group(1), "expected": {}}
            cases[current["case_id"]] = current
            in_expected = False
            continue
        if current is None:
            continue
        expected_match = re.match(r"^\s+expected:\s*$", line)
        if expected_match:
            in_expected = True
            continue
        top_match = re.match(r"^\s+([A-Za-z0-9_]+):\s*(.+)\s*$", line)
        if top_match and not in_expected:
            key, value = top_match.groups()
            if key == "tags":
                current[key] = parse_tags(value)
            elif key in {"replay", "frames", "weight"}:
                current[key] = parse_scalar(value)
            continue
        expected_value_match = re.match(r"^\s{6}([A-Za-z0-9_]+):\s*(.+)\s*$", line)
        if expected_value_match and in_expected:
            key, value = expected_value_match.groups()
            current["expected"][key] = parse_scalar(value)
            continue
        if line.strip() == "":
            in_expected = False
    return cases


def finalize_manifest(case_dir, case_contract, force):
    case_dir = Path(case_dir)
    manifest_path = case_dir / "case_manifest.json"
    if not manifest_path.exists():
        raise FileNotFoundError(f"missing manifest: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest_case_id = manifest.get("case_id") or case_dir.name
    if manifest_case_id != case_contract["case_id"]:
        raise ValueError(f"manifest case_id={manifest_case_id!r} does not match contract {case_contract['case_id']!r}")
    if manifest.get("reviewed") is True and not force:
        return {"case_id": manifest_case_id, "path": str(manifest_path), "updated": False}

    manifest["case_id"] = case_contract["case_id"]
    manifest["reviewed"] = True
    manifest["review_method"] = "auto_case_contract_v1"
    manifest["notes"] = (
        f"Auto-finalized from eval/cases.yaml for {case_contract['case_id']}. "
        "The capture directory is accepted as this fixed replay case by case_id contract; "
        "no manual visual ground-truth labeling is asserted."
    )
    manifest["expected"] = case_contract.get("expected", {})
    manifest["tags"] = case_contract.get("tags", [])
    manifest["case_contract"] = {
        "source": "eval/cases.yaml",
        "replay": case_contract.get("replay", ""),
        "frames": case_contract.get("frames"),
        "weight": case_contract.get("weight"),
    }
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return {"case_id": manifest_case_id, "path": str(manifest_path), "updated": True}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases-yaml", default="eval/cases.yaml")
    parser.add_argument("--dataset-root", default="datasets")
    parser.add_argument("--case-id", action="append", default=[])
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    contracts = parse_cases(args.cases_yaml)
    selected_case_ids = args.case_id or sorted(contracts.keys())
    results = []
    for case_id in selected_case_ids:
        if case_id not in contracts:
            raise SystemExit(f"case_id not found in {args.cases_yaml}: {case_id}")
        results.append(finalize_manifest(Path(args.dataset_root) / case_id, contracts[case_id], args.force))
    print(json.dumps({"finalized": results}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
