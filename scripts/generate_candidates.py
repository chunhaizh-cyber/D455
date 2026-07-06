#!/usr/bin/env python3
import argparse
import json
import random
import re
from pathlib import Path


def parse_values(text):
    params = {}
    current = None
    in_parameters = False
    for line in text.splitlines():
        if line.strip() == "parameters:":
            in_parameters = True
            continue
        if line.strip() == "run_modes:":
            in_parameters = False
        if not in_parameters:
            continue
        m_key = re.match(r"^  ([A-Za-z0-9_]+):\s*$", line)
        if m_key:
            current = m_key.group(1)
            continue
        m_values = re.match(r"^    values:\s*\[(.*)\]\s*$", line)
        if current and m_values:
            raw = [x.strip() for x in m_values.group(1).split(",") if x.strip()]
            values = []
            for x in raw:
                try:
                    values.append(int(x))
                except ValueError:
                    values.append(x.strip('"\''))
            params[current] = values
            current = None
    return params


def load_parent(path):
    with Path(path).open("r", encoding="utf-8") as f:
        return json.load(f)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--search-space", default="eval/search_space.yaml")
    parser.add_argument("--parent", default="configs/baseline/default.json")
    parser.add_argument("--count", type=int, default=8)
    parser.add_argument("--out", required=True)
    parser.add_argument("--seed", type=int, default=455)
    args = parser.parse_args()

    random.seed(args.seed)
    params = parse_values(Path(args.search_space).read_text(encoding="utf-8"))
    parent = load_parent(args.parent)
    parent_args = list(parent.get("args", []))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    names = sorted(params)
    for index in range(1, args.count + 1):
        changed = random.sample(names, k=min(5, len(names)))
        args_list = list(parent_args)
        expected = []
        for name in changed:
            value = random.choice(params[name])
            args_list.append(f"--{name.replace('_', '-')}={value}")
            expected.append(f"test {name}={value}")
        candidate_id = f"candidate_{index:04d}"
        data = {
            "candidate_id": candidate_id,
            "parent": parent.get("config_id", args.parent),
            "purpose": "Generated candidate for measured parameter search.",
            "args": args_list,
            "expected_effect": expected,
            "risk": [
                "May overfit current train cases.",
                "Requires run_score.json comparison before use."
            ],
        }
        path = out / f"{candidate_id}.json"
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
