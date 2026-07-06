#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def load_score(path):
    with Path(path).open("r", encoding="utf-8") as f:
        return json.load(f)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", required=True, help="Baseline run_score.json")
    parser.add_argument("--candidate", required=True, help="Candidate run_score.json")
    args = parser.parse_args()

    base = load_score(args.baseline)
    cand = load_score(args.candidate)
    delta = cand.get("total_score", 0) - base.get("total_score", 0)
    print(f"baseline: {base.get('run_id')} score={base.get('total_score')}")
    print(f"candidate: {cand.get('run_id')} score={cand.get('total_score')}")
    print(f"total_score_delta: {delta:.3f}")
    for key in sorted(cand.get("scores", {})):
        b = base.get("scores", {}).get(key, 0)
        c = cand.get("scores", {}).get(key, 0)
        print(f"{key}_delta: {c - b:.3f}")


if __name__ == "__main__":
    main()
