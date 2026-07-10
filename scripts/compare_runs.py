#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

try:
    from score_run import load_score_config
except ImportError:  # pragma: no cover - supports importing from another cwd
    load_score_config = None


def load_score(path):
    with Path(path).open("r", encoding="utf-8") as f:
        return json.load(f)


def score_dimension(score, preferred, fallback):
    scores = score.get("scores", {})
    if preferred in scores:
        return scores[preferred], preferred
    if fallback in scores:
        return scores[fallback], fallback
    return None, None


def evaluate_regression(base, cand, config):
    gate = config.get("regression_gate", {})
    base_total = base.get("total_score")
    cand_total = cand.get("total_score")
    base_near, near_source = score_dimension(base, "near_score", "spatial_quality")
    cand_near, _ = score_dimension(cand, "near_score", "spatial_quality")
    base_far, far_source = score_dimension(base, "far_score", "far_retention")
    cand_far, _ = score_dimension(cand, "far_score", "far_retention")
    base_p95 = cand_p95 = None
    base_metrics = base.get("metrics", {})
    cand_metrics = cand.get("metrics", {})
    if base_metrics.get("total_frame_ms_p95") is not None:
        base_p95 = float(base_metrics["total_frame_ms_p95"])
    if cand_metrics.get("total_frame_ms_p95") is not None:
        cand_p95 = float(cand_metrics["total_frame_ms_p95"])

    missing = []
    for name, value in [
        ("baseline.total_score", base_total),
        ("candidate.total_score", cand_total),
        ("baseline.near_score", base_near),
        ("candidate.near_score", cand_near),
        ("baseline.far_score", base_far),
        ("candidate.far_score", cand_far),
        ("baseline.total_frame_ms_p95", base_p95),
        ("candidate.total_frame_ms_p95", cand_p95),
    ]:
        if value is None:
            missing.append(name)
    if missing:
        return {
            "status": "insufficient_evidence",
            "missing": missing,
            "near_source": near_source,
            "far_source": far_source,
        }

    multiplier = float(gate.get("frame_ms_p95_baseline_multiplier_max", 1.25))
    checks = {
        "candidate_pass": bool(cand.get("pass")),
        "hard_fail_reasons_empty": not cand.get("hard_fail_reasons"),
        "overall_score_delta": cand_total - base_total >= float(gate.get("overall_score_delta_min", 2.0)),
        "near_score_delta": cand_near - base_near >= float(gate.get("near_score_delta_min", -1.0)),
        "far_score_delta": cand_far - base_far >= float(gate.get("far_score_delta_min", -1.0)),
        "frame_ms_p95_budget": cand_p95 <= base_p95 * multiplier,
    }
    return {
        "status": "pass" if all(checks.values()) else "fail",
        "checks": checks,
        "deltas": {
            "overall_score": cand_total - base_total,
            "near_score": cand_near - base_near,
            "far_score": cand_far - base_far,
            "frame_ms_p95": cand_p95 - base_p95,
        },
        "near_source": near_source,
        "far_source": far_source,
        "baseline_frame_ms_p95_multiplier_limit": base_p95 * multiplier,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", required=True, help="Baseline run_score.json")
    parser.add_argument("--candidate", required=True, help="Candidate run_score.json")
    parser.add_argument("--weights", default="eval/score_weights.yaml")
    parser.add_argument("--out", default="")
    parser.add_argument("--enforce", action="store_true", help="Return non-zero when the regression gate fails")
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
    config = load_score_config(Path(args.weights)) if load_score_config else {}
    result = evaluate_regression(base, cand, config)
    print(f"regression_status: {result['status']}")
    if result.get("near_source") or result.get("far_source"):
        print(f"near_score_source: {result.get('near_source')}")
        print(f"far_score_source: {result.get('far_source')}")
    if result.get("missing"):
        print("missing_metrics: " + ", ".join(result["missing"]))
    if args.out:
        output = {
            "baseline_run_id": base.get("run_id"),
            "candidate_run_id": cand.get("run_id"),
            "result": result,
        }
        Path(args.out).write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if args.enforce and result["status"] != "pass":
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
