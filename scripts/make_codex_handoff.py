#!/usr/bin/env python3
import argparse
import csv
from datetime import datetime
from pathlib import Path


def read_leaderboard(path):
    path = Path(path)
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--leaderboard", default="leaderboards/leaderboard.csv")
    parser.add_argument("--runs", default="analysis_runs")
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    rows = read_leaderboard(args.leaderboard)
    rows_sorted = sorted(rows, key=lambda r: float(r.get("total_score") or 0), reverse=True)
    best = rows_sorted[0] if rows_sorted else {}
    round_id = datetime.now().strftime("round_%Y%m%d_%H%M%S")
    out = Path(args.out or f".codex_handoff/{round_id}.md")
    out.parent.mkdir(parents=True, exist_ok=True)

    failing = [r for r in rows_sorted if r.get("pass") in ("0", "False", "false")]
    lines = [
        f"# Codex Handoff: {round_id}",
        "",
        "## Goal",
        "Improve D455 full-frame pixel clustering with measured evidence.",
        "",
        "Priority order:",
        "1. Preserve far visual contours when depth is unreliable.",
        "2. Lower unknown_percent.",
        "3. Avoid near-field merge events.",
        "4. Keep p95 frame time under the configured budget.",
        "",
        "## Current Best",
        f"- candidate_id: {best.get('candidate_id', '')}",
        f"- run_id: {best.get('run_id', '')}",
        f"- total_score: {best.get('total_score', '')}",
        f"- far_score: {best.get('far_score', '')}",
        f"- frame_ms_p95: {best.get('frame_ms_p95', '')}",
        "",
        "## Main Failures",
    ]
    if failing:
        for row in failing[:8]:
            lines.append(f"- run_id={row.get('run_id','')} score={row.get('total_score','')} unknown={row.get('unknown_p50','')} merge={row.get('merge_events','')} contour_lost={row.get('contour_lost_events','')}")
    else:
        lines.append("- No failing rows in leaderboard.")

    lines += [
        "",
        "## Evidence",
        f"- {args.leaderboard}",
        f"- {args.runs}/**/run_score.json",
        f"- {args.runs}/**/events.csv",
        "",
        "## Allowed Changes",
        "For config-only rounds, edit only:",
        "- configs/candidates/*.json",
        "- eval/search_space.yaml",
        "",
        "Do not edit:",
        "- D455.cpp",
        "- build files",
        "- analysis_runs real data",
        "",
        "## Required Output",
        "Create 8 new candidate configs with candidate_id, parent, purpose, args, expected_effect, and risk.",
        "",
    ]
    out.write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
