#!/usr/bin/env python3
import argparse
import csv
import json
import re
import subprocess
import sys
from pathlib import Path


def candidate_files(path):
    p = Path(path)
    if p.is_file():
        return [p]
    return sorted(p.glob("*.json"))


def parse_cases(path):
    text = Path(path).read_text(encoding="utf-8")
    cases = []
    current = None
    for line in text.splitlines():
        m_case = re.match(r"^\s*-\s+case_id:\s*([A-Za-z0-9_]+)\s*$", line)
        if m_case:
            current = {"case_id": m_case.group(1), "frames": "600", "replay": ""}
            cases.append(current)
            continue
        if current is None:
            continue
        m_replay = re.match(r"^\s+replay:\s*(.+)\s*$", line)
        if m_replay:
            current["replay"] = m_replay.group(1).strip()
        m_frames = re.match(r"^\s+frames:\s*(\d+)\s*$", line)
        if m_frames:
            current["frames"] = m_frames.group(1)
    return cases


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidates", required=True)
    parser.add_argument("--cases", default="eval/cases.yaml")
    parser.add_argument("--out", default="analysis_runs")
    parser.add_argument("--analysis-root", default="analysis_runs")
    parser.add_argument("--exe", default=".\\x64\\Release\\D455.exe")
    parser.add_argument("--analysis-export-every-n", type=int, default=0)
    parser.add_argument("--ignore-first-n-frames", type=int, default=0)
    parser.add_argument("--weights", default="eval\\score_weights.yaml")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    plan = out / "command_plan.csv"
    rows = []
    for cand_path in candidate_files(args.candidates):
        cand = json.loads(cand_path.read_text(encoding="utf-8"))
        for case in parse_cases(args.cases):
            run_id = f"{case['case_id']}_{cand['candidate_id']}"
            analysis_dir = str(Path(args.analysis_root) / run_id)
            cluster_map_base = f"{analysis_dir}\\cluster_map"
            final_segmentation_base = f"{analysis_dir}\\final_segmentation"
            profile_csv = f"{analysis_dir}\\profile.csv"
            command = " ".join([
                args.exe,
                *cand.get("args", []),
                *([f"--replay-dir={case['replay']}"] if case["replay"] else []),
                f"--max-frames={case['frames']}",
                *([f"--analysis-export-every-n={args.analysis_export_every_n}"] if args.analysis_export_every_n > 0 else []),
                f"--profile-csv={profile_csv}",
                f"--cluster-map-export={cluster_map_base}",
                f"--final-segmentation-export={final_segmentation_base}",
            ])
            convert_command = " ".join([
                "python",
                "scripts\\convert_exports_to_analysis_run.py",
                f"--run-dir={analysis_dir}",
                f"--cluster-map={cluster_map_base}_metadata.json",
                f"--final-segmentation={final_segmentation_base}_metadata.json",
                f"--profile-csv={profile_csv}",
                f"--candidate-config={cand_path}",
                f"--candidate-id={cand['candidate_id']}",
                f"--case-id={case['case_id']}",
                f"--run-id={run_id}",
                *([f"--ignore-first-n-frames={args.ignore_first_n_frames}"] if args.ignore_first_n_frames > 0 else []),
            ])
            score_command = " ".join([
                sys.executable,
                "scripts\\score_run.py",
                f"--runs={analysis_dir}",
                f"--weights={args.weights}",
            ])
            rows.append({
                "run_id": run_id,
                "candidate_id": cand["candidate_id"],
                "case_id": case["case_id"],
                "replay": case["replay"],
                "analysis_dir": analysis_dir,
                "command": command,
                "convert_command": convert_command,
                "score_command": score_command,
            })

    with plan.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "run_id",
                "candidate_id",
                "case_id",
                "replay",
                "analysis_dir",
                "command",
                "convert_command",
                "score_command",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {plan}")
    if args.dry_run:
        return
    if not args.execute:
        raise SystemExit("Execution is disabled by default. Re-run with --execute or use --dry-run.")

    for row in rows:
        if not row["replay"]:
            raise SystemExit(f"Case {row['case_id']} has no replay path.")
        if not Path(row["replay"]).exists():
            raise SystemExit(f"Replay path does not exist for {row['case_id']}: {row['replay']}")
        Path(row["analysis_dir"]).mkdir(parents=True, exist_ok=True)
        for label, command in [
            ("run", row["command"]),
            ("convert", row["convert_command"]),
            ("score", row["score_command"]),
        ]:
            print(f"{label}: {row['run_id']}")
            result = subprocess.run(command, shell=True)
            if result.returncode != 0:
                raise SystemExit(f"{label} failed for {row['run_id']} with exit code {result.returncode}")


if __name__ == "__main__":
    main()
