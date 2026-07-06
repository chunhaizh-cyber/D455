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


def filter_cases(cases, selected_case_ids):
    if not selected_case_ids:
        return cases
    selected = set(selected_case_ids)
    return [case for case in cases if case["case_id"] in selected]


def filter_candidates(paths, selected_candidate_ids):
    if not selected_candidate_ids:
        return paths
    selected = set(selected_candidate_ids)
    filtered = []
    for path in paths:
        cand = json.loads(path.read_text(encoding="utf-8"))
        if candidate_id_for(cand, path) in selected:
            filtered.append(path)
    return filtered


def candidate_id_for(candidate, path):
    if candidate.get("candidate_id"):
        return candidate["candidate_id"]
    if candidate.get("config_id"):
        return candidate["config_id"].replace("/", "_").replace("\\", "_")
    return Path(path).stem


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidates", required=True)
    parser.add_argument("--cases", default="eval/cases.yaml")
    parser.add_argument("--out", default="analysis_runs")
    parser.add_argument("--analysis-root", default="analysis_runs")
    parser.add_argument("--exe", default=".\\x64\\Release\\D455.exe")
    parser.add_argument("--analysis-export-every-n", type=int, default=0)
    parser.add_argument("--ignore-first-n-frames", type=int, default=0)
    parser.add_argument("--max-frames-override", type=int, default=0)
    parser.add_argument("--candidate-id", action="append", default=[])
    parser.add_argument("--case-id", action="append", default=[])
    parser.add_argument("--skip-missing-replay", action="store_true")
    parser.add_argument("--validate-replay", action="store_true")
    parser.add_argument("--require-replay-ir", action="store_true")
    parser.add_argument("--require-reviewed-manifest", action="store_true")
    parser.add_argument("--weights", default="eval\\score_weights.yaml")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    plan = out / "command_plan.csv"
    rows = []
    candidates = filter_candidates(candidate_files(args.candidates), args.candidate_id)
    cases = filter_cases(parse_cases(args.cases), args.case_id)
    for cand_path in candidates:
        cand = json.loads(cand_path.read_text(encoding="utf-8"))
        candidate_id = candidate_id_for(cand, cand_path)
        for case in cases:
            run_id = f"{case['case_id']}_{candidate_id}"
            analysis_dir = str(Path(args.analysis_root) / run_id)
            cluster_map_base = f"{analysis_dir}\\cluster_map"
            final_segmentation_base = f"{analysis_dir}\\final_segmentation"
            profile_csv = f"{analysis_dir}\\profile.csv"
            max_frames = str(args.max_frames_override if args.max_frames_override > 0 else case["frames"])
            command = " ".join([
                args.exe,
                *cand.get("args", []),
                *([f"--replay-dir={case['replay']}"] if case["replay"] else []),
                f"--max-frames={max_frames}",
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
                f"--candidate-id={candidate_id}",
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
                "candidate_id": candidate_id,
                "case_id": case["case_id"],
                "replay": case["replay"],
                "max_frames": max_frames,
                "analysis_dir": analysis_dir,
                "command": command,
                "convert_command": convert_command,
                "score_command": score_command,
            })

    if args.candidate_id and not candidates:
        raise SystemExit(f"No candidate matched --candidate-id: {', '.join(args.candidate_id)}")
    if args.case_id and not cases:
        raise SystemExit(f"No case matched --case-id: {', '.join(args.case_id)}")

    with plan.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "run_id",
                "candidate_id",
                "case_id",
                "replay",
                "max_frames",
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
            if args.skip_missing_replay:
                print(f"skip missing replay for {row['run_id']}: {row['replay']}")
                continue
            raise SystemExit(f"Replay path does not exist for {row['case_id']}: {row['replay']}")
        if args.validate_replay:
            validation_command = [
                sys.executable,
                "scripts\\validate_replay_dataset.py",
                f"--case-dir={row['replay']}",
                f"--min-frames={row['max_frames']}",
            ]
            if args.require_replay_ir:
                validation_command.extend(["--require-ir-left", "--require-ir-right"])
            if args.require_reviewed_manifest:
                validation_command.append("--require-reviewed-manifest")
            print(f"validate replay: {row['case_id']}")
            result = subprocess.run(validation_command)
            if result.returncode != 0:
                raise SystemExit(f"replay validation failed for {row['case_id']} with exit code {result.returncode}")
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
