# AGENTS.md

## Project Goal

This repository targets full-frame pixel clustering for Intel RealSense D455 video:

- Near reliable depth range: return precise 3D spatial clusters.
- Far or unreliable depth range: preserve accurate 2D visual contours.
- When depth is unreliable, downgrade distance mode instead of inventing precise depth.
- Every pixel should belong to a cluster mode: `PreciseDepth3D`, `ApproxStereoContour`, `ImageOnlyContour`, `BackgroundPlane`, `FarBackground`, `DepthHoleCandidate`, or `Unknown`.

## Evaluation First

Do not claim an improvement without evidence from the fixed evaluation artifacts:

- `analysis_runs/<run_id>/run_manifest.json`
- `analysis_runs/<run_id>/config_snapshot.json`
- `analysis_runs/<run_id>/frame_metrics.csv`
- `analysis_runs/<run_id>/cluster_metrics.jsonl`
- `analysis_runs/<run_id>/events.csv`
- `analysis_runs/<run_id>/run_score.json`

For config or protocol changes, run the relevant script smoke tests. For C++ changes, run:

```powershell
msbuild .\D455.vcxproj /p:Configuration=Release /p:Platform=x64 /m
```

## Automatic Experiment Policy

Codex may propose candidates, explain failures, or implement narrow changes, but the evaluation scripts own the judgment.

Prefer the smallest measured change that improves score. Do not optimize only one case. Do not improve far retention by destroying near precision. Do not output precise 3D for unreliable far clusters. A correct downgrade to `ImageOnlyContour` is better than a false precise distance.

## Hard Gates

A candidate fails if any of these conditions are true:

- `cluster_coverage_percent_p50 < 95`
- `unknown_percent_p50 > 15`
- `total_frame_ms_p95 > 100`
- `merge_event_count` increases by more than 25% against baseline
- `contour_lost_event_count` increases by more than 25% against baseline

## Allowed Automatic Changes

Config-only rounds may edit:

- `configs/candidates/*.json`
- `eval/search_space.yaml`

Analysis-only rounds may edit:

- `docs/codex_analysis/`
- `.codex_handoff/`

Code rounds may edit:

- `D455.cpp`
- `tools/`
- `scripts/`
- `eval/`
- `docs/`
- `README.md`

Do not edit:

- `analysis_runs/` real run data, except generated score/summary outputs for that run
- `datasets/`
- secrets or credentials
- build artifacts

## Required Documentation

Every improvement direction must update `README.md` or a linked document under `docs/`, including the affected pipeline, default behavior changes, main switches/parameters, and validation method.

## End-of-Round Cloud Sync

At the end of every Codex round, if any project file changed, automatically synchronize the branch to the cloud:

1. Run the relevant validation for the files touched.
2. Stage only intentional project files.
3. Commit with a concise message.
4. Push the current branch to `origin`.
5. Report branch, commit, and validation in the final response.

Do not auto-sync generated camera recordings, real run data, build artifacts, secrets, or unrelated user changes. If validation fails, do not push; report the blocker and leave the worktree state clear.
