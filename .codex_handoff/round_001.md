# Round 001 Automatic Optimization Handoff

## Scope

This round started the automatic optimization loop as a config-only round. It generated executable candidate configs and a dry-run command plan, but it did not run or score real experiments.

## Outputs

- Candidate configs: `configs/candidates/round_001/candidate_0001.json` through `candidate_0012.json`
- Dry-run plan: `.codex_handoff/round_001_plan/command_plan.csv`
- Planned commands: 60 total, from 12 candidates x 5 cases in `eval/cases.yaml`

## Important Fixes Before Candidate Generation

- `scripts/generate_candidates.py` now reads an explicit `arg` field from `eval/search_space.yaml`, so generated candidates use real D455 command-line switches instead of assuming every field maps by underscore-to-dash conversion.
- `eval/search_space.yaml` now binds each searchable parameter to its intended CLI switch.
- Baseline configs no longer include `--feature-profile=*`, because feature profiles are documented as a future switch and are not implemented in `D455.cpp` yet.

## Blocker

No measured winner can be selected in this round.

Current missing inputs:

- No `datasets/` directory exists for deterministic replay cases.
- No non-template `analysis_runs/**/run_score.json` files exist.
- `scripts/run_batch.py` currently supports dry-run command-plan generation only; actual replay execution is intentionally not implemented.

## Next Required Step

Implement one of these before score-based optimization:

1. Add deterministic replay support to `D455.exe` or `scripts/run_batch.py`, then run each case into `analysis_runs/<run_id>/`.
2. Create the fixed dataset/replay directories referenced by `eval/cases.yaml`.
3. Produce the required evaluation artifacts for each run, especially `frame_metrics.csv`, `cluster_metrics.jsonl`, `events.csv`, and `run_score.json`.

Until then, Codex must not claim a segmentation-quality improvement from Round 001. The only confirmed result is that candidate generation and command planning are reproducible.
