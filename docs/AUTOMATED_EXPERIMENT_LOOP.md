# D455 Automatic Experiment Loop

The automatic loop lets scripts run experiments, score results, build leaderboards, and hand evidence to Codex. Codex may propose the next candidates or a narrow code change, but the evaluator remains the judge.

## Roles

| Role | Responsibility |
|---|---|
| Experiment Runner | Runs a candidate config against one or more cases and writes `analysis_runs/<run_id>` |
| Evaluator | Reads metrics and writes `run_score.json`, `run_summary.md`, and `failure_report.md` |
| Candidate Generator | Produces the next `configs/candidates/*.json` from a search space or handoff |
| Codex Improver | Reads scores/failures and proposes config or code changes with evidence |

## Flow

```text
baseline config
  -> generate candidate configs
  -> run replay cases
  -> score each run
  -> write leaderboard.csv and pareto_front.csv
  -> generate .codex_handoff/round_xxx.md
  -> Codex proposes next candidates or a narrow patch
  -> repeat
```

This is not AI guessing. It is evidence-driven iteration: fixed metrics, fixed gates, reproducible run directories, and explicit rollback points.

## Permission Levels

| Level | Codex Permissions | Output |
|---|---|---|
| 1. Read-only analyst | Read leaderboards, scores, events, notes, sample frames | `docs/codex_analysis/<date>_diagnosis.md` |
| 2. Config generator | Edit `configs/candidates/*.json` and `eval/search_space.yaml` | Candidate JSON files |
| 3. Code improvement PR | Edit code/docs/scripts after measured failure evidence | Narrow patch plus validation |

Start with Level 1 and Level 2 until the replay/evaluation loop has run repeatedly. Open Level 3 only when parameter search plateaus or the metrics expose a missing capability.

## First Local Loop

```powershell
python scripts/generate_candidates.py --search-space eval/search_space.yaml --parent configs/baseline/default.json --count 12 --out configs/candidates/round_001
python scripts/run_batch.py --candidates configs/candidates/round_001 --cases eval/cases.yaml --out .codex_handoff/round_001_plan --dry-run
python scripts/convert_exports_to_analysis_run.py --run-dir analysis_runs\<run_id>
python scripts/score_run.py --runs analysis_runs
python scripts/select_winners.py --runs analysis_runs --out leaderboards
python scripts/make_codex_handoff.py --leaderboard leaderboards/leaderboard.csv --runs analysis_runs --out .codex_handoff/round_001.md
```

`run_batch.py` is intentionally dry-run friendly until replay input is implemented. Keep dry-run command plans under `.codex_handoff/` so they do not look like measured `analysis_runs` data. Do not treat a generated command plan as a completed experiment. The dry-run plan now includes a `convert_command` column; after manually running one planned D455 command, run that converter command to create the minimum standard package before scoring.

Round 001 has been initialized this way: 12 candidates were generated under `configs/candidates/round_001/`, and a 60-command dry-run plan was written to `.codex_handoff/round_001_plan/command_plan.csv`. This round has no winner because there is no deterministic replay dataset and no real `run_score.json` evidence yet.

The export converter still supports the old final-frame bridge, but smoke scoring should now prefer periodic analysis export:

```powershell
.\x64\Release\D455.exe --cluster-map --final-segmentation-export=analysis_runs\<run_id>\final_segmentation --cluster-map-export=analysis_runs\<run_id>\cluster_map --profile-csv=analysis_runs\<run_id>\profile.csv --analysis-export-every-n=30 --max-frames=120 --no-display
python scripts\convert_exports_to_analysis_run.py --run-dir=analysis_runs\<run_id>
```

`--analysis-export-every-n=N` writes frame-suffixed metadata such as `cluster_map_frame_000030_metadata.json` and `final_segmentation_frame_000030_metadata.json`. The converter combines those metadata files with matching rows from `profile.csv` to produce multi-row `frame_metrics.csv` and multi-frame `cluster_metrics.jsonl`. This is still not a deterministic benchmark until replay exists, but it is no longer limited to one final frame.

Directory replay is the first deterministic input path:

```text
datasets/<case_id>/
  frames/
    000000_color.png
    000000_depth16.png
    000000_ir_left.png
    000000_ir_right.png
  case_manifest.json
```

`depth16.png` is interpreted as unsigned 16-bit millimeters with `depth_scale=0.001`. `ir_left` and `ir_right` are optional for basic depth/color replay, but stereo contour distance needs both IR images. D455 can run this path without a connected camera:

```powershell
.\x64\Release\D455.exe --replay-dir=datasets\<case_id> --cluster-map --final-segmentation-export=analysis_runs\<run_id>\final_segmentation --cluster-map-export=analysis_runs\<run_id>\cluster_map --profile-csv=analysis_runs\<run_id>\profile.csv --analysis-export-every-n=30 --max-frames=120 --no-display
```

`run_batch.py --execute` now runs the full replay chain: D455 execution, converter, and `score_run.py`. Use `--ignore-first-n-frames=30` when converting/scoring replay exports to drop startup or tracker-warmup frames from metrics.

```powershell
python scripts\run_batch.py --candidates configs\candidates\round_001 --cases eval\cases.yaml --analysis-export-every-n=30 --ignore-first-n-frames=30 --execute
```

Capture the first fixed cases directly from D455 before running measured optimization:

```powershell
.\tools\Capture-ReplayCases.ps1 -Frames 120 -Warmup 30
```

The capture helper pauses before each case, writes replay frames under `datasets/<case_id>/`, captures both IR streams by passing `--quality-segmentation --stereo-contour-distance`, and validates each case with `validate_replay_dataset.py`. After capture, edit each `case_manifest.json` with scene notes and expected behavior. Do not treat uncategorized live captures as benchmark truth. `datasets/` is local real input data and is ignored by Git by default; syncing datasets needs a separate explicit decision.

Run the first measured replay round as a small gate, not as the full 60-run grid:

```powershell
python scripts\validate_replay_dataset.py --case-dir datasets\near_single_object --case-dir datasets\far_cabinet --case-dir datasets\depth_hole_black_object --min-frames=120 --require-ir-left --require-ir-right
python scripts\run_batch.py --candidates configs\baseline\default.json --cases eval\cases.yaml --case-id near_single_object --case-id far_cabinet --case-id depth_hole_black_object --max-frames-override=120 --analysis-export-every-n=30 --ignore-first-n-frames=30 --validate-replay --require-replay-ir --execute
python scripts\run_batch.py --candidates configs\candidates\round_001 --candidate-id candidate_0001 --candidate-id candidate_0002 --cases eval\cases.yaml --case-id near_single_object --case-id far_cabinet --case-id depth_hole_black_object --max-frames-override=120 --analysis-export-every-n=30 --ignore-first-n-frames=30 --validate-replay --require-replay-ir --execute
```

`validate_replay_dataset.py` checks color/depth frame pairs, PNG bit depth, dimensions, manifest format, and optional left/right IR. `run_batch.py` supports `--case-id` and `--candidate-id` filters for bounded smoke rounds, `--max-frames-override` to keep the first fixed cases at 60 to 120 frames, `--validate-replay` to gate each replay input before execution, and `--skip-missing-replay` only for command-chain validation before all datasets are captured. A skipped replay run is not scoring evidence.

When converting smoke exports, `convert_exports_to_analysis_run.py` updates template or incomplete `run_manifest.json` / `config_snapshot.json` records with the real `run_id`, `candidate_id`, `case_id`, `evaluation_mode=converted_export`, input paths, and command line. Existing non-template records are preserved unless `--update-manifest` or `--overwrite-config-snapshot` is passed.

## Gates

Hard fail conditions come from `eval/score_weights.yaml` and `docs/EVALUATION_FEATURES.md`.

Required regression checks:

```text
overall_score_delta >= +2.0
hard_fail = false
near_score_delta >= -1.0
far_score_delta >= -1.0
total_frame_ms_p95 <= baseline * 1.25
```

Leaderboards may include both passing and failing runs for diagnosis. Winner-like outputs must be pass-gated: `select_winners.py` writes `pareto_front.csv` from `pass=true` runs only, and writes `no_pass_candidate` for each slot when no passing run exists. `make_codex_handoff.py` follows the same rule for its current-best section.

## Data Splits

Use train/validation/test splits in `eval/cases.yaml`.

Codex may use train and validation summaries for proposal generation. Do not repeatedly tune against test cases.

## Current Minimal Landing

The current repository provides:

- `AGENTS.md` for durable project rules.
- Baseline/candidate config templates under `configs/`.
- Evaluation settings under `eval/`.
- Script skeletons under `scripts/`.
- Codex handoff and prompt templates.

The next implementation step is real replay support in `D455.exe` or a converter that turns recorded inputs into deterministic case directories.

## Codex Automation Surfaces

The loop uses official Codex surfaces conservatively:

- `AGENTS.md` stores durable repository rules. Codex reads it as project guidance at the start of a run.
- `codex exec` can run non-interactively in scripts, with explicit sandbox and approval settings.
- The Codex GitHub Action can run Codex in CI/CD, but should start as read-only review with limited triggers and sanitized prompt input.

Security rules:

- Use read-only or config-only rounds before code-edit rounds.
- Do not expose API keys to repository-controlled setup steps.
- Keep Codex review workflows limited to trusted manual triggers until prompt-injection risks are handled.
- Treat generated candidates as proposals until `run_score.json`, leaderboards, and regression gates pass.

Official references:

- AGENTS.md: https://developers.openai.com/codex/guides/agents-md
- Non-interactive mode: https://developers.openai.com/codex/noninteractive
- Codex GitHub Action: https://developers.openai.com/codex/github-action
