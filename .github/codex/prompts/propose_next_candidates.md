# Task

Improve the D455 full-pixel clustering system using measured evaluation results.

Read:

- `leaderboards/leaderboard.csv`
- `leaderboards/pareto_front.csv`
- latest `.codex_handoff/*.md`
- `eval/search_space.yaml`
- `configs/baseline/default.json`

Generate 8 new candidate config files under `configs/candidates/round_NEXT/`.

Do not edit C++ code.

## Optimization Goals

1. Preserve far visual contours when depth is unreliable.
2. Reduce `unknown_percent`.
3. Avoid near-field merge events.
4. Keep `total_frame_ms_p95` under 66ms when possible.

## Constraints

- Do not output precise 3D for unreliable far clusters.
- Prefer `ImageOnlyContour` over false distance.
- Do not worsen `merge_event_count` by more than 25%.
- Each candidate must change at most 5 parameters from its parent.

## Output

Create `candidate_XXXX.json` files. Each file must contain:

- `candidate_id`
- `parent`
- `purpose`
- `args`
- `expected_effect`
- `risk`
