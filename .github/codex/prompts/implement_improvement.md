# Task

Implement one measured improvement for the D455 full-pixel clustering pipeline.

## Evidence

Read:

- `.codex_handoff/<round>.md`
- `leaderboards/leaderboard.csv`
- specified `analysis_runs/*/run_score.json`
- specified `analysis_runs/*/events.csv`

## Required Behavior

When a visual contour is present but depth is unreliable:

- Do not discard it.
- Emit `mode = ImageOnlyContour` or `mode = ApproxStereoContour`.
- Preserve `bbox_2d`, `contour_area_px`, and `center_2d`.
- Set `distance_confidence = 0` when stereo is unreliable.
- Do not claim `PreciseDepth3D`.

## Required Checks

Run the build and relevant replay evaluation before committing.

## Output

Summarize changed files, expected effect, test results, and known risks.
