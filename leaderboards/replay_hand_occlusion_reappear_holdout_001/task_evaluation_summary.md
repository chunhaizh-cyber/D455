# Independent Hand-Occlusion Holdout Decision

## Scope

- Task: `task-hand-occlusion-local-refresh-001`
- Historical case: `hand_occlusion_reappear`
- Independent holdout: `hand_occlusion_reappear_holdout`
- Baseline: `candidate_0030`
- Probes: `candidate_0034`, `candidate_0035`

## Holdout Results

| Candidate | Hard gate | Score | Near | Far | p95 ms | Max ms | Frames >100 ms |
|---|---:|---:|---:|---:|---:|---:|---:|
| candidate_0030 | pass | 94.163 | 20 | 10 | 65.566 | 71.332 | 0 |
| candidate_0034 | pass | 66.881 | 6 | 0 | 87.4554 | 102.926 | 1 |
| candidate_0035 | pass | 66.483 | 6 | 0 | 90.111 | 101.359 | 2 |

## Task Decision

`evaluate_visual_task.py` returned `status=fail`, `holdout_pass=false`, and
`promotion_eligible=false`. Both probes fail the historical fixed-replay
regression gate and their split-specific holdout regression gate against
candidate_0030. The independent holdout shows a large near/far quality
regression for both probes.

- Do not promote candidate_0034 or candidate_0035.
- Keep candidate_0030 as the current motion baseline, not as proof that local ROI refresh is solved.
- Keep `visual-need-hand-occlusion-local-refresh` unsatisfied and prepare a new controlled method task.
- Do not enter P7 production shadow selection until a method passes fixed replay, independent holdout, and the required shadow gate.
