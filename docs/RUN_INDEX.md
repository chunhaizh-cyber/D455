# Run Index

Append every synchronized analysis run here. Keep one row per run id; create a new run id instead of overwriting large files from an older run.

| run_id | branch | commit | scene | purpose | result | notes |
|---|---|---|---|---|---|---|
| _template | _template | _template | _template | Protocol example | Not a real run | Copy `analysis_runs/_template` for new runs |
| replay_small_gate_far_scoring_fix | codex/imu-gravity-check | 23f7893 | near/far/depth-hole replay gate | compare candidate_0013 vs 0014 after far-retention scoring fix | candidate_0014 wins small gate | static replay only; no motion case yet |
| replay_small_matrix_static_001 | codex/imu-gravity-check | 23f7893 | near/far/depth-hole static replay matrix | compare baseline/default, candidate_0001, candidate_0002, candidate_0013, candidate_0014 | 6/15 pass; candidate_0014 best average static score; candidate_0013 faster | candidate_0014 p95 close to 100ms; motion-sensitive cases still missing |
| replay_small_gate_candidate_0015_static_smoke3 | codex/imu-gravity-check | 23f7893+local | near/far/depth-hole static replay smoke | check candidate_0015 motion-trigger implementation on static cases | 1/3 pass; do not promote candidate_0015 | p95 overhead still too close to hard gate; needs motion-sensitive replay before further tuning |
| replay_static_refresh_variants_001 | codex/imu-gravity-check | 2e70c72+local | near/far/depth-hole static replay smoke | compare conservative motion-refresh variants candidate_0016 and candidate_0017 | 6/6 pass; candidate_0017 leads this static smoke | refresh_count=0 on scored frames, so this validates cache reuse overhead only; motion-sensitive replay still missing |
