# Codex Handoff

Generated handoff files describe one experiment round for Codex.

They must include:

- goal and priority order
- current best candidate
- main failures
- evidence paths
- allowed changes
- forbidden changes
- required output shape

For config-only rounds, Codex may edit only `configs/candidates/*.json` and `eval/search_space.yaml`.
