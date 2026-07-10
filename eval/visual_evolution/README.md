# D455 视觉能力进化记录库

本目录保存需求—任务—方法—晋级—回退的机器可读治理记录。记录只保存引用、状态和判断，不复制视频、完整 `analysis_runs/` 或构建产物。

## 文件

| 文件 | 内容 |
|---|---|
| `schema.json` | JSON Schema 契约 v1 |
| `requirements.jsonl` | 由真实差距生成的需求记录 |
| `tasks.jsonl` | 单目标任务记录 |
| `methods.jsonl` | 可被任务查询的方法版本 |
| `promotions.jsonl` | 按适用桶登记的晋级记录 |
| `rollbacks.jsonl` | 方法回退记录 |

运行证据通过 `run_id` 引用现有 `analysis_runs/<...>/run_score.json`。校验器默认只检查记录结构和引用关系；加 `--check-evidence` 才检查当前工作区是否有对应运行包和配置文件。

`replay_only` 表示只在固定回放条件下登记，不能当作在线生产方法；`probe_only` 表示探针，不能进入晋级记录。

## 校验

默认只校验记录结构和跨记录引用；需要检查当前工作区历史运行包时加 `--check-evidence`：

```powershell
python scripts\validate_visual_evolution_records.py --root eval\visual_evolution
python scripts\validate_visual_evolution_records.py --root eval\visual_evolution --check-evidence --analysis-runs analysis_runs
```

## 需求候选生成

从标准运行包生成候选，默认不写入 `requirements.jsonl`：

```powershell
python scripts\derive_visual_requirements.py `
  --runs analysis_runs\replay_hand_occlusion_reappear_real_001\hand_occlusion_reappear_candidate_0035 `
  --out $env:TEMP\d455_requirement_candidates.json `
  --min-occurrences 2
```

只有显式增加 `--register`，并且候选达到 `--min-occurrences`，才会追加到正式需求库。单次运行默认只能形成候选，不能直接确认稳定需求。

## 任务筹办与方法查询

按适用桶查询方法：

```powershell
python scripts\select_visual_methods.py `
  --applicability-bucket local_motion_roi `
  --include-probes `
  --out $env:TEMP\d455_methods.json
```

生成单目标任务时必须显式指定基线和输入 case。没有独立留出 case 时，输出任务会保持 `blocked_missing_holdout`：

```powershell
python scripts\prepare_visual_task.py `
  --requirement-id visual-need-hand-occlusion-local-refresh `
  --applicability-bucket hand_occlusion_reappear `
  --baseline-method-id method-candidate-0030 `
  --candidate-method-id method-candidate-0030 `
  --candidate-method-id method-candidate-0034 `
  --candidate-method-id method-candidate-0035 `
  --case-id hand_occlusion_reappear `
  --out $env:TEMP\d455_task.json
```

任务元数据由 `run_batch.py` 写入 converter 命令和 `command_plan.csv`，converter 再写入 `run_manifest.json` 与 `config_snapshot.json`；这些字段不会传给 D455.exe。旧命令不带元数据参数时保持兼容。

## 任务评估

`compare_runs.py` 会读取 `eval/score_weights.yaml` 的 `regression_gate`。当前评分器的 `spatial_quality` 和 `far_retention` 分别作为 `near_score`、`far_score` 的兼容来源，并在结果中记录来源。`evaluate_visual_task.py` 还会拒绝没有独立留出/影子证据或复用固定 `run_id` 的任务：

```powershell
python scripts\compare_runs.py `
  --baseline analysis_runs\...\run_score.json `
  --candidate analysis_runs\...\run_score.json `
  --enforce

python scripts\evaluate_visual_task.py `
  --task $env:TEMP\d455_task.json `
  --baseline analysis_runs\...\run_score.json `
  --candidate analysis_runs\...\run_score.json `
  --out $env:TEMP\d455_task_evaluation.json
```

任务结果为 `insufficient_evidence` 时不能晋级；固定回放、留出回放和影子运行必须使用不同的运行编号。

## 晋级、回退与适用选择

生产选择只返回 `status=promoted` 且存在 `promotion.status=approved` 的方法；固定回放选择可以返回 `replay_only`：

```powershell
python scripts\select_applicable_method.py --applicability-bucket slow_pan_far_object --evaluation-split production --out $env:TEMP\d455_production_method.json
python scripts\select_applicable_method.py --applicability-bucket slow_pan_far_object --evaluation-split replay --out $env:TEMP\d455_replay_method.json
```

`promote_visual_method.py` 默认要求固定回放、独立留出和影子结果，并要求通过 regression gate；`--replay-only` 只能登记固定回放桶，不能把 probe-only 方法变成方法能力。`rollback_visual_method.py` 只更新方法生命周期并追加回退证据，不删除原候选或运行包。
