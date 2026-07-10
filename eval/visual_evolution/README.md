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
