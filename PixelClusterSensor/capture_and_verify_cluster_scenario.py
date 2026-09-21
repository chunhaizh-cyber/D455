"""Capture one auditable T3/T7 raw sequence, verify scene content, then run the cluster-control gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from evaluate_cluster_scenario_capture import SCENARIOS, evaluate_capture
from export_raw_sequence import export_sequence
from run_cluster_control_gate import run_gate


def capture_and_verify(*, executable: Path, output: Path, scenario: str, frames: int,
                       replay_source: Path | None = None, serial: str = "",
                       minimum_cluster_pixels: int = 1, retained_cluster_pixels: int | None = None,
                       confirmation_frames: int = 5) -> dict:
    if scenario not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {scenario}")
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if not 60 <= frames <= 600:
        raise ValueError("frames must be 60..600")
    output.mkdir(parents=True)
    report = {"格式": "PCS.ClusterScenarioGate/1", "状态": "运行中", "场景": scenario,
              "来源": "历史回放复验" if replay_source else "实时相机新采", "请求帧数": frames,
              "采集": None, "场景内容门禁": None, "控制门禁": None,
              "自动门禁通过": False, "人工视觉复核": "pending", "错误": None}
    try:
        capture = export_sequence(executable=executable, output=output / "capture", frames=frames,
                                  replay=replay_source, serial=serial)
        sequence = Path(capture["sequence"])
        scenario_result = evaluate_capture(sequence, scenario, output / "scenario_evaluation")
        control_result = run_gate(executable=executable, output=output / "control_gate", frames=frames,
                                  replay=sequence, minimum_cluster_pixels=minimum_cluster_pixels,
                                  retained_cluster_pixels=retained_cluster_pixels,
                                  confirmation_frames=confirmation_frames)
        report["采集"] = {"状态": capture["status"], "清单": capture["sequence"],
                          "帧数": len(capture["copied_frames"])}
        report["场景内容门禁"] = {"通过": scenario_result["通过"],
                                  "决策": str((output / "scenario_evaluation/scenario_decision.json").resolve())}
        report["控制门禁"] = {"通过": control_result["通过"],
                              "决策": str((output / "control_gate/run_manifest.json").resolve())}
        report["自动门禁通过"] = bool(scenario_result["通过"] and control_result["通过"])
        report["状态"] = "通过" if report["自动门禁通过"] else "失败"
    except Exception as error:
        report["状态"] = "失败"
        report["错误"] = str(error)
        raise
    finally:
        (output / "run_manifest.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                                   encoding="utf-8")
    return report


def main() -> None:
    project = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, default=project / "x64/Release/PixelClusterSensor.exe")
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), required=True)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--serial", default="")
    parser.add_argument("--replay-source", type=Path,
                        help="Test-only source. Omit for a new real-camera capture.")
    parser.add_argument("--minimum-cluster-pixels", type=int, default=1024)
    parser.add_argument("--retained-cluster-pixels", type=int, default=512)
    parser.add_argument("--confirmation-frames", type=int, default=5)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(capture_and_verify(executable=args.exe, output=args.output, scenario=args.scenario,
                                        frames=args.frames, replay_source=args.replay_source,
                                        serial=args.serial, minimum_cluster_pixels=args.minimum_cluster_pixels,
                                        retained_cluster_pixels=args.retained_cluster_pixels,
                                        confirmation_frames=args.confirmation_frames),
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
