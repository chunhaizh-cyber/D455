"""Synthetic export-and-replay test for PCS.RawSequence/1 material capture."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from client import Client
from export_raw_sequence import export_sequence
from test_cluster_stream import make_static_sequence


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    project = Path(__file__).resolve().parent.parent
    parser.add_argument("--exe", type=Path, default=project / "x64/Release/PixelClusterSensor.exe")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    report = {"status": "running", "tests": []}

    def run(name, fn):
        started = time.perf_counter()
        detail = fn()
        report["tests"].append({"name": name, "status": "pass", "seconds": round(time.perf_counter() - started, 4), "detail": detail})
        print(f"PASS {name}", flush=True)

    try:
        source = make_static_sequence(root / "source", 3)
        result = export_sequence(executable=args.exe, output=root / "export", frames=3, replay=source)
        sequence_path = root / "export" / "sequence.json"
        sequence = json.loads(sequence_path.read_text(encoding="utf-8"))
        run("export_reports_success", lambda: check(result["status"] == "pass", "Export did not complete"))
        run("export_preserves_required_source_contract", lambda: check(sequence["格式"] == "PCS.RawSequence/1" and
                                                                          sequence["材料来源"] == "历史回放" and
                                                                          len(sequence["帧列表"]) == 3,
                                                                          "Raw sequence contract is incomplete"))
        run("export_preserves_source_order_and_time_domain", lambda: check(
            [frame["源帧号"] for frame in sequence["帧列表"]] == ["1", "2", "3"] and
            {frame["时间域"] for frame in sequence["帧列表"]} == {"synthetic_clock"}, "Source ordering/time changed"))
        with Client(args.exe, root / "roundtrip") as client:
            client.call("打开设备", {"来源": "目录回放", "清单": str(sequence_path)})
            observations = [client.call("获取单帧观察") for _ in range(3)]
            client.call("关闭设备")
        run("exported_sequence_replays_in_source_order", lambda: check([item["源帧号"] for item in observations] == ["1", "2", "3"],
                                                                          "Roundtrip source order differs"))
        run("export_writes_audit_manifest", lambda: check((root / "export" / "export_manifest.json").is_file(),
                                                           "Export manifest missing"))
        report["status"] = "pass"
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
