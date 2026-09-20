"""P3 integration test for the bounded cluster stream evaluation bridge."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import time

from cluster_protocol import validate_cluster_packet
from cluster_stream import run_stream
from cluster_tracker import reconstruct
from test_cluster_conversion import make_replay


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def make_static_sequence(root: Path, frame_count: int = 3) -> Path:
    sequence_path = make_replay(root)
    sequence = json.loads(sequence_path.read_text(encoding="utf-8"))
    source = sequence["帧列表"][0]
    sequence["帧列表"] = [
        {**source, "源帧号": str(index), "彩图时间戳毫秒": float(index * 33),
         "深度时间戳毫秒": float(index * 33)}
        for index in range(1, frame_count + 1)
    ]
    sequence_path.write_text(json.dumps(sequence, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return sequence_path


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
        replay = make_static_sequence(root / "replay")
        stream = run_stream(executable=args.exe, output=root / "stream", frames=3, replay=replay)
        packet_paths = [Path(path) for path in stream["packets"]]
        packets = [json.loads(path.read_text(encoding="utf-8")) for path in packet_paths]
        run("stream_reports_success", lambda: check(stream["status"] == "pass" and stream["source_observation_count"] == 3,
                                                       "Three source observations were not completed"))
        run("all_published_packets_validate", lambda: [validate_cluster_packet(path) for path in packet_paths])
        run("first_packet_is_full_snapshot", lambda: check(packets[0]["包类型"] == "FullSnapshot" and
                                                             packets[0]["依赖全量序号"] is None, "First packet is not full"))
        run("static_following_packets_are_deltas", lambda: check(all(packet["包类型"] == "Delta" and
                                                                      packet["依赖全量序号"] == "1" for packet in packets[1:]),
                                                                "Static follow-up packets are not deltas"))
        run("static_candidate_keeps_track_id", lambda: check({packet["簇变化"][0]["相机跟踪候选编号"] for packet in packets} == {"1"},
                                                               "Static candidate changed camera-local track ID"))
        run("full_and_deltas_reconstruct_one_active_track", lambda: check(list(reconstruct(packets)) == ["1"],
                                                                            "Reconstruction lost the active track"))
        run("stream_audit_files_are_complete", lambda: check(all((root / "stream" / name).is_file()
                                                                    for name in ("run_manifest.json", "packet_metrics.csv", "events.csv")),
                                                               "Required stream audit file is missing"))
        with (root / "stream" / "packet_metrics.csv").open(encoding="utf-8", newline="") as file:
            rows = list(csv.DictReader(file))
        run("metrics_cover_every_source_frame", lambda: check([row["source_frame"] for row in rows] == ["1", "2", "3"],
                                                                "Metrics do not preserve source frame order"))
        report["status"] = "pass"
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
