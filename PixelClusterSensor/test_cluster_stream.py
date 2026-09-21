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
        run("static_following_packets_are_heartbeats", lambda: check(all(packet["包类型"] == "Heartbeat" and
                                                                           packet["依赖全量序号"] == "1" and not packet["簇变化"]
                                                                           for packet in packets[1:]),
                                                                     "Static follow-up packets are not empty heartbeats"))
        run("static_candidate_keeps_track_id", lambda: check(packets[0]["簇变化"][0]["相机跟踪候选编号"] == "1",
                                                               "Static candidate did not receive a camera-local ID"))
        run("full_and_deltas_reconstruct_one_active_track", lambda: check(list(reconstruct(packets)) == ["1"],
                                                                            "Reconstruction lost the active track"))
        run("stream_audit_files_are_complete", lambda: check(all((root / "stream" / name).is_file()
                                                                    for name in ("run_manifest.json", "packet_metrics.csv", "events.csv")),
                                                               "Required stream audit file is missing"))
        with (root / "stream" / "packet_metrics.csv").open(encoding="utf-8", newline="") as file:
            rows = list(csv.DictReader(file))
        run("metrics_cover_every_source_frame", lambda: check([row["source_frame"] for row in rows] == ["1", "2", "3"],
                                                                "Metrics do not preserve source frame order"))
        run("metrics_include_source_partition_diagnostics", lambda: check(
            all(name in rows[0] for name in ("source_cluster_count", "source_color_region_count",
                                             "source_alignment_ms", "source_partition_ms", "source_contour_ms",
                                             "source_color_partition_ms", "source_depth_seed_ms",
                                             "source_missing_depth_component_ms", "source_enclosed_missing_inheritance_ms",
                                             "source_depth_seed_count", "source_cross_color_merge_count",
                                             "source_retained_nonlocal_small_depth_seed_count",
                                             "source_forced_incompatible_depth_noise_merge_count",
                                             "source_missing_compatible_depth_merge_count")),
            "Source partition diagnostics are missing from packet metrics"))
        offset_stream = run_stream(executable=args.exe, output=root / "offset_stream", frames=2, replay=replay, start_frame=2)
        offset_packets = [json.loads(Path(path).read_text(encoding="utf-8")) for path in offset_stream["packets"]]
        with (root / "offset_stream" / "packet_metrics.csv").open(encoding="utf-8", newline="") as file:
            offset_rows = list(csv.DictReader(file))
        run("start_frame_skips_released_prefix", lambda: check(
            offset_stream["start_frame"] == 2 and [row["source_frame"] for row in offset_rows] == ["2", "3"] and
            [packet["输出序号"] for packet in offset_packets] == ["1", "2"],
            "Stream start offset did not select the requested source window"))
        long_replay = make_static_sequence(root / "long_replay", 129)
        long_stream = run_stream(executable=args.exe, output=root / "long_stream", frames=129, replay=long_replay)
        run("released_source_material_allows_more_than_default_packet_limit", lambda: check(
            long_stream["status"] == "pass" and long_stream["source_observation_count"] == 129,
            "Source material release did not permit a bounded long stream"))
        report["status"] = "pass"
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
