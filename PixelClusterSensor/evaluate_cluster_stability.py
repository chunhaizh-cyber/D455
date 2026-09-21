"""Evaluate deterministic static stability of completed PCS cluster stream runs.

This evaluator is deliberately limited to recorded stream evidence.  It does
not infer physical stillness, object identity, absolute depth accuracy, or
dynamic tracking quality from matching packets.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path

from cluster_protocol import validate_cluster_packet
from cluster_tracker import reconstruct


NONDETERMINISTIC_HEADER_FIELDS = {"包标识", "会话标识", "发布Unix毫秒"}


def normalized_packet(packet: dict) -> bytes:
    value = copy.deepcopy({key: item for key, item in packet.items() if key not in NONDETERMINISTIC_HEADER_FIELDS})
    # This checksum names a published PCS.Observation/1 manifest whose session
    # and publication timestamp legitimately vary across otherwise identical runs.
    # It remains in the packet for traceability, but cannot participate in an
    # algorithm determinism comparison.
    value.get("指标", {}).pop("转换来源清单SHA256", None)
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), sort_keys=True).encode("utf-8")


def load_run(root: Path) -> tuple[list[dict], list[Path]]:
    packet_root = root / "cluster_packets"
    paths = sorted(packet_root.glob("packet_*/packet.json"))
    if not paths:
        raise ValueError(f"No cluster packets: {root}")
    packets = []
    for path in paths:
        validate_cluster_packet(path)
        packets.append(json.loads(path.read_text(encoding="utf-8")))
    return packets, paths


def evaluate_runs(run_roots: list[Path], output: Path) -> dict:
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if len(run_roots) < 1:
        raise ValueError("At least one completed stream run is required")
    output.mkdir(parents=True)
    loaded = [load_run(root.resolve()) for root in run_roots]
    packet_lists = [item[0] for item in loaded]
    lengths = [len(packets) for packets in packet_lists]
    consistent_length = len(set(lengths)) == 1
    hashes = [[hashlib.sha256(normalized_packet(packet)).hexdigest() for packet in packets] for packets in packet_lists]
    deterministic = consistent_length and all(value == hashes[0] for value in hashes[1:])
    events: list[dict] = []
    metrics: list[dict] = []
    all_valid = True
    reconstruction_ok = True
    frame_local_id_reassignments = 0
    static_lifecycle_events = 0
    for run_index, packets in enumerate(packet_lists, start=1):
        previous_sequence = 0
        track_by_frame_cluster: dict[int, str] = {}
        try:
            active = reconstruct(packets)
        except Exception as error:
            all_valid = False
            reconstruction_ok = False
            events.append({"run_index": run_index, "event": "reconstruction_failure", "detail": str(error)})
            active = {}
        for packet_index, packet in enumerate(packets, start=1):
            sequence = int(packet["输出序号"])
            if sequence != previous_sequence + 1:
                all_valid = False
                events.append({"run_index": run_index, "event": "sequence_gap", "detail": f"{previous_sequence}->{sequence}"})
            previous_sequence = sequence
            for change in packet["簇变化"]:
                change_type = change["变化类型"]
                frame_cluster = change["帧内簇编号"]
                track = change["相机跟踪候选编号"]
                if frame_cluster is not None and track is not None:
                    old = track_by_frame_cluster.get(frame_cluster)
                    if old is not None and old != track:
                        # 本帧簇编号 only identifies one source frame.  Its
                        # reuse across frames cannot prove a physical track-ID
                        # switch, but it is useful to expose as a source-label
                        # churn diagnostic.
                        frame_local_id_reassignments += 1
                        events.append({"run_index": run_index, "event": "frame_local_id_reassignment",
                                       "detail": f"frame_cluster={frame_cluster} {old}->{track}"})
                    track_by_frame_cluster[frame_cluster] = track
                if change_type in {"Lost", "Reappeared", "Occluded"}:
                    static_lifecycle_events += 1
                    events.append({"run_index": run_index, "event": "lifecycle_event", "detail": change_type})
            metrics.append({"run_index": run_index, "packet_index": packet_index, "output_sequence": sequence,
                            "packet_type": packet["包类型"], "active_tracks_after_packet": len(active)})
    decision = {
        "format": "PCS.ClusterStabilityDecision/1", "scope": "static replay evidence only",
        "run_count": len(run_roots), "frames_per_run": lengths, "packet_validation_pass": all_valid,
        "normalized_determinism_pass": deterministic, "reconstruction_pass": reconstruction_ok,
        "frame_local_id_reassignment_count": frame_local_id_reassignments,
        "static_lifecycle_event_count": static_lifecycle_events,
        "pass": all_valid and deterministic and reconstruction_ok and static_lifecycle_events == 0,
        "not_proven": ["physical_scene_static", "dynamic_tracking", "world_identity", "absolute_depth_accuracy"],
    }
    (output / "run_decision.json").write_text(json.dumps(decision, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    with (output / "packet_metrics.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=["run_index", "packet_index", "output_sequence", "packet_type", "active_tracks_after_packet"])
        writer.writeheader()
        writer.writerows(metrics)
    with (output / "events.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=["run_index", "event", "detail"])
        writer.writeheader()
        writer.writerows(events)
    return decision


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", required=True, type=Path, nargs="+", help="Completed cluster_stream output directories")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(evaluate_runs(args.runs, args.output.resolve()), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
