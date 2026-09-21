"""T9 fault-injection tests for cluster packet loss, ordering, restart, and resync."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import time

from cluster_protocol import validate_cluster_packet
from cluster_receiver import ClusterPacketReceiver
from cluster_tracker import ClusterTracker, reconstruct
from test_cluster_protocol import package


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def material(path: Path) -> bytes:
    packet = json.loads(path.read_text(encoding="utf-8"))
    return (path.parent / packet["材料"]["精确轮廓链"]["文件"]).read_bytes()


def resequence(packet: dict, sequence: int) -> dict:
    result = copy.deepcopy(packet)
    result["输出序号"] = result["场景版本"] = str(sequence)
    result["包标识"] = f"source-{sequence}"
    return result


def write_packet(root: Path, packet: dict, contours: bytes) -> Path:
    root.mkdir(parents=True)
    descriptor = packet["材料"]["精确轮廓链"]
    descriptor["字节数"] = len(contours)
    descriptor["SHA256"] = hashlib.sha256(contours).hexdigest()
    (root / descriptor["文件"]).write_bytes(contours)
    path = root / "packet.json"
    path.write_text(json.dumps(packet, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    validate_cluster_packet(path)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    report = {"status": "running", "tests": []}

    def run(name, fn):
        started = time.perf_counter()
        detail = fn()
        report["tests"].append({"name": name, "status": "pass",
                                "seconds": round(time.perf_counter() - started, 4), "detail": detail})
        print(f"PASS {name}", flush=True)

    try:
        source_path = package(root / "source")
        source = json.loads(source_path.read_text(encoding="utf-8"))
        contours = material(source_path)
        tracker = ClusterTracker(confirmation_frames=1)
        full = tracker.update(resequence(source, 1), contours)

        moved = resequence(source, 2)
        moved["簇变化"][0]["图像中心XY"][0] += 0.5
        delta = tracker.update(moved, contours)
        unchanged = tracker.update(resequence(moved, 3), contours)
        full_path = write_packet(root / "full", full, contours)
        delta_path = write_packet(root / "delta", delta, contours)
        heartbeat_path = write_packet(root / "heartbeat", unchanged, contours)

        receiver = ClusterPacketReceiver()
        run("initial_full_snapshot_is_accepted", lambda: check(
            receiver.receive_path(full_path)["status"] == "accepted", "Initial full snapshot was rejected"))
        package_id_receiver = ClusterPacketReceiver()
        package_id_receiver.receive_path(full_path)
        reused_id = copy.deepcopy(full)
        reused_id["输出序号"] = "2"
        reused_id["包标识"] = full["包标识"]
        reused_id_path = write_packet(root / "reused_package_id", reused_id, contours)
        run("package_id_reuse_with_different_content_is_rejected", lambda: check(
            package_id_receiver.receive_path(reused_id_path)["reason"] == "package_id_conflict",
            "Package ID was reused for different content"))
        scene_receiver = ClusterPacketReceiver()
        scene_receiver.receive_path(full_path)
        wrong_scene = copy.deepcopy(delta)
        wrong_scene["前置场景版本"] = "99"
        wrong_scene_path = write_packet(root / "wrong_previous_scene", wrong_scene, contours)
        run("wrong_previous_scene_version_stops_incremental_apply", lambda: check(
            scene_receiver.receive_path(wrong_scene_path)["reason"] == "previous_scene_version_mismatch" and
            scene_receiver.needs_resync, "Wrong previous scene version was applied"))
        gap_result = receiver.receive_path(heartbeat_path)
        run("increment_loss_requests_resynchronization", lambda: check(
            gap_result["status"] == "rejected" and gap_result["reason"] == "sequence_gap" and
            gap_result["request_full_snapshot"], "Packet loss did not stop incremental reconstruction"))

        tracker.request_full_snapshot()
        resync = tracker.update(resequence(moved, 4), contours)
        resync_path = write_packet(root / "resync", resync, tracker.last_output_contours or b"")
        resync_result = receiver.receive_path(resync_path)
        run("requested_full_snapshot_recovers_after_gap", lambda: check(
            resync_result["status"] == "accepted" and not receiver.needs_resync,
            "Full snapshot did not recover the receiver"))
        run("resynchronized_state_equals_direct_tracker_snapshot", lambda: check(
            receiver.snapshot() == tracker.active_snapshot(), "Resynchronized state differs from direct snapshot"))
        run("exact_duplicate_is_idempotent", lambda: check(
            receiver.receive_path(resync_path)["status"] == "duplicate" and
            receiver.snapshot() == tracker.active_snapshot(), "Exact duplicate changed receiver state"))

        conflict = copy.deepcopy(resync)
        conflict["包标识"] = "conflicting-resync"
        conflict_path = write_packet(root / "conflict", conflict, tracker.last_output_contours or b"")
        conflict_result = receiver.receive_path(conflict_path)
        run("conflicting_duplicate_is_rejected", lambda: check(
            conflict_result["status"] == "rejected" and conflict_result["reason"] == "duplicate_conflict" and
            receiver.needs_resync, "Conflicting duplicate was not treated as a resync fault"))

        tracker.request_full_snapshot()
        recovered = tracker.update(resequence(moved, 5), contours)
        recovered_path = write_packet(root / "recovered", recovered, tracker.last_output_contours or b"")
        receiver.receive_path(recovered_path)
        reorder_result = receiver.receive_path(delta_path)
        run("late_unseen_packet_is_rejected_as_reordered", lambda: check(
            reorder_result["status"] == "rejected" and reorder_result["reason"] == "reordered_packet",
            "Late packet was silently applied"))

        new_epoch = copy.deepcopy(recovered)
        new_epoch["跟踪时期"] = "2"
        new_epoch["输出序号"] = "1"
        new_epoch["包标识"] = "epoch-2-full"
        new_epoch_path = write_packet(root / "new_epoch", new_epoch, tracker.last_output_contours or b"")
        run("new_tracking_epoch_requires_and_accepts_full_snapshot", lambda: check(
            receiver.receive_path(new_epoch_path)["status"] == "accepted" and receiver.identity[1] == "2",
            "New tracking epoch full snapshot was not accepted"))
        run("retired_epoch_cannot_overwrite_new_epoch", lambda: check(
            receiver.receive_path(recovered_path)["reason"] == "retired_tracking_epoch",
            "Retired tracking epoch was accepted after epoch switch"))

        restarted = copy.deepcopy(new_epoch)
        restarted["会话标识"] = "restarted-session"
        restarted["跟踪时期"] = "1"
        restarted["包标识"] = "restart-full"
        restarted_path = write_packet(root / "restart", restarted, tracker.last_output_contours or b"")
        run("process_restart_full_snapshot_changes_identity", lambda: check(
            receiver.receive_path(restarted_path)["status"] == "accepted" and
            receiver.identity == ("restarted-session", "1"), "Restart did not establish a new identity"))

        occlusion_tracker = ClusterTracker(max_missing_frames=3, confirmation_frames=1,
                                           occlusion_confirmation_frames=1)
        occlusion_tracker.update(resequence(source, 1), contours)
        blank = resequence(source, 2)
        blank["簇变化"] = []
        occlusion_tracker.update(blank, contours)
        occlusion_tracker.request_full_snapshot()
        occluded_full = occlusion_tracker.update(resequence(blank, 3), contours)
        occluded_path = write_packet(root / "occluded_full", occluded_full,
                                    occlusion_tracker.last_output_contours or b"")
        occluded_state = reconstruct([occluded_full])
        run("resync_snapshot_preserves_unexpired_occluded_candidate", lambda: check(
            validate_cluster_packet(occluded_path)["status"] == "pass" and
            occluded_full["簇变化"][0]["帧内簇编号"] is None and
            occluded_full["簇变化"][0]["深度证据"]["当前实测像素数"] == 0 and
            occluded_state == occlusion_tracker.active_snapshot(),
            "Occluded candidate or its contour material was lost during resync"))
        source_gap_tracker = ClusterTracker(confirmation_frames=1)
        source_gap = resequence(source, 1)
        source_gap["输入质量"]["源缺口"] = True
        source_gap_output = source_gap_tracker.update(source_gap, contours)
        run("source_frame_gap_remains_explicit_in_output_quality", lambda: check(
            source_gap_output["输入质量"]["源缺口"] is True,
            "Source frame gap was silently rewritten as a complete source"))
        report["status"] = "pass"
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
