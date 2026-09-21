"""Strict incremental receiver and resynchronization state for cluster packets."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

from cluster_protocol import validate_cluster_packet


def packet_digest(packet: dict) -> str:
    raw = json.dumps(packet, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


class ClusterPacketReceiver:
    """Apply one session/epoch stream without silently crossing a packet gap."""

    def __init__(self, maximum_remembered_packets: int = 4096,
                 maximum_retired_identities: int = 1024) -> None:
        if not 1 <= maximum_remembered_packets <= 65536:
            raise ValueError("maximum_remembered_packets must be 1..65536")
        if not 1 <= maximum_retired_identities <= 4096:
            raise ValueError("maximum_retired_identities must be 1..4096")
        self.maximum_remembered_packets = maximum_remembered_packets
        self.maximum_retired_identities = maximum_retired_identities
        self.identity: tuple[str, str] | None = None
        self.retired_identities: set[tuple[str, str]] = set()
        self.last_sequence: int | None = None
        self.scene_version: int | None = None
        self.base_sequence: str | None = None
        self.needs_resync = True
        self.state: dict[str, dict] = {}
        self.seen: dict[int, str] = {}
        self.seen_package_ids: dict[str, tuple[int, str]] = {}

    def receive_path(self, path: Path) -> dict:
        validate_cluster_packet(path)
        return self.receive(json.loads(path.read_text(encoding="utf-8")))

    def _reject(self, reason: str) -> dict:
        if reason != "retired_tracking_epoch":
            self.needs_resync = True
        return {"status": "rejected", "reason": reason, "request_full_snapshot": True,
                "active_tracks": len(self.state)}

    def receive(self, packet: dict) -> dict:
        identity = (packet["会话标识"], packet["跟踪时期"])
        sequence = int(packet["输出序号"])
        digest = packet_digest(packet)
        package_id = packet["包标识"]

        if identity in self.retired_identities:
            return self._reject("retired_tracking_epoch")
        if identity == self.identity and sequence in self.seen:
            if self.seen[sequence] == digest:
                return {"status": "duplicate", "reason": None, "request_full_snapshot": self.needs_resync,
                        "active_tracks": len(self.state)}
            return self._reject("duplicate_conflict")
        if identity == self.identity and package_id in self.seen_package_ids:
            return self._reject("package_id_conflict")

        identity_changed = self.identity is not None and identity != self.identity
        if identity_changed and packet["包类型"] != "FullSnapshot":
            return self._reject("tracking_epoch_change_requires_full_snapshot")
        if self.needs_resync and packet["包类型"] != "FullSnapshot":
            return self._reject("full_snapshot_required")
        if not identity_changed and self.last_sequence is not None and packet["包类型"] != "FullSnapshot":
            if sequence < self.last_sequence + 1:
                return self._reject("reordered_packet")
            if sequence > self.last_sequence + 1:
                return self._reject("sequence_gap")

        if packet["包类型"] == "FullSnapshot":
            if identity_changed:
                if len(self.retired_identities) >= self.maximum_retired_identities:
                    return self._reject("tracking_identity_budget_exhausted")
                self.retired_identities.add(self.identity)
            self.identity = identity
            self.seen = {}
            self.seen_package_ids = {}
            self.state = {}
            for entry in packet["簇变化"]:
                identifier = entry["相机跟踪候选编号"]
                if identifier is not None and entry["变化类型"] not in {"Lost", "Removed"}:
                    self.state[identifier] = copy.deepcopy(entry)
            self.base_sequence = packet["输出序号"]
            self.scene_version = int(packet["场景版本"])
            self.needs_resync = False
        else:
            if packet["依赖全量序号"] != self.base_sequence:
                return self._reject("base_snapshot_mismatch")
            if int(packet["前置场景版本"]) != self.scene_version:
                return self._reject("previous_scene_version_mismatch")
            next_scene = int(packet["场景版本"])
            expected_scene = self.scene_version + (1 if packet["包类型"] == "Delta" else 0)
            if next_scene != expected_scene:
                return self._reject("scene_version_mismatch")
            if packet["包类型"] == "Delta":
                next_state = copy.deepcopy(self.state)
                for entry in packet["簇变化"]:
                    identifier = entry["相机跟踪候选编号"]
                    if entry["变化类型"] in {"Lost", "Removed"}:
                        next_state.pop(identifier, None)
                    elif entry["帧内簇编号"] is None:
                        if identifier not in next_state:
                            return self._reject("unknown_track_tombstone")
                        retained = next_state[identifier]
                        retained["跟踪状态"] = "Occluded"
                        retained["时效"] = copy.deepcopy(entry["时效"])
                    else:
                        next_state[identifier] = copy.deepcopy(entry)
                self.state = next_state
            self.scene_version = next_scene

        self.last_sequence = sequence
        self.seen[sequence] = digest
        self.seen_package_ids[package_id] = (sequence, digest)
        while len(self.seen) > self.maximum_remembered_packets:
            oldest = next(iter(self.seen))
            del self.seen[oldest]
            self.seen_package_ids = {key: value for key, value in self.seen_package_ids.items()
                                     if value[0] != oldest}
        return {"status": "accepted", "reason": None, "request_full_snapshot": False,
                "active_tracks": len(self.state)}

    def snapshot(self) -> dict[str, dict]:
        return copy.deepcopy(self.state)
