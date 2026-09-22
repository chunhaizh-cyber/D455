"""Deterministic, bounded, offline P2 short-term tracker for PCS.ClusterObservation/1 snapshots.

The tracker emits camera-local candidate IDs only.  It never creates a world
existence identity and does not use history as current depth evidence.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
from dataclasses import dataclass

from cluster_protocol import validate_cluster_packet


UNMATCHED_COST = 1_000_000
INVALID_COST = 10_000_000
OBSERVATION_FIELDS = (
    "范围XYWH", "图像中心XY", "像素数", "触及视野边界", "轮廓", "形状指纹", "颜色摘要",
    "距离", "三维中心米", "尺寸米", "深度证据", "运动", "遮挡",
)


def bbox_iou(first: list[int], second: list[int]) -> float:
    left, top = max(first[0], second[0]), max(first[1], second[1])
    right, bottom = min(first[0] + first[2], second[0] + second[2]), min(first[1] + first[3], second[1] + second[3])
    intersection = max(0, right - left) * max(0, bottom - top)
    union = first[2] * first[3] + second[2] * second[3] - intersection
    return intersection / union if union else 0.0


def bbox_smaller_coverage(first: list[int], second: list[int]) -> float:
    left, top = max(first[0], second[0]), max(first[1], second[1])
    right = min(first[0] + first[2], second[0] + second[2])
    bottom = min(first[1] + first[3], second[1] + second[3])
    intersection = max(0, right - left) * max(0, bottom - top)
    smaller = min(first[2] * first[3], second[2] * second[3])
    return intersection / smaller if smaller else 0.0


def match_cost(previous: dict, current: dict, diagonal: float) -> int | None:
    a, b = previous["范围XYWH"], current["范围XYWH"]
    old_center, new_center = previous["图像中心XY"], current["图像中心XY"]
    center = math.dist(old_center, new_center) / diagonal
    area_ratio = current["像素数"] / previous["像素数"]
    if center > 0.12 or not 0.25 <= area_ratio <= 4.0:
        return None
    iou = bbox_iou(a, b)
    color_a, color_b = previous["颜色摘要"]["RGB均值"], current["颜色摘要"]["RGB均值"]
    color = sum(abs(x - y) for x, y in zip(color_a, color_b)) / (3 * 255)
    if color > 0.55:
        return None
    shape_a = previous["形状指纹"]["层级"][-1]["位图SHA256"]
    shape_b = current["形状指纹"]["层级"][-1]["位图SHA256"]
    shape = 0.0 if shape_a == shape_b else 0.5
    return int(round((0.45 * center + 0.40 * (1.0 - iou) + 0.10 * color + 0.05 * shape) * 1_000_000))


def tentative_growth_match(previous: dict, current: dict, diagonal: float, minimum_pixels: int) -> bool:
    """Accept bounded entry/exit growth without relaxing the general tentative gate."""
    if min(previous["像素数"], current["像素数"]) < minimum_pixels:
        return False
    if math.dist(previous["图像中心XY"], current["图像中心XY"]) / diagonal > 0.12:
        return False
    area_ratio = current["像素数"] / previous["像素数"]
    if not 0.25 <= area_ratio <= 4.0 or 0.85 < area_ratio < 1.25:
        return False
    if bbox_smaller_coverage(previous["范围XYWH"], current["范围XYWH"]) < 0.75:
        return False
    color_a = previous["颜色摘要"]["RGB均值"]
    color_b = current["颜色摘要"]["RGB均值"]
    color_delta = sum(abs(x - y) for x, y in zip(color_a, color_b)) / (3 * 255)
    return color_delta <= 0.15


def same_observation(previous: dict, current: dict) -> bool:
    """Compare current observable content, excluding tracker-owned bookkeeping."""
    return all(previous.get(field) == current.get(field) for field in OBSERVATION_FIELDS)


def hungarian(cost: list[list[int]]) -> list[int]:
    """Minimum-cost square assignment; result[row] is the selected column."""
    size = len(cost)
    u, v, p, way = [0] * (size + 1), [0] * (size + 1), [0] * (size + 1), [0] * (size + 1)
    for row in range(1, size + 1):
        p[0], column, minv, used = row, 0, [INVALID_COST] * (size + 1), [False] * (size + 1)
        while True:
            used[column] = True
            current_row, delta, next_column = p[column], INVALID_COST, 0
            for candidate in range(1, size + 1):
                if not used[candidate]:
                    value = cost[current_row - 1][candidate - 1] - u[current_row] - v[candidate]
                    if value < minv[candidate]:
                        minv[candidate], way[candidate] = value, column
                    if minv[candidate] < delta:
                        delta, next_column = minv[candidate], candidate
            for candidate in range(size + 1):
                if used[candidate]:
                    u[p[candidate]] += delta
                    v[candidate] -= delta
                else:
                    minv[candidate] -= delta
            column = next_column
            if p[column] == 0:
                break
        while True:
            previous = way[column]
            p[column], column = p[previous], previous
            if column == 0:
                break
    result = [-1] * size
    for column in range(1, size + 1):
        result[p[column] - 1] = column - 1
    return result


def component_matches(edges: dict[tuple[int, int], int], track_count: int, cluster_count: int, max_nodes: int = 64) -> set[tuple[int, int]]:
    """Solve each gated bipartite component globally; oversized components are left unmatched."""
    track_adj = {index: set() for index in range(track_count)}
    cluster_adj = {index: set() for index in range(cluster_count)}
    for (track, cluster), _ in edges.items():
        track_adj[track].add(cluster)
        cluster_adj[cluster].add(track)
    seen_tracks, seen_clusters, matched = set(), set(), set()
    for start in range(track_count):
        if start in seen_tracks or not track_adj[start]:
            continue
        stack, tracks, clusters = [("t", start)], set(), set()
        while stack:
            kind, index = stack.pop()
            if kind == "t":
                if index in tracks:
                    continue
                tracks.add(index)
                stack.extend(("c", value) for value in track_adj[index])
            else:
                if index in clusters:
                    continue
                clusters.add(index)
                stack.extend(("t", value) for value in cluster_adj[index])
        seen_tracks.update(tracks)
        seen_clusters.update(clusters)
        if len(tracks) + len(clusters) > max_nodes:
            continue
        track_list, cluster_list = sorted(tracks), sorted(clusters)
        rows, columns = len(track_list), len(cluster_list)
        size = rows + columns
        matrix = [[0] * size for _ in range(size)]
        for row in range(rows):
            for column in range(columns):
                matrix[row][column] = edges.get((track_list[row], cluster_list[column]), INVALID_COST)
            for column in range(columns, size):
                matrix[row][column] = UNMATCHED_COST
        for row in range(rows, size):
            for column in range(columns):
                matrix[row][column] = UNMATCHED_COST
            for column in range(columns, size):
                matrix[row][column] = 0
        assignment = hungarian(matrix)
        for row in range(rows):
            column = assignment[row]
            if column < columns and matrix[row][column] < UNMATCHED_COST:
                matched.add((track_list[row], cluster_list[column]))
    return matched


@dataclass
class Track:
    identifier: str
    record: dict
    association_record: dict
    contour_material: bytes | None
    visible_frames: int
    confirmed: bool
    confirmation_hits: int
    missing_frames: int = 0
    occlusion_published: bool = False


class ClusterTracker:
    def __init__(self, max_missing_frames: int = 2, confirmation_frames: int = 5,
                 minimum_new_cluster_pixels: int = 1, minimum_retained_cluster_pixels: int = 1,
                 maximum_tentative_match_cost: int = 200_000,
                 occlusion_confirmation_frames: int = 1,
                 allow_tentative_growth_association: bool = False):
        if not 1 <= max_missing_frames <= 120:
            raise ValueError("max_missing_frames must be 1..120")
        if not 1 <= confirmation_frames <= 30:
            raise ValueError("confirmation_frames must be 1..30")
        if minimum_new_cluster_pixels < 1 or not 1 <= minimum_retained_cluster_pixels <= minimum_new_cluster_pixels:
            raise ValueError("cluster pixel thresholds must satisfy 1 <= retained <= new")
        if not 0 <= maximum_tentative_match_cost < UNMATCHED_COST:
            raise ValueError("maximum_tentative_match_cost must be 0..999999")
        if not 1 <= occlusion_confirmation_frames < max_missing_frames:
            raise ValueError("occlusion_confirmation_frames must be at least 1 and below max_missing_frames")
        self.max_missing_frames = max_missing_frames
        self.confirmation_frames = confirmation_frames
        self.minimum_new_cluster_pixels = minimum_new_cluster_pixels
        self.minimum_retained_cluster_pixels = minimum_retained_cluster_pixels
        self.maximum_tentative_match_cost = maximum_tentative_match_cost
        self.occlusion_confirmation_frames = occlusion_confirmation_frames
        self.allow_tentative_growth_association = allow_tentative_growth_association
        self.tracks: dict[str, Track] = {}
        self.next_identifier = 1
        self.base_sequence: str | None = None
        self.scene_version = 0
        self.full_snapshot_requested = False
        self.last_output_contours: bytes | None = None

    @staticmethod
    def _store_entry(entry: dict, contours: bytes | None) -> tuple[dict, bytes | None]:
        stored = copy.deepcopy(entry)
        if contours is None:
            return stored, None
        packed = bytearray()
        for ring in stored["轮廓"]:
            length = (ring["有效位数"] + 7) // 8
            start = ring["字节偏移"]
            ring["字节偏移"] = len(packed)
            packed.extend(contours[start:start + length])
        return stored, bytes(packed)

    @staticmethod
    def _append_stored_entry(entry: dict, material: bytes | None,
                             output: bytearray) -> dict:
        result = copy.deepcopy(entry)
        if result["轮廓"] and material is None:
            raise ValueError("Tracked contour material is unavailable for a full snapshot")
        base = len(output)
        output.extend(material or b"")
        for ring in result["轮廓"]:
            ring["字节偏移"] += base
        return result

    def request_full_snapshot(self) -> None:
        """Force the next source update to publish a self-contained snapshot."""
        self.full_snapshot_requested = True

    def _snapshot_entries(self) -> tuple[list[dict], bytes]:
        entries = []
        contours = bytearray()
        for track in sorted(self.tracks.values(), key=lambda item: int(item.identifier)):
            entry = self._append_stored_entry(track.record, track.contour_material, contours)
            entry["关联证据"] = None
            if track.missing_frames:
                entry["帧内簇编号"] = None
                entry["变化类型"] = "Occluded"
                entry["跟踪状态"] = "Occluded"
                entry["颜色摘要"]["可比性"] = "未知"
                entry["距离"] = {"模式": "UnknownDistance", "值米": None, "区间米": None,
                                   "不确定度米": None, "依据": "证据不足"}
                entry["三维中心米"] = None
                entry["尺寸米"] = None
                entry["深度证据"] = {"当前实测像素数": 0, "历史候选像素数": entry["像素数"],
                                     "估算像素数": 0, "缺失像素数": 0, "范围外像素数": 0}
                entry["运动"] = None
                entry["遮挡"] = {"状态": "Occluded", "比例": None,
                                  "依据": "full_snapshot_preserves_unexpired_occluded_candidate"}
                entry["时效"] = {"连续可见帧数": 0, "连续缺失帧数": track.missing_frames,
                                 "证据年龄毫秒": None}
            entries.append(entry)
        return entries, bytes(contours)

    def update(self, packet: dict, contour_material: bytes | None = None) -> dict:
        if packet["包类型"] != "FullSnapshot":
            raise ValueError("Tracker input must be a full snapshot")
        current = packet["簇变化"]
        width, height = packet["图像尺寸WH"]
        diagonal = math.hypot(width, height)
        prior = sorted(self.tracks.values(), key=lambda item: int(item.identifier))
        edges = {}
        growth_edges = set()
        for track_index, track in enumerate(prior):
            for cluster_index, entry in enumerate(current):
                association_pixels = (self.minimum_retained_cluster_pixels if track.confirmed
                                      else self.minimum_new_cluster_pixels)
                if entry["像素数"] < association_pixels:
                    continue
                cost = match_cost(track.association_record, entry, diagonal)
                growth_match = bool(cost is not None and not track.confirmed and
                                    self.allow_tentative_growth_association and
                                    tentative_growth_match(track.association_record, entry, diagonal,
                                                           self.minimum_new_cluster_pixels * 3))
                if cost is not None and (track.confirmed or cost <= self.maximum_tentative_match_cost or growth_match):
                    edges[(track_index, cluster_index)] = cost
                    if growth_match and cost > self.maximum_tentative_match_cost:
                        growth_edges.add((track_index, cluster_index))
        matched = component_matches(edges, len(prior), len(current))
        matched_by_track = {track: cluster for track, cluster in matched}
        matched_by_cluster = {cluster: track for track, cluster in matched}
        changes = []
        filtered_new_clusters = 0
        filtered_new_pixels = 0
        tentative_growth_matches = 0
        meaningful_change = False
        for cluster_index, source in enumerate(current):
            entry = copy.deepcopy(source)
            if cluster_index in matched_by_cluster:
                track_index = matched_by_cluster[cluster_index]
                track = prior[track_index]
                edge = (track_index, cluster_index)
                growth_match = edge in growth_edges
                shift = math.dist(track.record["图像中心XY"], entry["图像中心XY"])
                unchanged = same_observation(track.record, entry)
                was_missing = track.missing_frames > 0
                visible_frames = 1 if was_missing else track.visible_frames + 1
                confirmation_hits = (0 if growth_match else
                                     (1 if was_missing else track.confirmation_hits + 1))
                became_confirmed = not track.confirmed and confirmation_hits >= self.confirmation_frames
                track.confirmed = track.confirmed or became_confirmed
                entry["相机跟踪候选编号"] = track.identifier
                reappeared = track.occlusion_published and track.confirmed
                entry["变化类型"] = "Reappeared" if reappeared else ("Moved" if shift >= 1.0 else "Updated")
                entry["跟踪状态"] = "Reappeared" if reappeared else ("Active" if track.confirmed else "Tentative")
                entry["时效"] = {"连续可见帧数": visible_frames, "连续缺失帧数": 0, "证据年龄毫秒": None}
                association_cost = edges[edge]
                if growth_match:
                    tentative_growth_matches += 1
                entry["关联证据"] = {"算法": ("bbox-contained-growth-assignment/1" if growth_match else
                                               "bbox-color-shape-assignment/1"), "代价": association_cost}
                stored_entry, stored_contours = self._store_entry(entry, contour_material)
                if association_cost <= self.maximum_tentative_match_cost or growth_match:
                    track.association_record = copy.deepcopy(stored_entry)
                track.record, track.contour_material = stored_entry, stored_contours
                track.visible_frames, track.confirmation_hits, track.missing_frames = visible_frames, confirmation_hits, 0
                track.occlusion_published = False
                if not unchanged or was_missing or became_confirmed:
                    meaningful_change = True
            else:
                if entry["像素数"] < self.minimum_new_cluster_pixels:
                    filtered_new_clusters += 1
                    filtered_new_pixels += entry["像素数"]
                    continue
                identifier = str(self.next_identifier)
                self.next_identifier += 1
                entry["相机跟踪候选编号"] = identifier
                entry["变化类型"] = "Added"
                confirmed = self.confirmation_frames == 1
                entry["跟踪状态"] = "Active" if confirmed else "Tentative"
                entry["时效"] = {"连续可见帧数": 1, "连续缺失帧数": 0, "证据年龄毫秒": None}
                entry["关联证据"] = None
                stored_entry, stored_contours = self._store_entry(entry, contour_material)
                self.tracks[identifier] = Track(identifier, stored_entry, copy.deepcopy(stored_entry),
                                                stored_contours, 1, confirmed, 1)
                meaningful_change = True
            changes.append(entry)
        for track_index, track in enumerate(prior):
            if track_index in matched_by_track:
                continue
            if not track.confirmed:
                changes.append({
                    "帧内簇编号": None, "相机跟踪候选编号": track.identifier, "自我绑定令牌": None,
                    "变化类型": "Removed", "跟踪状态": "Retired",
                    "范围XYWH": None, "图像中心XY": None, "像素数": None, "触及视野边界": None, "轮廓": None,
                    "形状指纹": None, "颜色摘要": None, "距离": None, "三维中心米": None, "尺寸米": None, "深度证据": None,
                    "运动": None, "遮挡": {"状态": "Unknown", "比例": None, "依据": "tentative_candidate_not_reobserved"},
                    "关联证据": None, "时效": {"连续可见帧数": 0, "连续缺失帧数": 1, "证据年龄毫秒": None},
                    "详细材料句柄": None,
                })
                del self.tracks[track.identifier]
                meaningful_change = True
                continue
            track.missing_frames += 1
            track.visible_frames = 0
            terminal = track.missing_frames >= self.max_missing_frames
            if not terminal and track.missing_frames < self.occlusion_confirmation_frames:
                continue
            if not terminal and track.occlusion_published:
                continue
            changes.append({
                "帧内簇编号": None, "相机跟踪候选编号": track.identifier, "自我绑定令牌": None,
                "变化类型": "Lost" if terminal else "Occluded", "跟踪状态": "Retired" if terminal else "Occluded",
                "范围XYWH": None, "图像中心XY": None, "像素数": None, "触及视野边界": None, "轮廓": None,
                "形状指纹": None, "颜色摘要": None, "距离": None, "三维中心米": None, "尺寸米": None, "深度证据": None,
                "运动": None, "遮挡": {"状态": "Occluded", "比例": None, "依据": "current_snapshot_has_no_matching_cluster"},
                "关联证据": None, "时效": {"连续可见帧数": 0, "连续缺失帧数": track.missing_frames, "证据年龄毫秒": None},
                "详细材料句柄": None,
            })
            if terminal:
                del self.tracks[track.identifier]
            else:
                track.occlusion_published = True
            meaningful_change = True
        output = copy.deepcopy(packet)
        coverage = output["全局覆盖摘要"]
        coverage["已处理像素数"] -= filtered_new_pixels
        coverage["未知像素数"] += filtered_new_pixels
        output.setdefault("指标", {})["跟踪新候选最小像素数"] = self.minimum_new_cluster_pixels
        output["指标"]["跟踪保留候选最小像素数"] = self.minimum_retained_cluster_pixels
        output["指标"]["未确认候选最大关联代价"] = self.maximum_tentative_match_cost
        output["指标"]["允许未确认候选增长关联"] = self.allow_tentative_growth_association
        output["指标"]["本帧未确认候选增长关联数"] = tentative_growth_matches
        output["指标"]["遮挡确认缺失帧数"] = self.occlusion_confirmation_frames
        output["指标"]["丢失判定缺失帧数"] = self.max_missing_frames
        output["指标"]["本帧过滤新候选数"] = filtered_new_clusters
        output["指标"]["本帧过滤新候选像素数"] = filtered_new_pixels
        output["包标识"] = "tracked-" + packet["输出序号"]
        output["任务意图"] = "Mixed"
        self.last_output_contours = contour_material
        if self.base_sequence is None:
            output["包类型"], output["依赖全量序号"], self.base_sequence = "FullSnapshot", None, output["输出序号"]
            output["前置场景版本"] = None
            self.scene_version = 1
            output["簇变化"] = changes
            output["指标"]["重同步全量"] = False
        elif self.full_snapshot_requested:
            output["包类型"], output["依赖全量序号"], self.base_sequence = "FullSnapshot", None, output["输出序号"]
            output["前置场景版本"] = None
            output["簇变化"], self.last_output_contours = self._snapshot_entries()
            output["指标"]["重同步全量"] = True
            self.full_snapshot_requested = False
        elif not meaningful_change:
            # The source-time header proves that the supply chain is alive.  The
            # current schema cannot refresh per-track evidence age in a heartbeat,
            # so no hidden per-track measurement claim is made here.
            output["包类型"], output["依赖全量序号"], output["簇变化"] = "Heartbeat", self.base_sequence, []
            output["前置场景版本"] = str(self.scene_version)
        else:
            output["包类型"], output["依赖全量序号"], output["簇变化"] = "Delta", self.base_sequence, changes
            output["前置场景版本"] = str(self.scene_version)
            self.scene_version += 1
        output["场景版本"] = str(self.scene_version)
        return output

    def active_snapshot(self) -> dict[str, dict]:
        entries, _ = self._snapshot_entries()
        return {entry["相机跟踪候选编号"]: entry for entry in entries}

    def detailed_contour_material(self, identifier: str) -> tuple[dict, bytes]:
        if identifier not in self.tracks:
            raise KeyError(identifier)
        track = self.tracks[identifier]
        if track.contour_material is None:
            raise ValueError("Tracked contour material is unavailable")
        return copy.deepcopy(track.record), bytes(track.contour_material)


def reconstruct(packets: list[dict]) -> dict[str, dict]:
    state: dict[str, dict] = {}
    base: str | None = None
    for packet in packets:
        if packet["包类型"] == "FullSnapshot":
            state, base = {}, packet["输出序号"]
        elif packet["包类型"] == "Delta":
            if base is None or packet["依赖全量序号"] != base:
                raise ValueError("Delta does not have its declared base snapshot")
        else:
            continue
        for entry in packet["簇变化"]:
            identifier = entry["相机跟踪候选编号"]
            if entry["帧内簇编号"] is None:
                if entry["变化类型"] in {"Lost", "Removed"}:
                    state.pop(identifier, None)
                elif packet["包类型"] == "FullSnapshot" and entry["变化类型"] == "Occluded":
                    state[identifier] = copy.deepcopy(entry)
                elif identifier in state:
                    state[identifier]["跟踪状态"] = "Occluded"
                    state[identifier]["时效"] = copy.deepcopy(entry["时效"])
                continue
            state[identifier] = copy.deepcopy(entry)
    return state


def write_packets(input_paths: list[Path], output_root: Path, max_missing_frames: int = 2,
                  confirmation_frames: int = 5, minimum_new_cluster_pixels: int = 1,
                  minimum_retained_cluster_pixels: int = 1,
                  maximum_tentative_match_cost: int = 200_000,
                  occlusion_confirmation_frames: int = 1,
                  allow_tentative_growth_association: bool = False) -> list[Path]:
    if output_root.exists():
        raise ValueError(f"Output already exists: {output_root}")
    tracker = ClusterTracker(max_missing_frames, confirmation_frames, minimum_new_cluster_pixels,
                             minimum_retained_cluster_pixels, maximum_tentative_match_cost,
                             occlusion_confirmation_frames, allow_tentative_growth_association)
    output_root.mkdir(parents=True)
    results = []
    try:
        for index, path in enumerate(input_paths, start=1):
            validate_cluster_packet(path)
            source = json.loads(path.read_text(encoding="utf-8"))
            source_descriptor = source["材料"]["精确轮廓链"]
            source_contours = (path.parent / source_descriptor["文件"]).read_bytes()
            output = tracker.update(source, source_contours)
            target = output_root / f"packet_{index:06d}"
            target.mkdir()
            descriptor = output["材料"]["精确轮廓链"]
            output_contours = tracker.last_output_contours
            if output_contours is None:
                raise ValueError("Tracker did not provide output contour material")
            descriptor["字节数"] = len(output_contours)
            descriptor["SHA256"] = hashlib.sha256(output_contours).hexdigest()
            (target / descriptor["文件"]).write_bytes(output_contours)
            target_packet = target / "packet.json"
            target_packet.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            validate_cluster_packet(target_packet)
            results.append(target_packet)
        return results
    except Exception:
        shutil.rmtree(output_root, ignore_errors=True)
        raise


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", type=Path, nargs="+", help="Ordered FullSnapshot packet.json paths")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-missing-frames", type=int, default=2)
    parser.add_argument("--confirmation-frames", type=int, default=5)
    parser.add_argument("--minimum-new-cluster-pixels", type=int, default=1)
    parser.add_argument("--minimum-retained-cluster-pixels", type=int, default=1)
    parser.add_argument("--maximum-tentative-match-cost", type=int, default=200_000)
    parser.add_argument("--occlusion-confirmation-frames", type=int, default=1)
    parser.add_argument("--allow-tentative-growth-association", action="store_true")
    args = parser.parse_args()
    packets = write_packets(args.inputs, args.output, args.max_missing_frames, args.confirmation_frames,
                            args.minimum_new_cluster_pixels, args.minimum_retained_cluster_pixels,
                            args.maximum_tentative_match_cost, args.occlusion_confirmation_frames,
                            args.allow_tentative_growth_association)
    data = [json.loads(path.read_text(encoding="utf-8")) for path in packets]
    print(json.dumps({"status": "pass", "packets": [str(path) for path in packets], "active_tracks": len(reconstruct(data))}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
