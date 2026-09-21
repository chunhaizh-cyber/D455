"""Bounded P3 shadow control and scheduling layer for one immutable cluster stream."""

from __future__ import annotations

import base64
from collections import deque
import copy
from dataclasses import dataclass, field
import hashlib
import json
from pathlib import Path
import shutil

from cluster_stream import write_tracked_packet
from cluster_tracker import ClusterTracker


@dataclass
class Task:
    identifier: str
    kind: str
    track_id: str | None
    remaining_frames: int | None
    binding_token: str | None = None
    queue: deque[dict] = field(default_factory=deque)
    queue_bytes: int = 0
    active: bool = True


class ClusterControlSession:
    """Testable shadow scheduler; it does not own a camera or create world identity."""

    def __init__(self, output: Path, tracker: ClusterTracker | None = None,
                 maximum_results_per_task: int = 64, maximum_result_bytes_per_task: int = 4 * 1024 * 1024,
                 maximum_result_age_frames: int = 120,
                 maximum_material_bytes: int = 16 * 1024 * 1024) -> None:
        if not 1 <= maximum_results_per_task <= 1024:
            raise ValueError("maximum_results_per_task must be 1..1024")
        if not 1 <= maximum_material_bytes <= 256 * 1024 * 1024:
            raise ValueError("maximum_material_bytes is out of range")
        if not 1024 <= maximum_result_bytes_per_task <= 64 * 1024 * 1024:
            raise ValueError("maximum_result_bytes_per_task is out of range")
        if not 1 <= maximum_result_age_frames <= 10000:
            raise ValueError("maximum_result_age_frames must be 1..10000")
        self.output = output.resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        self.tracker = tracker or ClusterTracker()
        self.maximum_results_per_task = maximum_results_per_task
        self.maximum_result_bytes_per_task = maximum_result_bytes_per_task
        self.maximum_result_age_frames = maximum_result_age_frames
        self.maximum_material_bytes = maximum_material_bytes
        self.tasks: dict[str, Task] = {}
        self.materials: dict[str, dict] = {}
        self.material_bytes = 0
        self.next_task = 1
        self.next_material = 1
        self.packet_count = 0
        self.current_session: str | None = None
        self.current_epoch: str | None = None

    def _new_task(self, kind: str, track_id: str | None = None,
                  remaining_frames: int | None = None, binding_token: str | None = None) -> Task:
        identifier = str(self.next_task)
        self.next_task += 1
        task = Task(identifier, kind, track_id, remaining_frames, binding_token)
        self.tasks[identifier] = task
        return task

    def _active_scan_tasks(self) -> list[Task]:
        return [task for task in self.tasks.values() if task.active and task.kind == "Scan"]

    def _require_scan(self) -> None:
        if not self._active_scan_tasks():
            raise ValueError("scan_required")

    def command(self, name: str, parameters: dict | None = None) -> dict:
        parameters = parameters or {}
        if name == "开始扫描":
            if parameters.get("区域", "全画面") != "全画面":
                raise ValueError("only_full_frame_scan_is_implemented")
            task = self._new_task("Scan")
            return {"任务编号": task.identifier, "区域": "全画面", "状态": "运行中"}
        if name == "停止扫描":
            task = self._task(parameters["任务编号"], "Scan")
            if len(self._active_scan_tasks()) == 1 and any(
                    item.active and item.kind in {"Observe", "Track"} for item in self.tasks.values()):
                raise ValueError("scan_required_by_active_tasks")
            task.active = False
            return {"任务编号": task.identifier, "状态": "已停止"}
        if name == "观察簇候选":
            self._require_scan()
            frames = int(parameters.get("截止帧数", 30))
            if not 1 <= frames <= 120:
                raise ValueError("截止帧数 must be 1..120")
            task = self._new_task("Observe", str(parameters["相机跟踪候选编号"]), frames)
            return {"任务编号": task.identifier, "状态": "等待候选"}
        if name == "开始跟踪簇候选":
            self._require_scan()
            frames = int(parameters.get("最长帧数", 300))
            if not 1 <= frames <= 10000:
                raise ValueError("最长帧数 must be 1..10000")
            token = parameters.get("自我绑定令牌")
            if token is not None and (not isinstance(token, str) or not 1 <= len(token.encode("utf-8")) <= 256):
                raise ValueError("自我绑定令牌 byte length is out of range")
            task = self._new_task("Track", str(parameters["相机跟踪候选编号"]), frames, token)
            return {"任务编号": task.identifier, "状态": "运行中"}
        if name == "停止跟踪簇候选":
            if "任务编号" in parameters:
                task = self._task(parameters["任务编号"], "Track")
            else:
                matches = [item for item in self.tasks.values() if item.kind == "Track" and item.active and
                           item.track_id == str(parameters["相机跟踪候选编号"])]
                if len(matches) != 1:
                    raise ValueError("tracking_task_not_unique")
                task = matches[0]
            task.active = False
            return {"任务编号": task.identifier, "相机跟踪候选编号": task.track_id,
                    "状态": "已停止", "候选状态未被伪造为退役": True}
        if name == "请求全量快照":
            if self.current_session is None:
                raise ValueError("no_cluster_stream_state")
            if parameters.get("预期会话") != self.current_session:
                raise ValueError("session_mismatch")
            expected = parameters.get("预期跟踪时期")
            if str(expected) != self.current_epoch:
                raise ValueError("tracking_epoch_mismatch")
            self.tracker.request_full_snapshot()
            return {"状态": "已请求", "将在下一源帧发布": True}
        if name == "读取簇级结果":
            task = self._task(parameters["任务编号"])
            maximum = int(parameters.get("最多包数", 8))
            if not 1 <= maximum <= 64:
                raise ValueError("最多包数 must be 1..64")
            results = []
            for _ in range(min(maximum, len(task.queue))):
                result = task.queue.popleft()
                task.queue_bytes -= self._result_size(result)
                results.append(result)
            return {"任务编号": task.identifier, "任务类型": task.kind, "结果": results,
                    "队列剩余": len(task.queue), "状态": "运行中" if task.active else "完成"}
        if name == "读取详细材料":
            handle = self.materials.get(str(parameters["句柄"]))
            if handle is None:
                raise ValueError("unknown_material")
            if parameters.get("预期SHA256") != handle["SHA256"]:
                raise ValueError("material_hash_mismatch")
            offset = int(parameters.get("偏移", 0))
            length = int(parameters.get("字节数", len(handle["数据"]) - offset))
            if offset < 0 or length < 0 or length > 1024 * 1024 or offset + length > len(handle["数据"]):
                raise ValueError("material_range_invalid")
            return {"句柄": parameters["句柄"], "偏移": offset, "字节数": length,
                    "数据Base64": base64.b64encode(handle["数据"][offset:offset + length]).decode("ascii")}
        if name == "释放详细材料":
            identifier = str(parameters["句柄"])
            handle = self.materials.pop(identifier, None)
            if handle is None:
                raise ValueError("unknown_material")
            self.material_bytes -= len(handle["数据"])
            return {"句柄": identifier, "状态": "已释放"}
        raise ValueError("unsupported_command")

    def _task(self, identifier: str, kind: str | None = None) -> Task:
        task = self.tasks.get(str(identifier))
        if task is None or (kind is not None and task.kind != kind):
            raise ValueError("unknown_task")
        return task

    def _ensure_queue_capacity(self) -> None:
        for task in self.tasks.values():
            if not task.active:
                continue
            if len(task.queue) >= self.maximum_results_per_task:
                raise ValueError(f"result_queue_backpressure:{task.identifier}:count")
            if task.queue_bytes >= self.maximum_result_bytes_per_task:
                raise ValueError(f"result_queue_backpressure:{task.identifier}:bytes")
            if task.queue and self.packet_count - task.queue[0]["调度帧序号"] >= self.maximum_result_age_frames:
                raise ValueError(f"result_queue_backpressure:{task.identifier}:age")

    @staticmethod
    def _result_size(result: dict) -> int:
        return len(json.dumps(result, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8"))

    def _enqueue(self, task: Task, result: dict) -> None:
        size = self._result_size(result)
        if size > self.maximum_result_bytes_per_task or task.queue_bytes + size > self.maximum_result_bytes_per_task:
            raise ValueError(f"result_queue_backpressure:{task.identifier}:bytes")
        task.queue.append(result)
        task.queue_bytes += size

    def _make_material(self, track_id: str, packet_path: Path) -> dict:
        entry, raw = self.tracker.detailed_contour_material(track_id)
        if self.material_bytes + len(raw) > self.maximum_material_bytes:
            raise ValueError("material_budget")
        identifier = str(self.next_material)
        self.next_material += 1
        digest = hashlib.sha256(raw).hexdigest()
        self.materials[identifier] = {"数据": raw, "SHA256": digest, "相机跟踪候选编号": track_id,
                                      "包路径": str(packet_path), "簇记录": entry}
        self.material_bytes += len(raw)
        return {"句柄": identifier, "编码": "PCS.ContourChain8/1", "字节数": len(raw), "SHA256": digest}

    def process_source(self, packet: dict, contours: bytes) -> Path:
        checkpoint = {
            "tracker": copy.deepcopy(self.tracker), "tasks": copy.deepcopy(self.tasks),
            "materials": copy.deepcopy(self.materials), "material_bytes": self.material_bytes,
            "next_material": self.next_material, "packet_count": self.packet_count,
            "current_session": self.current_session, "current_epoch": self.current_epoch,
        }
        target = self.output / "cluster_packets" / f"packet_{self.packet_count + 1:06d}"
        try:
            return self._process_source(packet, contours)
        except Exception:
            self.tracker = checkpoint["tracker"]
            self.tasks = checkpoint["tasks"]
            self.materials = checkpoint["materials"]
            self.material_bytes = checkpoint["material_bytes"]
            self.next_material = checkpoint["next_material"]
            self.packet_count = checkpoint["packet_count"]
            self.current_session = checkpoint["current_session"]
            self.current_epoch = checkpoint["current_epoch"]
            shutil.rmtree(target, ignore_errors=True)
            raise

    def _process_source(self, packet: dict, contours: bytes) -> Path:
        self._require_scan()
        self._ensure_queue_capacity()
        identity = (packet["会话标识"], packet["跟踪时期"])
        if self.current_session is not None and identity != (self.current_session, self.current_epoch):
            raise ValueError("source_tracking_identity_changed")
        self.current_session, self.current_epoch = identity
        source_sequence = packet["输出序号"]
        packet = copy.deepcopy(packet)
        packet["输出序号"] = packet["场景版本"] = str(self.packet_count + 1)
        packet["包标识"] = f"cluster-control-source-{self.packet_count + 1}"
        packet.setdefault("指标", {})["控制来源输出序号"] = source_sequence
        tracked = self.tracker.update(packet, contours)
        if self.tracker.last_output_contours is None:
            raise ValueError("Tracker did not provide output contour material")
        self.packet_count += 1
        target = self.output / "cluster_packets" / f"packet_{self.packet_count:06d}"
        packet_path = write_tracked_packet(tracked, self.tracker.last_output_contours, target)
        common = {"包路径": str(packet_path), "包标识": tracked["包标识"],
                  "输出序号": tracked["输出序号"], "场景版本": tracked["场景版本"],
                  "调度帧序号": self.packet_count}
        state = self.tracker.active_snapshot()
        for task in self.tasks.values():
            if not task.active:
                continue
            if task.kind == "Scan":
                self._enqueue(task, {**common, "任务意图": "Scan", "全画面扫描保留": True})
            elif task.kind == "Track":
                self._enqueue(task, {**common, "任务意图": "Track", "相机跟踪候选编号": task.track_id,
                                     "自我绑定令牌": task.binding_token, "候选状态": state.get(task.track_id)})
                task.remaining_frames -= 1
                if task.remaining_frames == 0:
                    task.active = False
            elif task.kind == "Observe":
                if task.track_id in state:
                    try:
                        detail = self._make_material(task.track_id, packet_path)
                        self._enqueue(task, {**common, "任务意图": "Observe", "相机跟踪候选编号": task.track_id,
                                             "簇记录": state[task.track_id], "详细材料": detail})
                    except ValueError as error:
                        self._enqueue(task, {**common, "任务意图": "Observe", "错误": str(error)})
                    task.active = False
                else:
                    task.remaining_frames -= 1
                    if task.remaining_frames == 0:
                        self._enqueue(task, {**common, "任务意图": "Observe", "错误": "candidate_not_found"})
                        task.active = False
        return packet_path

    def write_manifest(self) -> Path:
        path = self.output / "control_manifest.json"
        data = {"格式": "PCS.ClusterControlShadow/1", "包数": self.packet_count,
                "任务数": len(self.tasks), "未释放材料数": len(self.materials),
                "未释放材料字节数": self.material_bytes,
                "边界": ["Python影子调度", "不拥有相机", "不创建世界身份", "不证明实时性能"]}
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return path
