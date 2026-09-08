"""Local JSONL client. The child process is the only RealSense pipeline owner."""

from __future__ import annotations

import argparse
from collections import deque
import json
from pathlib import Path
import queue
import subprocess
import threading
import time


class Client:
    def __init__(self, executable: Path, output: Path, max_packets: int = 128, max_bytes: int = 268435456):
        self.process = subprocess.Popen(
            [str(executable.resolve()), "--stdio", f"--output-root={output.resolve()}",
             f"--max-packets={max_packets}", f"--max-bytes={max_bytes}"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, encoding="utf-8", bufsize=1,
        )
        self.replies: queue.Queue = queue.Queue()
        self.errors: deque[str] = deque(maxlen=100)
        self.session = ""
        self.counter = 0
        self.revision = "1"
        self.output_thread = threading.Thread(target=self._read_output, daemon=True)
        self.error_thread = threading.Thread(target=self._read_errors, daemon=True)
        self.output_thread.start()
        self.error_thread.start()

    def _read_output(self):
        try:
            for line in self.process.stdout:
                try:
                    self.replies.put(json.loads(line))
                except json.JSONDecodeError:
                    self.replies.put(RuntimeError(f"Non-JSON protocol output: {line!r}"))
        finally:
            self.replies.put(EOFError("Sensor output closed"))

    def _read_errors(self):
        for line in self.process.stderr:
            self.errors.append(line.rstrip())

    def request(self, command: str, parameters=None, **extra) -> dict:
        self.counter += 1
        result = {"协议": "PCS.Control/1", "请求编号": str(self.counter), "指令": command,
                  "参数": parameters or {}, "截止Unix毫秒": int(time.time() * 1000) + 60000}
        if self.session:
            result["会话标识"] = self.session
        result.update(extra)
        return result

    def send_raw(self, text: str, timeout: float = 30) -> dict:
        self.process.stdin.write(text + "\n")
        self.process.stdin.flush()
        try:
            result = self.replies.get(timeout=timeout)
        except queue.Empty as exc:
            raise TimeoutError(f"No response. stderr={list(self.errors)!r}") from exc
        if isinstance(result, Exception):
            raise result
        return result

    def send(self, request: dict, timeout: float = 30) -> dict:
        return self.send_raw(json.dumps(request, ensure_ascii=False), timeout)

    def call(self, command: str, parameters=None, **extra) -> dict:
        reply = self.send(self.request(command, parameters, **extra))
        if reply["状态"] != "完成":
            raise RuntimeError(json.dumps(reply, ensure_ascii=False))
        result = reply["结果"]
        if command == "打开设备":
            self.session = result["会话标识"]
            self.revision = result["配置版本"]
        elif command == "关闭设备":
            self.session = ""
        elif command == "设置处理配置":
            self.revision = result["配置版本"]
        return result

    def close(self):
        if self.process.poll() is None:
            try:
                self.process.stdin.close()
            except BrokenPipeError:
                pass
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.output_thread.join(timeout=2)
        self.error_thread.join(timeout=2)
        self.process.stdout.close()
        self.process.stderr.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def main():
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="像素簇视觉外设客户端")
    parser.add_argument("--exe", type=Path, default=root / "x64/Release/PixelClusterSensor.exe")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--camera", action="store_true", help="打开真实相机")
    source.add_argument("--replay", type=Path, help="PCS.RawSequence/1清单")
    source.add_argument("--list-devices", action="store_true")
    parser.add_argument("--serial", default="")
    parser.add_argument("--frames", type=int, default=1)
    parser.add_argument("--output", type=Path, default=root / ".codex_tmp/PixelClusterSensor/client")
    parser.add_argument("--validate", action="store_true", help="独立读回检查，需要NumPy和Pillow")
    args = parser.parse_args()
    if not 1 <= args.frames <= 128:
        parser.error("--frames must be between 1 and 128")
    with Client(args.exe, args.output) as client:
        if args.list_devices:
            print(json.dumps(client.call("查询设备能力"), ensure_ascii=False, indent=2))
            return
        parameters = ({"来源": "实时相机", "设备序列号": args.serial} if args.camera
                      else {"来源": "目录回放", "清单": str(args.replay.resolve())})
        client.call("打开设备", parameters)
        for _ in range(args.frames):
            result = client.call("获取单帧观察")
            if args.validate:
                from validate_packet import validate_packet
                result["独立读回校验"] = validate_packet(Path(result["材料路径"]), reference=result)
            print(json.dumps(result, ensure_ascii=False))
        client.call("关闭设备")


if __name__ == "__main__":
    main()
