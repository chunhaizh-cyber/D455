"""Export bounded PCS.RawSequence/1 replay material from published PCS observations.

The C++ service remains the sole camera owner.  This exporter copies only the
source color and raw depth images plus the calibration and per-frame source
time evidence needed for an auditable directory replay.  It never converts
filled depth, cluster labels, or host publication time into source evidence.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import time

from client import Client


def _copy_material(frame: dict, key: str, target: Path) -> None:
    descriptor = frame["材料"][key]
    source = Path(frame["_path"]).parent / descriptor["文件"]
    if descriptor["编码"] not in {"PNG_RGB8", "PNG_U16"}:
        raise ValueError(f"Unexpected {key} encoding: {descriptor['编码']}")
    shutil.copy2(source, target)


def export_sequence(*, executable: Path, output: Path, frames: int, replay: Path | None,
                    serial: str = "") -> dict:
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if not 1 <= frames <= 2048:
        raise ValueError("frames must be 1..2048")
    output.mkdir(parents=True)
    frame_root = output / "frames"
    frame_root.mkdir()
    report = {"format": "PCS.RawSequenceExport/1", "status": "running", "requested_frames": frames,
              "source": "directory_replay" if replay else "live_camera", "copied_frames": [], "error": None}
    try:
        with Client(executable, output / "observation_source", max_packets=frames + 1) as client:
            parameters = ({"来源": "目录回放", "清单": str(replay.resolve())} if replay
                          else {"来源": "实时相机", "设备序列号": serial})
            opened = client.call("打开设备", parameters)
            actual = opened["实际配置与标定"]
            calibration = {
                "彩图内参": actual["彩图内参"], "深度内参": actual["深度内参"],
                "深度到彩图外参": actual["深度到彩图外参"], "深度单位米": actual["深度单位米"],
            }
            previous = -1
            for index in range(1, frames + 1):
                reference = client.call("获取单帧观察")
                source_path = Path(reference["材料路径"])
                frame = json.loads(source_path.read_text(encoding="utf-8"))
                frame["_path"] = str(source_path)
                source = frame["源信息"]
                source_number = int(source["源帧号"])
                if source_number <= previous:
                    raise ValueError("Source frame numbers are not strictly increasing")
                previous = source_number
                if source["彩图时间域"] != source["深度时间域"]:
                    raise ValueError("Cannot export mixed source time domains")
                name = f"{index:06d}"
                _copy_material(frame, "颜色图", frame_root / f"{name}_color.png")
                _copy_material(frame, "原始深度", frame_root / f"{name}_depth.png")
                report["copied_frames"].append({
                    "彩图": f"frames/{name}_color.png", "深度": f"frames/{name}_depth.png",
                    "源帧号": source["源帧号"], "彩图时间戳毫秒": source["彩图时间戳毫秒"],
                    "深度时间戳毫秒": source["深度时间戳毫秒"], "时间域": source["彩图时间域"],
                })
            client.call("关闭设备")
        sequence = {"格式": "PCS.RawSequence/1", "材料来源": "历史回放", "设备标识": actual["设备标识"],
                    **calibration, "帧列表": report["copied_frames"]}
        (output / "sequence.json").write_text(json.dumps(sequence, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        report["status"] = "pass"
        report["sequence"] = str((output / "sequence.json").resolve())
        report["captured_unix_ms"] = int(time.time() * 1000)
    except Exception as error:
        report["status"] = "fail"
        report["error"] = str(error)
        raise
    finally:
        (output / "export_manifest.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    project = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, default=project / "x64/Release/PixelClusterSensor.exe")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--camera", action="store_true")
    source.add_argument("--replay", type=Path)
    parser.add_argument("--serial", default="")
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(export_sequence(executable=args.exe, output=args.output.resolve(), frames=args.frames,
                                     replay=args.replay, serial=args.serial), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
