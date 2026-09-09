"""Synthetic startup-window negatives and shared-Source raw capture regression."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import unittest

import numpy as np

from analyze_startup import THRESHOLDS, analyze, decide, measure, metadata_changed, read_png
from test_protocol import fixture


def stable_rows():
    return [{"continuous": True, "color_timestamp_ms": i * 1000 / 30, "valid_percent": 80,
             "tile_valid_percent": [80] * 16, "common_valid_pixels": 2400,
             "valid_flip_percent": 0.1, "depth_delta_p95_m": 0.002,
             "color_mean_abs": 0.1, "color_changed_percent": 0.1} for i in range(16)]


class WindowTests(unittest.TestCase):
    def test_stable_and_warmup(self):
        self.assertEqual(decide(stable_rows())[0], "变化较小候选")
        self.assertEqual(decide(stable_rows()[:15])[0], "积累中")

    def test_all_missing_not_stable(self):
        rows = stable_rows()
        for r in rows:
            r["valid_percent"] = 0
            r["common_valid_pixels"] = 0
        self.assertEqual(decide(rows)[0], "证据不足")

    def test_local_change_with_same_global_count(self):
        rows = stable_rows()
        rows[-1]["tile_valid_percent"] = [60, 100] + [80] * 14
        self.assertEqual(decide(rows)[0], "变化中")

    def test_moving_holes_with_same_coverage(self):
        rows = stable_rows()
        rows[-1]["valid_flip_percent"] = 5
        self.assertEqual(decide(rows)[0], "变化中")

    def test_depth_color_and_exposure_changes(self):
        for key in ("depth_delta_p95_m", "color_mean_abs", "color_changed_percent"):
            rows = stable_rows()
            rows[-1][key] = THRESHOLDS[key] * 2
            self.assertEqual(decide(rows)[0], "变化中")
        rows = stable_rows()
        rows[-1]["exposure_changed"] = True
        self.assertEqual(decide(rows)[0], "变化中")

    def test_gap_recovers_only_after_fresh_window(self):
        rows = stable_rows()
        rows[-1]["continuous"] = False
        self.assertEqual(decide(rows)[0], "序列中断")
        for i in range(1, 16):
            rows.append({**copy.deepcopy(stable_rows()[-1]), "color_timestamp_ms": (15 + i) * 1000 / 30})
            self.assertEqual(decide(rows)[0], "变化较小候选" if i == 15 else "序列中断")

    def test_time_window_and_nonfinite(self):
        rows = stable_rows()
        rows[-1]["color_timestamp_ms"] = 1200
        self.assertEqual(decide(rows)[0], "证据不足")
        rows = stable_rows()
        rows[-1]["depth_delta_p95_m"] = float("nan")
        self.assertEqual(decide(rows)[0], "变化中")

    def test_metrics_preserve_input_and_saturation(self):
        color = np.full((48, 64, 3), 100, np.uint8)
        depth = np.full((48, 64), 1000, np.uint16)
        changed = depth.copy()
        changed[0, 0] = 65535
        changed[0, 1] = 0
        originals = [a.copy() for a in (color, depth, changed)]
        row = measure(color, changed, (color, depth), 0.001)
        self.assertAlmostEqual(row["valid_percent"], 100 * (3072 - 2) / 3072)
        self.assertEqual(row["depth_delta_p95_m"], 0)
        for a, b in zip((color, depth, changed), originals):
            np.testing.assert_array_equal(a, b)

    def test_exposure_missing_not_fabricated(self):
        self.assertFalse(metadata_changed({}, {}))
        source = {"逐流元数据": {"彩图": {"Actual Exposure": {"支持": True, "原始值": "1000"}}}}
        self.assertTrue(metadata_changed(source, {}))
        self.assertFalse(metadata_changed(source, source))

    def test_distance_band_attribution(self):
        color = np.full((48, 64, 3), 100, np.uint8)
        depth = np.full((48, 64), 800, np.uint16)
        depth[:, 32:] = 2400
        current = depth.copy()
        current[:, :32] += 7
        current[:, 32:] += 73
        row = measure(color, current, (color, depth), 0.001)
        self.assertAlmostEqual(row["depth_bands"]["0.3_to_1m"]["delta_p95_m"], 0.007)
        self.assertAlmostEqual(row["depth_bands"]["2_to_3.5m"]["delta_p95_m"], 0.073)
        self.assertIsNone(row["depth_bands"]["above_3.5m"]["delta_p95_m"])


def integration(exe, root):
    color = np.full((48, 64, 3), 100, np.uint8)
    depth = np.full((48, 64), 1000, np.uint16)
    path = fixture(root, "startup", color, depth, frames=20)
    output = root / "probe"
    command = [str(exe.resolve()), "--probe-startup", "--sessions=1", "--frames=20",
               f"--output-root={output.resolve()}", f"--replay-manifest={path.resolve()}"]
    subprocess.run(command, check=True, timeout=30)
    report = json.loads((output / "probe.json").read_text(encoding="utf-8"))
    for r in report["会话"][0]["帧列表"]:
        np.testing.assert_array_equal(read_png(output, r["彩图"]), color)
        np.testing.assert_array_equal(read_png(output, r["深度"]), depth)
        assert r["源信息"]["类型"] == "合成夹具"
    result = analyze(output / "probe.json", root / "analysis")
    assert result["sessions"][0]["first_candidate_index"] == 15
    assert result["sensor_warmup_confirmed"] is False
    digest = hashlib.sha256((output / "probe.json").read_bytes()).hexdigest()
    assert subprocess.run(command, capture_output=True, timeout=30).returncode != 0
    assert hashlib.sha256((output / "probe.json").read_bytes()).hexdigest() == digest
    short = root / "short"
    command[3] = "--frames=21"
    command[4] = f"--output-root={short.resolve()}"
    assert subprocess.run(command, capture_output=True, timeout=30).returncode != 0
    partial = json.loads((short / "probe.json").read_text(encoding="utf-8"))
    assert partial["状态"] == "未完成"
    assert len(partial["会话"][0]["帧列表"]) == 20
    assert partial["会话"][0]["异常事件"][-1]["代码"] == "end_of_source"
    (root / "test_report.json").write_text(json.dumps({"status": "pass", "unit_groups": 10,
        "integration": "shared source, raw equality, offline gate, no overwrite, partial EOF retained"}, indent=2), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--exe", type=Path, default=Path(__file__).resolve().parent.parent / "x64/Release/PixelClusterSensor.exe")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(WindowTests)
    if not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful():
        raise SystemExit(1)
    integration(args.exe, args.output)
    print("PASS raw startup integration")
