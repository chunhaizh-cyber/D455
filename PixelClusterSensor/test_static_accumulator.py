"""Static accumulation invariants, expiry and known-unobservable-change counterexample."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import unittest

import numpy as np

from analyze_startup import read_png
from static_accumulator import CONFIG, StaticAccumulator, evaluate
from test_protocol import fixture


class AccumulatorTests(unittest.TestCase):
    def setUp(self):
        self.depth = np.full((8, 8), 1000, np.uint16)
        self.color = np.full((8, 8, 3), 100, np.uint8)
        self.acc = StaticAccumulator(.001)

    def warm(self):
        for index in range(3):
            self.acc.update(self.depth, self.color, index * 30, index)

    def test_current_wins_and_no_input_mutation(self):
        self.warm()
        depth, color = self.depth.copy(), self.color.copy()
        depth[3, 3] = 2000
        out, _ = self.acc.update(depth, color, 90, 3)
        np.testing.assert_array_equal(out["combined_depth16"], depth)
        np.testing.assert_array_equal(out["raw_depth16"], depth)
        np.testing.assert_array_equal(out["color_rgb"], color)
        self.assertEqual(out["source_index"][3, 3], 3)
        self.assertEqual(out["evidence_state"][3, 3], 1)
        self.acc.update(self.depth, self.color, 120, 4)
        self.assertEqual(out["combined_depth16"][3, 3], 2000)
        self.assertEqual(depth[3, 3], 2000)

    def test_hole_reuses_only_confirmed_raw_with_age(self):
        self.warm()
        hole = self.depth.copy()
        hole[3, 3] = 0
        out, _ = self.acc.update(hole, self.color, 90, 3)
        self.assertEqual(out["raw_depth16"][3, 3], 0)
        self.assertEqual(out["combined_depth16"][3, 3], 1000)
        self.assertEqual(out["evidence_state"][3, 3], 2)
        self.assertEqual(out["source_index"][3, 3], 2)
        self.assertEqual(out["last_observation_age_ms"][3, 3], 30)

    def test_no_recursive_refresh_of_history_age(self):
        self.warm()
        hole = self.depth.copy()
        hole[3, 3] = 0
        for index in range(3, 10):
            out, _ = self.acc.update(hole, self.color, index * 30, index)
            if index < 9:
                self.assertEqual(out["source_index"][3, 3], 2)
                self.assertEqual(out["last_observation_age_ms"][3, 3], (index - 2) * 30)
            else:
                self.assertEqual(out["evidence_state"][3, 3], 0)
                self.assertEqual(out["combined_depth16"][3, 3], 0)
                self.assertEqual(out["source_index"][3, 3], -1)

    def test_one_sample_and_never_seen_not_invented(self):
        self.acc.update(self.depth, self.color, 0, 0)
        hole = self.depth.copy()
        hole[3, 3] = 0
        out, _ = self.acc.update(hole, self.color, 30, 1)
        self.assertEqual(out["combined_depth16"][3, 3], 0)
        acc = StaticAccumulator(.001)
        for i in range(20):
            out, _ = acc.update(hole, self.color, i * 30, i)
        self.assertEqual(out["combined_depth16"][3, 3], 0)

    def test_out_of_range_and_saturation_preserved(self):
        self.warm()
        depth = self.depth.copy()
        depth[1, 1], depth[2, 2] = 5000, 65535
        out, _ = self.acc.update(depth, self.color, 90, 3)
        self.assertEqual(out["combined_depth16"][1, 1], 5000)
        self.assertEqual(out["evidence_state"][1, 1], 3)
        self.assertEqual(out["combined_depth16"][2, 2], 65535)
        self.assertEqual(out["evidence_state"][2, 2], 4)
        depth[1, 1] = depth[2, 2] = 0
        out, _ = self.acc.update(depth, self.color, 120, 4)
        self.assertEqual(out["combined_depth16"][1, 1], 0)
        self.assertEqual(out["combined_depth16"][2, 2], 0)

    def test_boundary_conflict_invalidates_neighbor_hole(self):
        self.warm()
        depth = self.depth.copy()
        depth[3, 3], depth[3, 4] = 0, 2000
        out, diag = self.acc.update(depth, self.color, 90, 3)
        self.assertEqual(out["combined_depth16"][3, 3], 0)
        self.assertEqual(out["combined_depth16"][3, 4], 2000)
        self.assertGreater(diag["invalidated_pixels"], 0)

    def test_color_change_veto_with_unregistered_shapes(self):
        self.warm()
        hole = self.depth.copy()
        hole[3, 3] = 0
        out, diag = self.acc.update(hole, self.color + 40, 90, 3)
        self.assertEqual(diag["reset_reason"], "color_change")
        self.assertEqual(out["combined_depth16"][3, 3], 0)
        acc = StaticAccumulator(.001)
        out, _ = acc.update(self.depth, np.zeros((6, 10, 3), np.uint8), 0, 0)
        self.assertEqual(out["combined_depth16"].shape, (8, 8))

    def test_small_cumulative_color_drift_reaches_epoch_veto(self):
        for i in range(5):
            _, diag = self.acc.update(self.depth, self.color + i, i * 30, i)
        self.assertEqual(diag["reset_reason"], "color_change")

    def test_source_gap_resets(self):
        for kwargs, t, i in (({"source_continuous": False}, 90, 3), ({}, 120, 3), ({}, 90, 5), ({}, 60, 3)):
            self.setUp()
            self.warm()
            hole = self.depth.copy()
            hole[3, 3] = 0
            out, diag = self.acc.update(hole, self.color, t, i, **kwargs)
            self.assertEqual(diag["reset_reason"], "source_discontinuity")
            self.assertEqual(out["evidence_state"][3, 3], 0)

    def test_slow_depth_motion_has_no_numeric_averaging_lag(self):
        for i in range(20):
            depth = self.depth + i * 10
            out, _ = self.acc.update(depth, self.color, i * 30, i)
            np.testing.assert_array_equal(out["combined_depth16"], depth)

    def test_unobservable_change_remains_known_limit(self):
        self.warm()
        # A same-color object can replace the old surface while giving no new depth.
        # The sensor evidence cannot distinguish this from a static depth hole.
        depth = self.depth.copy()
        depth[3, 3] = 0
        out, _ = self.acc.update(depth, self.color, 90, 3)
        actual_new_depth = 2000
        self.assertEqual(out["evidence_state"][3, 3], 2)
        self.assertNotEqual(out["combined_depth16"][3, 3], actual_new_depth)

    def test_state_memory_bounded(self):
        self.warm()
        size = self.acc.state_bytes()
        for i in range(3, 303):
            self.acc.update(self.depth, self.color, i * 30, i)
        self.assertEqual(size, self.acc.state_bytes())
        self.assertLessEqual(int(self.acc.support.max()), CONFIG["required_observations"])

    def test_validation_and_independent_sessions(self):
        with self.assertRaises(ValueError):
            StaticAccumulator(float("nan"))
        with self.assertRaises(ValueError):
            self.acc.update(self.depth.astype(np.float32), self.color, 0, 0)
        self.warm()
        with self.assertRaises(ValueError):
            self.acc.update(self.depth, self.color, float("nan"), 3)
        with self.assertRaises(ValueError):
            self.acc.update(self.depth[:4], self.color, 90, 3)
        other = StaticAccumulator(.001)
        out, _ = other.update(np.zeros_like(self.depth), self.color, 0, 0)
        self.assertFalse(out["combined_depth16"].any())


def validate_samples(probe, analysis):
    manifest_data = probe.read_bytes()
    manifest = json.loads(manifest_data)
    report = json.loads((analysis / "summary.json").read_text(encoding="utf-8"))
    assert hashlib.sha256(manifest_data).hexdigest() == report["probe_sha256"]
    assert report["status"] == "measured" and report["promotion"] is False
    assert hashlib.sha256((analysis / "config_snapshot.json").read_bytes()).hexdigest() == report["config_sha256"]
    config = json.loads((analysis / "config_snapshot.json").read_text())["config"]
    sessions = {s["编号"]: s for s in manifest["会话"]}
    historical_pixels = sample_count = 0
    for sample in report["samples"]:
        sample_path = (analysis / sample["file"]).resolve()
        assert sample_path.is_relative_to(analysis.resolve())
        assert hashlib.sha256(sample_path.read_bytes()).hexdigest() == sample["sha256"]
        session = sessions[sample["session"]]
        index = sample["index"]
        item = session["帧列表"][index]
        scale = session["标定与源状态"]["深度单位米"]
        raw, color = read_png(probe.parent, item["深度"]), read_png(probe.parent, item["彩图"])
        with np.load(sample_path, allow_pickle=False) as arrays:
            np.testing.assert_array_equal(arrays["raw_depth16"], raw)
            np.testing.assert_array_equal(arrays["color_rgb"], color)
            state, sources = arrays["evidence_state"], arrays["source_index"]
            combined, age = arrays["combined_depth16"], arrays["last_observation_age_ms"]
            assert state.shape == raw.shape and np.isin(state, [0, 1, 2, 3, 4]).all()
            valid = (raw > 0) & (raw < 65535)
            usable = valid & (raw * scale >= config["near_m"]) & (raw * scale < config["far_m"])
            np.testing.assert_array_equal(state == 1, usable)
            np.testing.assert_array_equal(state == 3, valid & ~usable)
            np.testing.assert_array_equal(state == 4, raw == 65535)
            history = state == 2
            np.testing.assert_array_equal(combined[~history], raw[~history])
            assert np.all(raw[history] == 0) and np.all(age[~history] == 0)
            assert np.all(sources[valid | (raw == 65535)] == index)
            assert np.all(sources[state == 0] == -1)
            assert np.all(arrays["support_count"][history] >= config["required_observations"])
            assert np.all((age[history] > 0) & (age[history] <= config["history_max_age_ms"]))
            for old_index in np.unique(sources[history]):
                assert 0 <= old_index < index
                mask = history & (sources == old_index)
                old_item = session["帧列表"][int(old_index)]
                old = read_png(probe.parent, old_item["深度"])
                np.testing.assert_array_equal(combined[mask], old[mask])
                assert np.all((old[mask] * scale >= config["near_m"]) & (old[mask] * scale < config["far_m"]))
                expected_age = item["源信息"]["深度时间戳毫秒"] - old_item["源信息"]["深度时间戳毫秒"]
                np.testing.assert_allclose(age[mask], expected_age, atol=1e-3)
            historical_pixels += int(history.sum())
            sample_count += 1
    assert sample_count > 0
    return {"status": "pass", "samples": sample_count, "historical_pixel_observations": historical_pixels,
            "scope": "value/source/age/current-color readback, not physical correctness of retained history"}


def integration(exe, root):
    color = np.full((24, 32, 3), 100, np.uint8)
    depth = np.full((24, 32), 1200, np.uint16)
    raw = fixture(root, "accumulator", color, depth, frames=48)
    probe = root / "raw"
    subprocess.run([str(exe.resolve()), "--probe-startup", "--sessions=1", "--frames=48",
                    f"--replay-manifest={raw.resolve()}", f"--output-root={probe.resolve()}"], check=True, timeout=30)
    output = root / "analysis"
    report = evaluate(probe / "probe.json", output, repeats=1, save_every_n=1)
    assert report["status"] == "measured" and report["promotion"] is False
    assert report["quality"]["current_usable_percent"]["p50"] == 100
    for sample in report["samples"]:
        path = output / sample["file"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == sample["sha256"]
        with np.load(path, allow_pickle=False) as arrays:
            np.testing.assert_array_equal(arrays["raw_depth16"], depth)
            np.testing.assert_array_equal(arrays["combined_depth16"], depth)
            np.testing.assert_array_equal(arrays["color_rgb"], color)
            np.testing.assert_array_equal(arrays["evidence_state"], 1)
            np.testing.assert_array_equal(arrays["source_index"], sample["index"])
    validate_samples(probe / "probe.json", output)
    (root / "test_report.json").write_text(json.dumps({"status": "pass", "unit_groups": 13,
        "integration": "raw replay, every-frame output and independent readback",
        "known_limit": "unobservable same-color replacement can retain stale historical candidate until expiry"}, indent=2), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--exe", type=Path, default=Path(__file__).resolve().parent.parent / "x64/Release/PixelClusterSensor.exe")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--review-probe", type=Path)
    parser.add_argument("--review-analysis", type=Path)
    args = parser.parse_args()
    if bool(args.review_probe) != bool(args.review_analysis):
        parser.error("review requires both probe and analysis")
    args.output.mkdir(parents=True, exist_ok=False)
    if args.review_probe:
        review = validate_samples(args.review_probe, args.review_analysis)
        (args.output / "readback_report.json").write_text(json.dumps(review, indent=2), encoding="utf-8")
        print(json.dumps(review, indent=2))
        raise SystemExit(0)
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(AccumulatorTests))
    if not result.wasSuccessful():
        raise SystemExit(1)
    integration(args.exe, args.output)
    print("PASS static accumulation integration")
