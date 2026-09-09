"""Known-truth negatives: lower temporal variance does not certify safe filtering."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import unittest

import numpy as np

from temporal_estimator import CONFIG, REASONS, boundary_mask, cell_starts, color_motion, continuous, estimate, evaluate
from test_protocol import fixture


def run_estimate(depth, method="mean", **kw):
    return estimate(depth, np.arange(len(depth)) * (1000 / 30), 0.001, method, **kw)


class EstimatorTests(unittest.TestCase):
    def test_constant_and_no_input_mutation(self):
        d = np.full((5, 8, 8), 1000, np.uint16)
        original = d.copy()
        for method in CONFIG["methods"]:
            r = run_estimate(d, method)
            np.testing.assert_array_equal(r["estimate_m"], 1)
            np.testing.assert_array_equal(r["support_count"], 5)
            np.testing.assert_allclose(r["oldest_sample_age_ms"], 400 / 3, atol=1e-4)
            np.testing.assert_array_equal(r["raw_current"], d[-1])
        np.testing.assert_array_equal(d, original)

    def test_invalid_last_not_filled_from_history(self):
        d = np.full((5, 8, 8), 1000, np.uint16)
        d[-1, 2, 2] = 0
        d[-1, 3, 3] = 65535
        for method in CONFIG["methods"]:
            r = run_estimate(d, method)
            self.assertEqual(r["estimate_state"][2, 2], 0)
            self.assertEqual(r["estimate_state"][3, 3], 0)
            self.assertEqual(r["raw_current"][3, 3], 65535)

    def test_support_threshold_and_valid_only_average(self):
        d = np.full((5, 8, 8), 1000, np.uint16)
        d[0, 2, 2] = 0
        d[:2, 3, 3] = 65535
        r = run_estimate(d)
        self.assertEqual(r["estimate_m"][2, 2], 1)
        self.assertEqual(r["support_count"][2, 2], 4)
        self.assertEqual(r["estimate_state"][3, 3], 0)

    def test_out_of_range_not_promoted(self):
        d = np.full((5, 8, 8), 1000, np.uint16)
        d[-1, 2, 2] = 5000
        d[0, 3, 3] = 8000
        r = run_estimate(d)
        self.assertTrue(r["reject_reason"][2, 2] & REASONS["current_out_of_range"])
        self.assertTrue(r["reject_reason"][3, 3] & REASONS["history_out_of_range"])

    def test_median_resists_single_outlier_but_guard_abstains(self):
        d = np.full((5, 8, 8), 1000, np.uint16)
        d[1] = 1500
        self.assertAlmostEqual(float(run_estimate(d)["estimate_m"][0, 0]), 1.1)
        self.assertEqual(run_estimate(d, "median")["estimate_m"][0, 0], 1)
        self.assertEqual(run_estimate(d, "guarded_median")["estimate_state"][0, 0], 0)

    def test_step_and_reappearing_background_reject(self):
        d = np.full((5, 8, 8), 1000, np.uint16)
        d[-1, 2:6, 2:6] = 2000
        self.assertLess(run_estimate(d)["estimate_m"][3, 3], 1.3)
        self.assertEqual(run_estimate(d, "median")["estimate_m"][3, 3], 1)
        r = run_estimate(d, "guarded_median")
        self.assertEqual(r["estimate_state"][3, 3], 0)
        self.assertEqual(r["raw_current"][3, 3], 2000)

    def test_gradual_motion_guard_is_not_motion_proof(self):
        d = np.broadcast_to((1000 + np.arange(5) * 10)[:, None, None], (5, 8, 8)).astype(np.uint16).copy()
        r = run_estimate(d, "guarded_median")
        self.assertEqual(r["estimate_state"][0, 0], 1)
        self.assertGreater(1.04 - r["estimate_m"][0, 0], 0.019)

    def test_no_spatial_boundary_blur(self):
        d = np.full((5, 8, 8), 1000, np.uint16)
        d[:, :, 4:] = 2000
        for method in CONFIG["methods"]:
            r = run_estimate(d, method)
            np.testing.assert_array_equal(r["estimate_m"][:, :4], 1)
            np.testing.assert_array_equal(r["estimate_m"][:, 4:], 2)
        edge = boundary_mask(d[-1], 0.001)
        self.assertTrue(edge[:, 3:5].all())
        self.assertFalse(edge[:, 0].any())

    def test_color_veto(self):
        d = np.full((5, 8, 8), 1000, np.uint16)
        rgb = np.full((5, 8, 8, 3), 100, np.uint8)
        self.assertFalse(color_motion(rgb))
        rgb[-1] = 140
        self.assertTrue(color_motion(rgb))
        self.assertFalse(run_estimate(d, "guarded_median", color_changed=True)["estimate_state"].any())

    def test_timestamps_and_disjoint_cells(self):
        d = np.full((5, 8, 8), 1000, np.uint16)
        with self.assertRaises(ValueError):
            estimate(d, [0, 33, 33, 99, 133], .001, "mean")
        starts = list(cell_starts(120))
        self.assertEqual(starts, [30, 48, 66, 84, 102])
        sets = [set(range(i, i + 18)) for i in starts]
        self.assertEqual(sum(map(len, sets)), len(set.union(*sets)))

    def test_source_gap_reject(self):
        rows = [{"序号": i, "源信息": {**{f"{s}源帧号": str(i + 1) for s in ("彩图", "深度")},
                **{f"{s}时间域": "synthetic" for s in ("彩图", "深度")},
                **{f"{s}时间戳毫秒": i * 1000 / 30 for s in ("彩图", "深度")}}} for i in range(18)]
        self.assertTrue(continuous(rows, set()))
        self.assertFalse(continuous(rows, {5}))
        altered = copy.deepcopy(rows)
        altered[4]["源信息"]["深度源帧号"] = "9"
        self.assertFalse(continuous(altered, set()))


def integration(exe, root):
    color = np.full((24, 32, 3), 100, np.uint8)
    depth = np.full((24, 32), 1200, np.uint16)
    raw = fixture(root, "temporal", color, depth, frames=48)
    probe = root / "raw"
    subprocess.run([str(exe.resolve()), "--probe-startup", "--sessions=1", "--frames=48",
                    f"--replay-manifest={raw.resolve()}", f"--output-root={probe.resolve()}"], check=True, timeout=30)
    path = probe / "probe.json"
    before = hashlib.sha256(path.read_bytes()).hexdigest()
    output = root / "analysis"
    report = evaluate(path, output)
    assert report["status"] == "measured" and report["production_promotion"] is False
    assert report["sessions"][0]["accepted_cells"] == 1
    for row in report["aggregate"]:
        if row["paired_cells"]:
            assert row["estimate_delta_p95_m_median"] == 0
    for sample in report["samples"]:
        with np.load(output / sample["file"], allow_pickle=False) as arrays:
            np.testing.assert_array_equal(arrays["raw_current"], depth)
            np.testing.assert_allclose(arrays["estimate_m"], 1.2)
            assert arrays["support_count"].min() == sample["window"]
    assert hashlib.sha256(path.read_bytes()).hexdigest() == before
    (root / "test_report.json").write_text(json.dumps({"status": "pass", "unit_groups": 11,
        "integration": "raw input, disjoint block evaluation and estimate readback",
        "known_limit": "gradual depth motion can pass guard and lag current surface; no promotion"}, indent=2), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--exe", type=Path, default=Path(__file__).resolve().parent.parent / "x64/Release/PixelClusterSensor.exe")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(EstimatorTests)
    if not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful():
        raise SystemExit(1)
    integration(args.exe, args.output)
    print("PASS temporal estimator integration")
