"""Legacy input limits and dynamic diagnostic checks; generated fixtures are not real recordings."""
import argparse
import csv
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image

from evaluate_dynamic_accumulation import evaluate, read_case


def make_case(root, count=42, timestamps=False):
    frames = root / "frames"
    frames.mkdir(parents=True)
    manifest = {"format": "d455_directory_replay_v1", "depth_unit": "millimeter_uint16",
                "case_id": "synthetic_dynamic", "frame_count": count,
                "depth_resolution": [8, 8], "color_resolution": [8, 8]}
    (root / "case_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    for i in range(count):
        depth = np.full((8, 8), 1000, np.uint16)
        color = np.full((8, 8, 3), 100, np.uint8)
        if i >= 35:
            color[3, 3] = 150
            depth[3, 3] = 0 if i < 37 else 2000
        Image.fromarray(depth).save(frames / f"{i:06}_depth16.png")
        Image.fromarray(color).save(frames / f"{i:06}_color.png")
    if timestamps:
        with (root / "source_timestamps.csv").open("w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["sample_index", "source_frameset_index", "color_frame_number", "depth_frame_number", "color_timestamp_ms", "depth_timestamp_ms"])
            writer.writerows([i, i + 10, i + 20, i + 30, i * 33 + 5, i * 33] for i in range(count))


class DynamicTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_nominal_timeline_must_be_explicit(self):
        case = self.root / "case"
        make_case(case)
        with self.assertRaises(ValueError):
            read_case(case)
        _, _, _, info = read_case(case, 30)
        self.assertEqual(info["timing"]["mode"], "assumed_nominal_timeline")
        self.assertFalse(info["timing"]["physical_age_verified"])

    def test_missing_png_rejected(self):
        case = self.root / "case"
        make_case(case)
        (case / "frames/000005_color.png").unlink()
        with self.assertRaises(ValueError):
            read_case(case, 30)

    def test_recorded_times_not_overridden_by_nominal(self):
        case = self.root / "case"
        make_case(case, timestamps=True)
        _, times, continuity, info = read_case(case, 60)
        self.assertEqual(times[1], 33)
        self.assertTrue(all(continuity[1:]))
        self.assertFalse(info["timing"]["timestamp_domains_recorded"])

    def test_duplicate_or_nan_timestamps_rejected(self):
        case = self.root / "case"
        make_case(case, timestamps=True)
        path = case / "source_timestamps.csv"
        data = path.read_text()
        path.write_text(data.replace("1,11,21,31,38,33", "0,11,21,31,38,33"))
        with self.assertRaises(ValueError):
            read_case(case)
        path.write_text(data.replace("1,11,21,31,38,33", "1,11,21,31,nan,33"))
        with self.assertRaises(ValueError):
            read_case(case)

    def test_source_gap_preserved_as_reset(self):
        case = self.root / "case"
        make_case(case, timestamps=True)
        path = case / "source_timestamps.csv"
        path.write_text(path.read_text().replace("1,11,21,31,38,33", "1,11,25,31,38,33"))
        _, _, continuity, info = read_case(case)
        self.assertFalse(continuity[1])
        self.assertEqual(info["source_gap_count"], 2)

    def test_dynamic_risk_and_deterministic_output(self):
        case = self.root / "case"
        make_case(case, timestamps=True)
        result = evaluate(case, self.root / "out", repeats=2)
        self.assertEqual(result["status"], "measured_with_source_limits")
        self.assertEqual(result["source_value_age_checks"], "pass_all_frames")
        self.assertTrue(result["repeat_deterministic"])
        self.assertEqual(result["after_30"]["totals"]["local_color_conflict_pixels"], 2)
        self.assertEqual(result["after_30"]["totals"]["reobserved_conflict_pixels"], 1)
        self.assertFalse(result["promotion"])
        with self.assertRaises(FileExistsError):
            evaluate(case, self.root / "out", repeats=1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(DynamicTests))
    (args.output / "test_report.json").write_text(json.dumps({"status": "pass" if result.wasSuccessful() else "fail",
        "tests": result.testsRun, "scope": "synthetic legacy adapter and dynamic risk diagnostics, not motion safety"}, indent=2), encoding="utf-8")
    raise SystemExit(0 if result.wasSuccessful() else 1)
