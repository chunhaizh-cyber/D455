#!/usr/bin/env python3
import argparse
import json
import struct
import sys
from pathlib import Path


PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def read_png_header(path):
    with Path(path).open("rb") as f:
        signature = f.read(8)
        if signature != PNG_SIGNATURE:
            raise ValueError("not a PNG file")
        length_data = f.read(4)
        chunk_type = f.read(4)
        if len(length_data) != 4 or chunk_type != b"IHDR":
            raise ValueError("missing PNG IHDR")
        length = struct.unpack(">I", length_data)[0]
        if length < 13:
            raise ValueError("invalid PNG IHDR length")
        data = f.read(length)
    width, height, bit_depth, color_type = struct.unpack(">IIBB", data[:10])
    return {
        "width": width,
        "height": height,
        "bit_depth": bit_depth,
        "color_type": color_type,
    }


def frame_stems(frames_dir, suffix):
    stems = {}
    for path in sorted(frames_dir.glob(f"*_{suffix}.png")):
        stem = path.name[: -(len(suffix) + 5)]
        stems[stem] = path
    return stems


def load_manifest(case_dir):
    manifest_path = case_dir / "case_manifest.json"
    if not manifest_path.exists():
        return None, f"missing {manifest_path}"
    try:
        return json.loads(manifest_path.read_text(encoding="utf-8")), None
    except json.JSONDecodeError as exc:
        return None, f"invalid JSON in {manifest_path}: {exc}"


PLACEHOLDER_NOTES = "Fill expected scene notes before using this case for regression."


def validate_case(case_dir, min_frames, require_ir_left, require_ir_right, require_reviewed_manifest):
    errors = []
    warnings = []
    case_dir = Path(case_dir)
    frames_dir = case_dir / "frames"
    if not case_dir.exists():
        return {"case_dir": str(case_dir), "pass": False, "errors": [f"missing case directory: {case_dir}"], "warnings": warnings}
    if not frames_dir.exists():
        return {"case_dir": str(case_dir), "pass": False, "errors": [f"missing frames directory: {frames_dir}"], "warnings": warnings}

    manifest, manifest_error = load_manifest(case_dir)
    if manifest_error:
        errors.append(manifest_error)

    color = frame_stems(frames_dir, "color")
    depth = frame_stems(frames_dir, "depth16")
    ir_left = frame_stems(frames_dir, "ir_left")
    ir_right = frame_stems(frames_dir, "ir_right")
    common_stems = sorted(set(color) & set(depth))

    if len(common_stems) < min_frames:
        errors.append(f"only {len(common_stems)} color/depth frame pairs, expected at least {min_frames}")
    missing_depth = sorted(set(color) - set(depth))
    missing_color = sorted(set(depth) - set(color))
    if missing_depth:
        errors.append(f"{len(missing_depth)} color frames have no depth16 pair; first={missing_depth[0]}")
    if missing_color:
        errors.append(f"{len(missing_color)} depth16 frames have no color pair; first={missing_color[0]}")

    if require_ir_left:
        missing_left = sorted(set(common_stems) - set(ir_left))
        if missing_left:
            errors.append(f"{len(missing_left)} frame pairs have no ir_left pair; first={missing_left[0]}")
    if require_ir_right:
        missing_right = sorted(set(common_stems) - set(ir_right))
        if missing_right:
            errors.append(f"{len(missing_right)} frame pairs have no ir_right pair; first={missing_right[0]}")

    expected_color_header = None
    expected_depth_header = None
    for stem in common_stems:
        try:
            color_header = read_png_header(color[stem])
            depth_header = read_png_header(depth[stem])
        except ValueError as exc:
            errors.append(f"bad PNG at frame {stem}: {exc}")
            continue

        if expected_color_header is None:
            expected_color_header = color_header
            expected_depth_header = depth_header
            if color_header["bit_depth"] != 8 or color_header["color_type"] != 2:
                errors.append(f"color PNG should be 8-bit truecolor, got {color_header}")
            if depth_header["bit_depth"] != 16 or depth_header["color_type"] != 0:
                errors.append(f"depth16 PNG should be 16-bit grayscale, got {depth_header}")
        elif color_header != expected_color_header:
            errors.append(f"color PNG header changes at frame {stem}: {color_header} != {expected_color_header}")
        elif depth_header != expected_depth_header:
            errors.append(f"depth16 PNG header changes at frame {stem}: {depth_header} != {expected_depth_header}")

        if color_header["width"] != depth_header["width"] or color_header["height"] != depth_header["height"]:
            errors.append(f"color/depth resolution mismatch at frame {stem}: {color_header} vs {depth_header}")

    for label, paths in [("ir_left", ir_left), ("ir_right", ir_right)]:
        for stem in sorted(set(common_stems) & set(paths)):
            try:
                header = read_png_header(paths[stem])
            except ValueError as exc:
                errors.append(f"bad {label} PNG at frame {stem}: {exc}")
                continue
            if header["bit_depth"] != 8 or header["color_type"] != 0:
                errors.append(f"{label} PNG should be 8-bit grayscale at frame {stem}, got {header}")
            if expected_color_header and (
                header["width"] != expected_color_header["width"] or
                header["height"] != expected_color_header["height"]
            ):
                errors.append(f"{label} resolution mismatch at frame {stem}: {header} vs {expected_color_header}")

    if manifest:
        manifest_count = manifest.get("frame_count")
        if manifest_count is not None and manifest_count != len(common_stems):
            warnings.append(f"manifest frame_count={manifest_count}, actual color/depth pairs={len(common_stems)}")
        if manifest.get("depth_unit") != "millimeter_uint16":
            errors.append(f"manifest depth_unit should be millimeter_uint16, got {manifest.get('depth_unit')!r}")
        if manifest.get("format") != "d455_directory_replay_v1":
            warnings.append(f"manifest format is {manifest.get('format')!r}, expected d455_directory_replay_v1")
        if require_reviewed_manifest:
            if manifest.get("reviewed") is not True:
                errors.append("manifest must contain reviewed=true before scoring")
            notes = str(manifest.get("notes") or "").strip()
            if not notes or notes == PLACEHOLDER_NOTES:
                errors.append("manifest notes must describe the reviewed scene before scoring")
            expected = manifest.get("expected")
            if not isinstance(expected, dict) or not expected:
                errors.append("manifest expected must be a non-empty object before scoring")

    return {
        "case_dir": str(case_dir),
        "pass": not errors,
        "frame_pairs": len(common_stems),
        "color_frames": len(color),
        "depth_frames": len(depth),
        "ir_left_frames": len(ir_left),
        "ir_right_frames": len(ir_right),
        "color_header": expected_color_header,
        "depth_header": expected_depth_header,
        "errors": errors,
        "warnings": warnings,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-dir", action="append", required=True)
    parser.add_argument("--min-frames", type=int, default=1)
    parser.add_argument("--require-ir-left", action="store_true")
    parser.add_argument("--require-ir-right", action="store_true")
    parser.add_argument("--require-reviewed-manifest", action="store_true")
    parser.add_argument("--report")
    args = parser.parse_args()

    reports = [
        validate_case(
            path,
            args.min_frames,
            args.require_ir_left,
            args.require_ir_right,
            args.require_reviewed_manifest,
        )
        for path in args.case_dir
    ]
    output = {"pass": all(report["pass"] for report in reports), "cases": reports}
    text = json.dumps(output, ensure_ascii=False, indent=2)
    if args.report:
        report_path = Path(args.report)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if output["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
