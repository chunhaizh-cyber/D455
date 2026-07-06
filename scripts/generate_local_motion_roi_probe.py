#!/usr/bin/env python3
import argparse
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path

from PIL import Image, ImageDraw


def frame_stems(frames_dir, suffix):
    return sorted(
        path.name[: -(len(suffix) + 5)]
        for path in frames_dir.glob(f"*_{suffix}.png")
    )


def load_frame_paths(source_case, frame_stem):
    frames_dir = source_case / "frames"
    return {
        "color": frames_dir / f"{frame_stem}_color.png",
        "depth16": frames_dir / f"{frame_stem}_depth16.png",
        "ir_left": frames_dir / f"{frame_stem}_ir_left.png",
        "ir_right": frames_dir / f"{frame_stem}_ir_right.png",
    }


def local_motion_box(frame_index, width, height, box_width, box_height, travel_x, travel_y, period):
    base_x = max(20, (width - box_width - max(1, travel_x)) // 2)
    base_y = max(20, (height - box_height - max(1, travel_y)) // 2)
    phase = (frame_index % max(2, period)) / float(max(1, period - 1))
    if phase > 0.5:
        phase = 1.0 - phase
    phase *= 2.0
    x = base_x + int(round(phase * max(0, travel_x)))
    y = base_y + int(round(0.25 * max(0, travel_y)))
    return (x, y, x + box_width, y + box_height)


def draw_probe_object(color, depth, ir_left, ir_right, box, object_depth_mm):
    color_draw = ImageDraw.Draw(color)
    color_draw.rectangle(box, fill=(245, 40, 35))
    inset = (box[0] + 8, box[1] + 8, box[2] - 8, box[3] - 8)
    color_draw.rectangle(inset, outline=(255, 245, 40), width=4)

    depth_draw = ImageDraw.Draw(depth)
    depth_draw.rectangle(box, fill=int(object_depth_mm))

    for ir in [ir_left, ir_right]:
        ir_draw = ImageDraw.Draw(ir)
        ir_draw.rectangle(box, fill=230)
        ir_draw.rectangle(inset, fill=80)


def copy_or_save(path, image):
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-case", default="datasets/near_single_object")
    parser.add_argument("--out-case", default="datasets/local_motion_roi_probe")
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--source-frame", default="000000")
    parser.add_argument("--box-width", type=int, default=120)
    parser.add_argument("--box-height", type=int, default=100)
    parser.add_argument("--travel-x", type=int, default=80)
    parser.add_argument("--travel-y", type=int, default=0)
    parser.add_argument("--period", type=int, default=60)
    parser.add_argument("--object-depth-mm", type=int, default=1200)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    source_case = Path(args.source_case)
    out_case = Path(args.out_case)
    source_frames = source_case / "frames"
    if not source_frames.exists():
        raise SystemExit(f"missing source frames directory: {source_frames}")

    available = set(frame_stems(source_frames, "color"))
    frame_stem = args.source_frame
    if frame_stem not in available:
        if not available:
            raise SystemExit(f"no source color frames under {source_frames}")
        frame_stem = sorted(available)[0]

    if out_case.exists() and args.force:
        shutil.rmtree(out_case)
    if out_case.exists() and any(out_case.iterdir()):
        raise SystemExit(f"output case already exists; pass --force to replace: {out_case}")

    paths = load_frame_paths(source_case, frame_stem)
    for label, path in paths.items():
        if not path.exists():
            raise SystemExit(f"source {label} frame missing: {path}")

    base_color = Image.open(paths["color"]).convert("RGB")
    base_depth = Image.open(paths["depth16"])
    base_ir_left = Image.open(paths["ir_left"]).convert("L")
    base_ir_right = Image.open(paths["ir_right"]).convert("L")
    width, height = base_color.size

    out_frames = out_case / "frames"
    out_frames.mkdir(parents=True, exist_ok=True)
    for index in range(args.frames):
        color = base_color.copy()
        depth = base_depth.copy()
        ir_left = base_ir_left.copy()
        ir_right = base_ir_right.copy()
        box = local_motion_box(
            index,
            width,
            height,
            args.box_width,
            args.box_height,
            args.travel_x,
            args.travel_y,
            args.period,
        )
        draw_probe_object(color, depth, ir_left, ir_right, box, args.object_depth_mm)
        stem = f"{index:06d}"
        copy_or_save(out_frames / f"{stem}_color.png", color)
        copy_or_save(out_frames / f"{stem}_depth16.png", depth)
        copy_or_save(out_frames / f"{stem}_ir_left.png", ir_left)
        copy_or_save(out_frames / f"{stem}_ir_right.png", ir_right)

    manifest = {
        "case_id": out_case.name,
        "frame_count": args.frames,
        "format": "d455_directory_replay_v1",
        "depth_unit": "millimeter_uint16",
        "color_resolution": [width, height],
        "depth_resolution": [width, height],
        "ir_left_present": True,
        "ir_right_present": True,
        "reviewed": True,
        "review_method": "auto_case_contract_v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "generator": "scripts/generate_local_motion_roi_probe.py",
        "source_case": str(source_case),
        "source_frame": frame_stem,
        "motion_box": {
            "width": args.box_width,
            "height": args.box_height,
            "travel_x": args.travel_x,
            "travel_y": args.travel_y,
            "period": args.period,
        },
        "notes": (
            "Synthetic local-motion ROI probe generated from one reviewed D455 replay frame. "
            "It is intended to validate ROI refresh plumbing, not real-scene segmentation quality."
        ),
        "expected": {
            "motion_refresh_required": True,
            "roi_refresh_required": True,
            "full_frame_roi_rejected_large_max": 0,
        },
        "tags": ["motion", "local_motion", "roi_refresh", "synthetic_probe"],
        "case_contract": {
            "source": "eval/cases.yaml",
            "replay": str(out_case).replace("/", "\\"),
            "frames": args.frames,
            "weight": 1.0,
        },
    }
    (out_case / "case_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"case_dir": str(out_case), "frames": args.frames, "source_frame": frame_stem}, indent=2))


if __name__ == "__main__":
    main()
