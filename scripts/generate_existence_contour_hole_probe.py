#!/usr/bin/env python3
import argparse
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path

from PIL import Image, ImageDraw


def clamp_box(box, width, height):
    left, top, right, bottom = box
    return (
        max(0, min(width - 1, left)),
        max(0, min(height - 1, top)),
        max(0, min(width - 1, right)),
        max(0, min(height - 1, bottom)),
    )


def draw_background(color, depth, ir_left, ir_right, background_depth_mm):
    color_draw = ImageDraw.Draw(color)
    left_draw = ImageDraw.Draw(ir_left)
    right_draw = ImageDraw.Draw(ir_right)
    for y in range(0, color.height, 32):
        shade = 72 if (y // 32) % 2 == 0 else 82
        color_draw.rectangle((0, y, color.width - 1, min(color.height - 1, y + 31)), fill=(shade, shade + 8, shade + 14))
        left_draw.rectangle((0, y, color.width - 1, min(color.height - 1, y + 31)), fill=shade)
        right_draw.rectangle((0, y, color.width - 1, min(color.height - 1, y + 31)), fill=shade)
    ImageDraw.Draw(depth).rectangle((0, 0, depth.width - 1, depth.height - 1), fill=background_depth_mm)


def draw_probe(color, depth, ir_left, ir_right, outer_box, hole_box, mode, foreground_depth_mm, background_depth_mm):
    hole_center = ((hole_box[0] + hole_box[2]) // 2, (hole_box[1] + hole_box[3]) // 2)
    background_color = color.getpixel(hole_center)
    background_ir = ir_left.getpixel(hole_center)
    color_draw = ImageDraw.Draw(color)
    depth_draw = ImageDraw.Draw(depth)
    left_draw = ImageDraw.Draw(ir_left)
    right_draw = ImageDraw.Draw(ir_right)

    color_draw.rectangle(outer_box, fill=(210, 42, 42))
    depth_draw.rectangle(outer_box, fill=foreground_depth_mm)
    left_draw.rectangle(outer_box, fill=220)
    right_draw.rectangle(outer_box, fill=220)

    if mode == "background-hole":
        color_draw.rectangle(hole_box, fill=background_color)
        depth_draw.rectangle(hole_box, fill=background_depth_mm)
        left_draw.rectangle(hole_box, fill=background_ir)
        right_draw.rectangle(hole_box, fill=background_ir)
    else:
        depth_draw.rectangle(hole_box, fill=0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-case", required=True)
    parser.add_argument("--mode", choices=["background-hole", "missing-depth"], required=True)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--foreground-depth-mm", type=int, default=900)
    parser.add_argument("--background-depth-mm", type=int, default=2500)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    if args.frames < 1 or args.width < 160 or args.height < 120:
        raise SystemExit("frames and image dimensions are too small")
    if args.background_depth_mm <= args.foreground_depth_mm:
        raise SystemExit("background depth must be greater than foreground depth")

    out_case = Path(args.out_case)
    if out_case.exists() and args.force:
        shutil.rmtree(out_case)
    if out_case.exists() and any(out_case.iterdir()):
        raise SystemExit(f"output case already exists; pass --force to replace: {out_case}")

    outer_width = min(300, args.width - 80)
    outer_height = min(260, args.height - 80)
    outer_left = (args.width - outer_width) // 2
    outer_top = (args.height - outer_height) // 2
    outer_box = clamp_box(
        (outer_left, outer_top, outer_left + outer_width, outer_top + outer_height),
        args.width,
        args.height,
    )
    hole_width = max(48, outer_width // 3)
    hole_height = max(48, outer_height // 3)
    hole_left = (args.width - hole_width) // 2
    hole_top = (args.height - hole_height) // 2
    hole_box = clamp_box(
        (hole_left, hole_top, hole_left + hole_width, hole_top + hole_height),
        args.width,
        args.height,
    )

    out_frames = out_case / "frames"
    out_frames.mkdir(parents=True, exist_ok=True)
    for index in range(args.frames):
        color = Image.new("RGB", (args.width, args.height))
        depth = Image.new("I;16", (args.width, args.height))
        ir_left = Image.new("L", (args.width, args.height))
        ir_right = Image.new("L", (args.width, args.height))
        draw_background(color, depth, ir_left, ir_right, args.background_depth_mm)
        draw_probe(
            color,
            depth,
            ir_left,
            ir_right,
            outer_box,
            hole_box,
            args.mode,
            args.foreground_depth_mm,
            args.background_depth_mm,
        )
        stem = f"{index:06d}"
        color.save(out_frames / f"{stem}_color.png")
        depth.save(out_frames / f"{stem}_depth16.png")
        ir_left.save(out_frames / f"{stem}_ir_left.png")
        ir_right.save(out_frames / f"{stem}_ir_right.png")

    manifest = {
        "case_id": out_case.name,
        "frame_count": args.frames,
        "format": "d455_directory_replay_v1",
        "depth_unit": "millimeter_uint16",
        "color_resolution": [args.width, args.height],
        "depth_resolution": [args.width, args.height],
        "ir_left_present": True,
        "ir_right_present": True,
        "reviewed": True,
        "review_method": "auto_case_contract_v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "generator": "scripts/generate_existence_contour_hole_probe.py",
        "mode": args.mode,
        "foreground_depth_mm": args.foreground_depth_mm,
        "background_depth_mm": args.background_depth_mm,
        "outer_box": list(outer_box),
        "hole_box": list(hole_box),
        "expected": {
            "confirmed_hole_required": args.mode == "background-hole",
            "retained_no_depth_required": args.mode == "missing-depth",
            "confirmed_hole_pixels_max": 0 if args.mode == "missing-depth" else None,
        },
        "notes": (
            "Synthetic mechanism probe only. background-hole exposes real background through a connected foreground ring; "
            "missing-depth keeps foreground color/IR ownership while setting only interior depth to zero."
        ),
    }
    (out_case / "case_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"case_dir": str(out_case), "mode": args.mode, "frames": args.frames}, indent=2))


if __name__ == "__main__":
    main()
