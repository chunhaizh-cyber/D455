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


def clamp_box(box, width, height):
    return (
        max(0, min(width, int(box[0]))),
        max(0, min(height, int(box[1]))),
        max(0, min(width, int(box[2]))),
        max(0, min(height, int(box[3]))),
    )


def shifted_box(box, shift_x, shift_y, width, height):
    return clamp_box(
        (box[0] + shift_x, box[1] + shift_y, box[2] + shift_x, box[3] + shift_y),
        width,
        height,
    )


def target_box(width, height, box_width, box_height, offset_x, offset_y):
    x = (width - box_width) // 2 + offset_x
    y = (height - box_height) // 2 + offset_y
    return clamp_box((x, y, x + box_width, y + box_height), width, height)


def occluder_box(index, frame_count, target, width, height, width_px, height_px):
    enter_end = max(1, frame_count // 3)
    hold_end = max(enter_end + 1, (frame_count * 2) // 3)
    leave_end = max(hold_end + 1, frame_count - 1)
    target_cx = (target[0] + target[2]) // 2
    target_cy = (target[1] + target[3]) // 2
    start_x = target[0] - width_px - 32
    center_x = target_cx - width_px // 2
    end_x = target[2] + 32
    if index < enter_end:
        t = index / float(enter_end)
        x = round(start_x + (center_x - start_x) * t)
    elif index < hold_end:
        x = center_x
    else:
        t = (index - hold_end) / float(max(1, leave_end - hold_end))
        x = round(center_x + (end_x - center_x) * t)
    y = target_cy - height_px // 2
    return clamp_box((x, y, x + width_px, y + height_px), width, height)


def draw_stereo_patch(ir, box, fill, inner):
    draw = ImageDraw.Draw(ir)
    draw.rectangle(box, fill=fill)
    inset = (box[0] + 8, box[1] + 8, box[2] - 8, box[3] - 8)
    if inset[2] > inset[0] and inset[3] > inset[1]:
        draw.rectangle(inset, fill=inner)


def draw_target(color, depth, ir_left, ir_right, box, depth_mm, right_ir_shift_x, right_ir_shift_y):
    width, height = color.size
    color_draw = ImageDraw.Draw(color)
    color_draw.rectangle(box, fill=(245, 40, 35))
    inset = (box[0] + 10, box[1] + 10, box[2] - 10, box[3] - 10)
    if inset[2] > inset[0] and inset[3] > inset[1]:
        color_draw.rectangle(inset, outline=(255, 245, 40), width=5)
    ImageDraw.Draw(depth).rectangle(box, fill=int(depth_mm))
    draw_stereo_patch(ir_left, box, 230, 80)
    draw_stereo_patch(ir_right, shifted_box(box, right_ir_shift_x, right_ir_shift_y, width, height), 230, 80)


def draw_occluder(color, depth, ir_left, ir_right, box, depth_mm, right_ir_shift_x, right_ir_shift_y):
    width, height = color.size
    color_draw = ImageDraw.Draw(color)
    color_draw.rectangle(box, fill=(198, 142, 111))
    for offset in range(12, max(12, box[2] - box[0] - 8), 18):
        x = box[0] + offset
        color_draw.line((x, box[1] + 6, x - 8, box[3] - 6), fill=(166, 106, 84), width=2)
    ImageDraw.Draw(depth).rectangle(box, fill=int(depth_mm))
    draw_stereo_patch(ir_left, box, 188, 112)
    draw_stereo_patch(ir_right, shifted_box(box, right_ir_shift_x, right_ir_shift_y, width, height), 188, 112)


def save_image(path, image):
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-case", default="datasets/near_single_object")
    parser.add_argument("--out-case", default="analysis_runs/generated_cases/hand_occlusion_reappear_proxy")
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--source-frame", default="000000")
    parser.add_argument("--target-width", type=int, default=120)
    parser.add_argument("--target-height", type=int, default=100)
    parser.add_argument("--target-offset-x", type=int, default=0)
    parser.add_argument("--target-offset-y", type=int, default=0)
    parser.add_argument("--target-depth-mm", type=int, default=1200)
    parser.add_argument("--target-right-ir-shift-x", type=int, default=-30)
    parser.add_argument("--occluder-width", type=int, default=150)
    parser.add_argument("--occluder-height", type=int, default=145)
    parser.add_argument("--occluder-depth-mm", type=int, default=850)
    parser.add_argument("--occluder-right-ir-shift-x", type=int, default=-42)
    parser.add_argument("--case-yaml", default="")
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
    target = target_box(
        width,
        height,
        args.target_width,
        args.target_height,
        args.target_offset_x,
        args.target_offset_y,
    )

    out_frames = out_case / "frames"
    out_frames.mkdir(parents=True, exist_ok=True)
    for index in range(args.frames):
        color = base_color.copy()
        depth = base_depth.copy()
        ir_left = base_ir_left.copy()
        ir_right = base_ir_right.copy()
        draw_target(
            color,
            depth,
            ir_left,
            ir_right,
            target,
            args.target_depth_mm,
            args.target_right_ir_shift_x,
            0,
        )
        occ = occluder_box(index, args.frames, target, width, height, args.occluder_width, args.occluder_height)
        draw_occluder(
            color,
            depth,
            ir_left,
            ir_right,
            occ,
            args.occluder_depth_mm,
            args.occluder_right_ir_shift_x,
            0,
        )
        stem = f"{index:06d}"
        save_image(out_frames / f"{stem}_color.png", color)
        save_image(out_frames / f"{stem}_depth16.png", depth)
        save_image(out_frames / f"{stem}_ir_left.png", ir_left)
        save_image(out_frames / f"{stem}_ir_right.png", ir_right)

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
        "generator": "scripts/generate_hand_occlusion_reappear_proxy.py",
        "source_case": str(source_case),
        "source_frame": frame_stem,
        "notes": (
            "Synthetic hand-occlusion/reappear proxy generated from one reviewed D455 replay frame. "
            "It exercises G3 drop-review plumbing and target reappearance, but does not replace a real "
            "hand_occlusion_reappear camera capture."
        ),
        "target": {
            "bbox": list(target),
            "depth_mm": args.target_depth_mm,
            "right_ir_shift_x": args.target_right_ir_shift_x,
        },
        "occluder": {
            "width": args.occluder_width,
            "height": args.occluder_height,
            "depth_mm": args.occluder_depth_mm,
            "right_ir_shift_x": args.occluder_right_ir_shift_x,
            "motion": "left_enter_hold_right_leave",
        },
        "expected": {
            "motion_refresh_required": True,
            "roi_refresh_required": True,
            "contour_recovery_required": True,
            "roi_drop_visual_review_required": True,
            "roi_stereo_g1_preservation_required": True,
            "roi_stereo_g2_rebuild_required": True,
            "roi_stereo_failed_count_max": 0,
            "roi_stereo_reuse_or_built_min": 1,
        },
        "tags": [
            "motion",
            "occlusion",
            "reappear",
            "roi_refresh",
            "roi_drop_review",
            "synthetic_proxy",
        ],
        "case_contract": {
            "source": "generated_proxy",
            "frames": args.frames,
            "weight": 1.0,
        },
    }
    (out_case / "case_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    case_yaml = Path(args.case_yaml) if args.case_yaml else out_case.with_name(f"{out_case.name}_cases.yaml")
    case_yaml.write_text(
        "\n".join([
            "splits:",
            "  validation:",
            f"    - {out_case.name}",
            "cases:",
            f"  - case_id: {out_case.name}",
            f"    replay: {str(out_case).replace(chr(92), '/')}",
            f"    frames: {args.frames}",
            "    tags: [motion, occlusion, reappear, roi_drop_review, synthetic_proxy]",
            "    weight: 1.0",
            "    expected:",
            "      contour_recovery_required: true",
            "      roi_drop_visual_review_required: true",
            "      roi_stereo_g2_rebuild_required: true",
            "",
        ]),
        encoding="utf-8",
    )
    print(json.dumps({
        "case_dir": str(out_case),
        "case_yaml": str(case_yaml),
        "frames": args.frames,
        "source_frame": frame_stem,
    }, indent=2))


if __name__ == "__main__":
    main()
