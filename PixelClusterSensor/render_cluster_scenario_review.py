"""Render bounded visual review evidence for T3 local motion or T7 dark depth-hole captures."""

from __future__ import annotations

import argparse
import hashlib
import html
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from evaluate_cluster_scenario_capture import SCENARIOS, _read_frames


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sample_indices(frame_count: int, maximum: int = 6) -> list[int]:
    if frame_count <= maximum:
        return list(range(frame_count))
    return sorted({round(index * (frame_count - 1) / (maximum - 1)) for index in range(maximum)})


def overlay_mask(color: np.ndarray, mask: np.ndarray, rgb: tuple[int, int, int]) -> Image.Image:
    result = color.astype(np.float32).copy()
    tint = np.asarray(rgb, dtype=np.float32)
    result[mask] = result[mask] * 0.35 + tint * 0.65
    return Image.fromarray(np.clip(result, 0, 255).astype(np.uint8), "RGB")


def render_review(sequence_path: Path, scenario: str, output: Path) -> dict:
    if scenario not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {scenario}")
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    output.mkdir(parents=True)
    _, frames = _read_frames(sequence_path)
    indices = sample_indices(len(frames))
    if scenario == "T3_local_motion" and indices and indices[0] == 0:
        indices[0] = 1
        indices = sorted(set(indices))
    samples = []
    for index in indices:
        color, depth = frames[index]
        source_path = output / f"frame_{index + 1:06d}_source.png"
        Image.fromarray(color, "RGB").save(source_path)
        if scenario == "T3_local_motion":
            previous = frames[index - 1][0]
            delta = np.max(np.abs(color.astype(np.int16) - previous.astype(np.int16)), axis=2)
            mask = delta >= 30
            overlay = overlay_mask(color, mask, (255, 0, 0))
            if np.any(mask):
                ys, xs = np.where(mask)
                ImageDraw.Draw(overlay).rectangle((int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())),
                                                  outline=(0, 255, 0), width=2)
            mask_kind = "red=motion, green=bbox"
        else:
            luminance = (color[:, :, 0].astype(np.uint16) * 54 + color[:, :, 1].astype(np.uint16) * 183 +
                         color[:, :, 2].astype(np.uint16) * 19) // 256
            dark = luminance <= 60
            holes = depth == 0
            overlay = overlay_mask(color, dark, (255, 215, 0))
            combined = np.asarray(overlay).copy()
            combined[dark & holes] = [255, 0, 0]
            overlay = Image.fromarray(combined, "RGB")
            mask_kind = "yellow=dark, red=dark-and-depth-hole"
        overlay_path = output / f"frame_{index + 1:06d}_overlay.png"
        overlay.save(overlay_path)
        samples.append({"frame_index": index + 1, "source": source_path.name, "overlay": overlay_path.name,
                        "source_sha256": sha256(source_path), "overlay_sha256": sha256(overlay_path),
                        "legend": mask_kind})

    rows = "\n".join(
        f'<section><h2>Frame {item["frame_index"]}</h2><p>{html.escape(item["legend"])}</p>'
        f'<img src="{html.escape(item["source"])}" alt="source frame">'
        f'<img src="{html.escape(item["overlay"])}" alt="evidence overlay"></section>'
        for item in samples)
    document = ("<!doctype html><meta charset=\"utf-8\"><title>Cluster scenario review</title>"
                "<style>body{font-family:Segoe UI,sans-serif;margin:24px;background:#f4f5f7;color:#202124}"
                "section{margin:0 0 24px}img{max-width:48%;height:auto;margin-right:12px;border:1px solid #888}"
                "h1,h2{letter-spacing:0}</style>"
                f"<h1>{html.escape(scenario)} visual review</h1>"
                "<p>These overlays are evidence aids, not automatic object identity or scene truth.</p>" + rows)
    (output / "review.html").write_text(document, encoding="utf-8")
    manifest = {"格式": "PCS.ClusterScenarioVisualReview/1", "场景": scenario,
                "输入清单": str(sequence_path.resolve()), "样本": samples,
                "人工复核状态": "pending",
                "复核问题": (["红色变化是否来自局部目标而非相机整体移动？", "绿色框是否包住目标变化区域？"]
                             if scenario == "T3_local_motion" else
                             ["红色孔洞是否位于目标暗色表面？", "红色区域是否只是画面外黑边或无关背景？"])}
    (output / "review_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
                                                   encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", type=Path, required=True)
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(render_review(args.sequence, args.scenario, args.output), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
