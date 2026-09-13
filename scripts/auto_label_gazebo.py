#!/usr/bin/env python3
"""
Auto-label Gazebo outdoor_path (town) images using color rules.

Flat materials (no shadows) → reliable HSV thresholds.

Class IDs:
  0 = background (sky / houses / buildings / unknown)
  1 = road (asphalt + yellow center line)
  2 = grass
  3 = tree (canopy + dark trunks)

Usage:
  python3 scripts/auto_label_gazebo.py \\
    --images_dir ~/ugv_vision_nav_ws/data/images_town_YYYYMMDD_HHMMSS \\
    --out_dir ~/ugv_vision_nav_ws/data/dataset_autolabel
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np


CLASS_BG = 0
CLASS_ROAD = 1
CLASS_GRASS = 2
CLASS_TREE = 3

PREVIEW = {
    CLASS_BG: (180, 180, 180),
    CLASS_ROAD: (50, 50, 50),
    CLASS_GRASS: (0, 200, 0),
    CLASS_TREE: (0, 100, 0),
}


def classify_hsv(hsv: np.ndarray) -> np.ndarray:
    """Return HxW uint8 mask with class IDs."""
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    mask = np.zeros(h.shape, dtype=np.uint8)

    # Asphalt: near-neutral gray (buildings have a blue cast → higher S / hue ~120)
    asphalt = (s < 18) & (v > 110) & (v < 185)
    # Yellow center line → treat as road (traversable)
    yellow = (h >= 22) & (h <= 38) & (s > 55) & (v > 180)
    # Bright grass verge
    grass = (h >= 48) & (h <= 75) & (s > 55) & (v > 145)
    # Darker canopy greens
    canopy = (h >= 48) & (h <= 85) & (s > 70) & (v <= 145)
    # Trunks only (darker brown); house walls are lighter tan (V ~130+) → bg
    trunk = (h >= 8) & (h <= 22) & (s > 45) & (v > 35) & (v < 115)

    mask[asphalt | yellow] = CLASS_ROAD
    mask[grass] = CLASS_GRASS
    mask[canopy | trunk] = CLASS_TREE
    return mask


def colorize(mask: np.ndarray) -> np.ndarray:
    out = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
    for cid, bgr in PREVIEW.items():
        out[mask == cid] = bgr
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images_dir', type=Path, required=True)
    parser.add_argument('--out_dir', type=Path, default=None)
    parser.add_argument('--preview_every', type=int, default=15)
    parser.add_argument(
        '--clean',
        action='store_true',
        help='Wipe out_dir images/masks/previews before writing',
    )
    args = parser.parse_args()

    images_dir = args.images_dir.expanduser().resolve()
    if args.out_dir is None:
        out_dir = images_dir.parent / 'dataset_autolabel'
    else:
        out_dir = args.out_dir.expanduser().resolve()

    img_out = out_dir / 'images'
    mask_out = out_dir / 'masks'
    preview_out = out_dir / 'previews'
    if args.clean and out_dir.exists():
        for sub in (img_out, mask_out, preview_out):
            if sub.exists():
                shutil.rmtree(sub)
    img_out.mkdir(parents=True, exist_ok=True)
    mask_out.mkdir(parents=True, exist_ok=True)
    preview_out.mkdir(parents=True, exist_ok=True)

    paths = sorted(images_dir.glob('frame_*.png'))
    if not paths:
        paths = sorted(images_dir.glob('*.png'))
    if not paths:
        raise SystemExit(f'No images found in {images_dir}')

    print(f'Auto-labeling {len(paths)} images from {images_dir}')
    print(f'Writing dataset to {out_dir}')
    print('Classes: 0=bg  1=road  2=grass  3=tree')

    counts = np.zeros(4, dtype=np.int64)
    for i, path in enumerate(paths):
        bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if bgr is None:
            print(f'  skip unreadable: {path.name}')
            continue
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = classify_hsv(hsv)

        stem = path.stem
        cv2.imwrite(str(img_out / f'{stem}.png'), bgr)
        cv2.imwrite(str(mask_out / f'{stem}.png'), mask)

        for c in range(4):
            counts[c] += int((mask == c).sum())

        if args.preview_every > 0 and i % args.preview_every == 0:
            overlay = cv2.addWeighted(bgr, 0.45, colorize(mask), 0.55, 0)
            cv2.imwrite(str(preview_out / f'{stem}_preview.png'), overlay)

        if (i + 1) % 50 == 0 or i == 0:
            print(f'  {i + 1}/{len(paths)}')

    total = max(int(counts.sum()), 1)
    print('Done.')
    print(
        f'Pixel share: bg={counts[0]/total:.2%} road={counts[1]/total:.2%} '
        f'grass={counts[2]/total:.2%} tree={counts[3]/total:.2%}'
    )
    print(f'Images:  {img_out} ({len(list(img_out.glob("*.png")))} files)')
    print(f'Masks:   {mask_out} ({len(list(mask_out.glob("*.png")))} files)')
    print(f'Preview: {preview_out}')
    (out_dir / 'classes.txt').write_text('background\nroad\ngrass\ntree\n')


if __name__ == '__main__':
    main()
