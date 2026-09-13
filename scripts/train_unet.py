#!/usr/bin/env python3
"""
Train a small UNet on auto-labeled Gazebo outdoor images.

Usage:
  python3 scripts/train_unet.py \\
    --data_dir ~/ugv_vision_nav_ws/data/dataset_autolabel \\
    --epochs 25 --batch_size 8
"""

from __future__ import annotations

import argparse
import json
import random
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset

# Allow `from unet import UNet` when run as scripts/train_unet.py
sys.path.insert(0, str(Path(__file__).resolve().parent))
from unet import UNet  # noqa: E402

NUM_CLASSES = 4
CLASS_NAMES = ['background', 'road', 'grass', 'tree']


class SegDataset(Dataset):
    def __init__(self, pairs: list[tuple[Path, Path]], size: tuple[int, int], augment: bool) -> None:
        self.pairs = pairs
        self.size = size  # (W, H)
        self.augment = augment

    def __len__(self) -> int:
        return len(self.pairs)

    def __getitem__(self, idx: int) -> tuple[torch.Tensor, torch.Tensor]:
        img_path, mask_path = self.pairs[idx]
        bgr = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        mask = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
        if bgr is None or mask is None:
            raise RuntimeError(f'Failed to read {img_path} / {mask_path}')
        if mask.ndim == 3:
            mask = mask[:, :, 0]

        bgr = cv2.resize(bgr, self.size, interpolation=cv2.INTER_LINEAR)
        mask = cv2.resize(mask, self.size, interpolation=cv2.INTER_NEAREST)

        if self.augment and random.random() < 0.5:
            bgr = cv2.flip(bgr, 1)
            mask = cv2.flip(mask, 1)

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        x = torch.from_numpy(rgb).permute(2, 0, 1).contiguous()
        y = torch.from_numpy(mask.astype(np.int64))
        return x, y


def list_pairs(data_dir: Path) -> list[tuple[Path, Path]]:
    img_dir = data_dir / 'images'
    mask_dir = data_dir / 'masks'
    pairs: list[tuple[Path, Path]] = []
    for img in sorted(img_dir.glob('*.png')):
        mask = mask_dir / img.name
        if mask.is_file():
            pairs.append((img, mask))
    if not pairs:
        raise SystemExit(f'No image/mask pairs in {data_dir}')
    return pairs


def split_pairs(
    pairs: list[tuple[Path, Path]], val_ratio: float, seed: int
) -> tuple[list[tuple[Path, Path]], list[tuple[Path, Path]]]:
    rng = random.Random(seed)
    shuffled = pairs[:]
    rng.shuffle(shuffled)
    n_val = max(1, int(len(shuffled) * val_ratio))
    return shuffled[n_val:], shuffled[:n_val]


@torch.no_grad()
def evaluate(model: nn.Module, loader: DataLoader, device: torch.device, criterion: nn.Module):
    model.eval()
    total_loss = 0.0
    inter = torch.zeros(NUM_CLASSES, device=device)
    union = torch.zeros(NUM_CLASSES, device=device)
    correct = 0
    total = 0
    for x, y in loader:
        x = x.to(device)
        y = y.to(device)
        logits = model(x)
        loss = criterion(logits, y)
        total_loss += float(loss.item()) * x.size(0)
        pred = logits.argmax(dim=1)
        correct += int((pred == y).sum().item())
        total += int(y.numel())
        for c in range(NUM_CLASSES):
            p = pred == c
            g = y == c
            inter[c] += (p & g).sum()
            union[c] += (p | g).sum()
    miou = []
    for c in range(NUM_CLASSES):
        if union[c] > 0:
            miou.append(float((inter[c] / union[c]).item()))
        else:
            miou.append(float('nan'))
    return {
        'loss': total_loss / max(total // (y.shape[-1] * y.shape[-2] * loader.batch_size or 1), 1)
        if False
        else total_loss / max(len(loader.dataset), 1),
        'acc': correct / max(total, 1),
        'iou': miou,
        'miou': float(np.nanmean(miou)),
    }


def colorize(mask: np.ndarray) -> np.ndarray:
    palette = {
        0: (180, 180, 180),
        1: (80, 80, 80),
        2: (0, 220, 0),
        3: (0, 90, 0),
    }
    out = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
    for c, bgr in palette.items():
        out[mask == c] = bgr
    return out


@torch.no_grad()
def save_val_previews(
    model: nn.Module,
    pairs: list[tuple[Path, Path]],
    out_dir: Path,
    device: torch.device,
    size: tuple[int, int],
    n: int = 8,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    model.eval()
    for i, (img_path, mask_path) in enumerate(pairs[:n]):
        bgr = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        gt = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
        if gt.ndim == 3:
            gt = gt[:, :, 0]
        bgr_r = cv2.resize(bgr, size, interpolation=cv2.INTER_LINEAR)
        gt_r = cv2.resize(gt, size, interpolation=cv2.INTER_NEAREST)
        rgb = cv2.cvtColor(bgr_r, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        x = torch.from_numpy(rgb).permute(2, 0, 1).unsqueeze(0).to(device)
        pred = model(x).argmax(dim=1).squeeze(0).cpu().numpy().astype(np.uint8)
        panel = np.hstack([bgr_r, colorize(gt_r), colorize(pred)])
        cv2.imwrite(str(out_dir / f'val_{i:02d}.png'), panel)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data_dir', type=Path, required=True)
    parser.add_argument('--out_dir', type=Path, default=None)
    parser.add_argument('--epochs', type=int, default=25)
    parser.add_argument('--batch_size', type=int, default=8)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--val_ratio', type=float, default=0.15)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--width', type=int, default=320)
    parser.add_argument('--height', type=int, default=240)
    parser.add_argument('--num_workers', type=int, default=2)
    parser.add_argument('--base', type=int, default=32, help='UNet channel width')
    args = parser.parse_args()

    data_dir = args.data_dir.expanduser().resolve()
    out_dir = (
        args.out_dir.expanduser().resolve()
        if args.out_dir
        else data_dir.parent / 'unet_runs' / time.strftime('%Y%m%d_%H%M%S')
    )
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / 'checkpoints').mkdir(exist_ok=True)
    (out_dir / 'previews').mkdir(exist_ok=True)

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    pairs = list_pairs(data_dir)
    train_pairs, val_pairs = split_pairs(pairs, args.val_ratio, args.seed)
    size = (args.width, args.height)

    train_ds = SegDataset(train_pairs, size, augment=True)
    val_ds = SegDataset(val_pairs, size, augment=False)
    train_loader = DataLoader(
        train_ds,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=torch.cuda.is_available(),
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=torch.cuda.is_available(),
    )

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = UNet(num_classes=NUM_CLASSES, base=args.base).to(device)

    # Class weights from train set (approx via subsample) to balance rare tree class
    class_counts = np.zeros(NUM_CLASSES, dtype=np.float64)
    for _, mask_path in train_pairs[:: max(1, len(train_pairs) // 80)]:
        m = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
        if m is None:
            continue
        if m.ndim == 3:
            m = m[:, :, 0]
        for c in range(NUM_CLASSES):
            class_counts[c] += (m == c).sum()
    class_counts = np.maximum(class_counts, 1.0)
    weights = (class_counts.sum() / (NUM_CLASSES * class_counts)).astype(np.float32)
    weights = torch.tensor(weights, device=device)
    criterion = nn.CrossEntropyLoss(weight=weights)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs)

    meta = {
        'data_dir': str(data_dir),
        'num_train': len(train_pairs),
        'num_val': len(val_pairs),
        'size': [args.width, args.height],
        'classes': CLASS_NAMES,
        'class_weights': weights.detach().cpu().tolist(),
        'device': str(device),
        'base': args.base,
    }
    (out_dir / 'meta.json').write_text(json.dumps(meta, indent=2))
    print(json.dumps(meta, indent=2))
    print(f'Writing run to {out_dir}')

    best_miou = -1.0
    history = []
    for epoch in range(1, args.epochs + 1):
        model.train()
        running = 0.0
        t0 = time.time()
        for x, y in train_loader:
            x = x.to(device)
            y = y.to(device)
            optimizer.zero_grad(set_to_none=True)
            logits = model(x)
            loss = criterion(logits, y)
            loss.backward()
            optimizer.step()
            running += float(loss.item()) * x.size(0)
        scheduler.step()
        train_loss = running / max(len(train_ds), 1)
        metrics = evaluate(model, val_loader, device, criterion)
        history.append(
            {
                'epoch': epoch,
                'train_loss': train_loss,
                'val_loss': metrics['loss'],
                'val_acc': metrics['acc'],
                'val_miou': metrics['miou'],
                'val_iou': metrics['iou'],
                'sec': time.time() - t0,
            }
        )
        iou_str = ' '.join(
            f'{n}={v:.3f}' if v == v else f'{n}=nan' for n, v in zip(CLASS_NAMES, metrics['iou'])
        )
        print(
            f'epoch {epoch:02d}/{args.epochs}  '
            f'train_loss={train_loss:.4f}  val_loss={metrics["loss"]:.4f}  '
            f'acc={metrics["acc"]:.3f}  mIoU={metrics["miou"]:.3f}  [{iou_str}]  '
            f'{history[-1]["sec"]:.1f}s'
        )

        ckpt = {
            'epoch': epoch,
            'model': model.state_dict(),
            'num_classes': NUM_CLASSES,
            'base': args.base,
            'size': [args.width, args.height],
            'classes': CLASS_NAMES,
            'val_miou': metrics['miou'],
        }
        torch.save(ckpt, out_dir / 'checkpoints' / 'last.pt')
        if metrics['miou'] > best_miou:
            best_miou = metrics['miou']
            torch.save(ckpt, out_dir / 'checkpoints' / 'best.pt')
            save_val_previews(model, val_pairs, out_dir / 'previews', device, size)
            print(f'  saved best.pt (mIoU={best_miou:.3f})')

    (out_dir / 'history.json').write_text(json.dumps(history, indent=2))
    print(f'Done. Best mIoU={best_miou:.3f}')
    print(f'Best weights: {out_dir / "checkpoints" / "best.pt"}')
    print(f'Val previews: {out_dir / "previews"}  (left=image, mid=GT, right=pred)')


if __name__ == '__main__':
    main()
