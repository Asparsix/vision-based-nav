# Data artefacts

| Path | Purpose |
|------|---------|
| `images_courtyard_20260916_213150/` | Raw RGB captures from the **updated enclosed courtyard** world (350 frames) |
| `dataset_autolabel/` | Auto-labelled train set for that capture (`images/` + `masks/` + `previews/`) |
| `models/unet_outdoor_best.pt` | UNet weights fine-tuned on the courtyard dataset |
| `models/unet_outdoor_meta.json` | Training metadata (classes, mIoU, etc.) |
| `maps/` | RTAB-Map databases |

**Removed:** older town/open-world image folders and their previous labels (no longer match the current Gazebo world).

Not uploaded: `unet_runs/` checkpoints, `*.back` map backups, debug overlays.
