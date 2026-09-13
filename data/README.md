# Data artefacts

| Path | Purpose |
|------|---------|
| `models/unet_outdoor_best.pt` | Trained outdoor segmentation weights |
| `models/unet_outdoor_meta.json` | Class / training metadata |
| `maps/rtabmap.db` | Active RTAB-Map database (mapping/localization) |
| `maps/rtabmap_latest_saved.db` | Saved courtyard map snapshot |
| `dataset_autolabel/` | Auto-labelled train set (images + masks) |
| `images_*` | Raw capture sessions used for labelling |

Not uploaded: `unet_runs/` checkpoints (duplicates of best weights) and `*.back` map backups.
