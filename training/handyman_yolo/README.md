# Handyman Custom YOLO Workspace

This folder is a local training workspace for a custom detector that can
recognize the 26 graspable object classes in the Handyman task.

The runtime ROS package stays unchanged. Training happens here, and the
final model is copied to the appropriate runtime location.

## Layout

```text
training/handyman_yolo/
  classes.txt
  dataset.yaml
  DATA_COLLECTION.md
  data/
    raw/                 # capture sessions before dataset split
    images/{train,val,test}/
    labels/{train,val,test}/
  scripts/
    extract_frames.py    # sample frames from recorded videos
    split_dataset.py     # split raw sessions into YOLO train/val/test
    train.py             # start Ultralytics training with sane defaults
  runs/                  # Ultralytics outputs
  weights/               # optional place for downloaded base weights
  exports/               # optional exported models
```

## Quick Start

1. Read `training/handyman_yolo/DATA_COLLECTION.md`.
2. Record one capture video per session and extract frames:

   ```bash
   pixi run python training/handyman_yolo/scripts/extract_frames.py \
     --video /path/to/session.mp4 \
     --session 20260320_bear_doll_scan_01 \
     --every-seconds 0.4
   ```

3. Annotate the images under
   `training/handyman_yolo/data/raw/<session>/images/` and save YOLO-format
   labels to `training/handyman_yolo/data/raw/<session>/labels/`.

4. Build the train/val/test split by session:

   ```bash
   pixi run python training/handyman_yolo/scripts/split_dataset.py --clean
   ```

5. Train from the current pretrained base model:

   ```bash
   pixi run python training/handyman_yolo/scripts/train.py \
     --epochs 120 \
     --batch 16 \
     --name handyman_baseline
   ```

6. After training, copy the best checkpoint into the runtime package:

   ```bash
   cp training/handyman_yolo/runs/handyman_baseline/weights/best.pt \
     src/handyman_vision_ros2/models/handyman_model.pt
   ```

7. Relaunch:

   ```bash
   ros2 launch handyman handyman.launch.py
   ```

## Graspable Object Classes (26)

The Handyman task uses 26 types of graspable objects across 4 layouts
(LayoutA-D). Classes are derived from source code analysis and
`EnvironmentInfo01-06.json` configuration files.

| Category | Classes |
|----------|---------|
| Toys/Dolls (7) | bear_doll, rabbit_doll, dog_doll, toy_penguin, toy_duck, toy_car, rubiks_cube |
| Containers (6) | tumbler, pink_cup, white_cup, empty_plastic_bottle, empty_ketchup, nursing_bottle |
| Beverages (2) | canned_juice, filled_plastic_bottle |
| Condiments (6) | soysauce, sauce, filled_ketchup, sugar, ground_pepper, salt |
| Food (1) | apple |
| Electronics (1) | camera |
| Misc (3) | cigarette, cubic_clock, hourglass |

Destination objects (tables, shelves, trash boxes, etc.) are not included
because their positions are hardcoded in the Handyman source code.

## Notes

- This workspace intentionally does not hardcode any map positions.
- Large image/video data and training outputs are ignored by
  `training/handyman_yolo/.gitignore`.
- `.pt` files are also ignored globally by the repo root `.gitignore`.
- Handyman has 4 room layouts (LayoutA-D) with different furniture
  arrangements; ensure data collection covers multiple layouts.
