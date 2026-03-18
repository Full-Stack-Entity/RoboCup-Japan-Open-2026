# Interactive Cleanup Custom YOLO Workspace

This folder is a local training workspace for a custom detector that can
replace the generic COCO model currently used by
`src/cleanup_vision_ros2/scripts/cleanup_detection_node.py`.

The runtime ROS package stays unchanged. Training happens here, and the
final model is copied to:

`src/cleanup_vision_ros2/models/cleanup_model.pt`

## Layout

```text
training/cleanup_yolo/
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

1. Read `training/cleanup_yolo/DATA_COLLECTION.md`.
2. Record one capture video per session and extract frames:

   ```bash
   pixi run python training/cleanup_yolo/scripts/extract_frames.py \
     --video /path/to/session.mp4 \
     --session 20260318_soysauce_scan_01 \
     --every-seconds 0.4
   ```

3. Annotate the images under
   `training/cleanup_yolo/data/raw/<session>/images/` and save YOLO-format
   labels to `training/cleanup_yolo/data/raw/<session>/labels/`.

4. Build the train/val/test split by session:

   ```bash
   pixi run python training/cleanup_yolo/scripts/split_dataset.py --clean
   ```

5. Train from the current pretrained base model:

   ```bash
   pixi run python training/cleanup_yolo/scripts/train.py \
     --epochs 120 \
     --batch 16 \
     --name cleanup_baseline
   ```

6. After training, copy the best checkpoint into the runtime package:

   ```bash
   cp training/cleanup_yolo/runs/cleanup_baseline/weights/best.pt \
     src/cleanup_vision_ros2/models/cleanup_model.pt
   ```

7. Relaunch:

   ```bash
   ros2 launch interactive_cleanup cleanup.launch.py
   ```

## Starter Class List

`classes.txt` and `dataset.yaml` are initialized from the current
Interactive Cleanup reference assets in `unity_reference/SIGVerseConfig`.
Treat them as a starting point, not as an immutable competition truth.
If the official object set changes, update both files before labeling.

## Notes

- This workspace intentionally does not hardcode any map positions.
- Large image/video data and training outputs are ignored by
  `training/cleanup_yolo/.gitignore`.
- `.pt` files are also ignored globally by the repo root `.gitignore`.
