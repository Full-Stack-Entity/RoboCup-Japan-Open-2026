# models/

Place your trained YOLO weights here as `last.pt`.

The node (`object_detection_node.py`) will:
1. First look for `models/last.pt` (your custom-trained weights).
2. If not found, automatically download and use `yolo11n.pt`
   (YOLOv11 nano pretrained on COCO) from the ultralytics hub.

## Upgrading from YOLOv8 to YOLOv11

YOLOv11 is the current best model in the ultralytics lineup (2024).
It offers improved accuracy and speed over YOLOv8 with no API changes.

To retrain with your custom dataset:

```bash
pip install ultralytics>=8.3.0
yolo train model=yolo11n.pt data=your_dataset.yaml epochs=100 imgsz=640
```

Then copy the resulting `runs/detect/train/weights/best.pt` here as `last.pt`.
