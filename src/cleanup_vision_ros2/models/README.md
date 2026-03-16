# models/

Place YOLO weights here. The node searches in order:

1. `cleanup_model.pt` — custom-trained on competition objects
2. `last.pt` — any custom training output
3. `yolo12n.pt` — COCO pretrained (auto-downloaded if missing)

COCO pretrained can detect ~80 common classes (person, bottle, cup,
dining table, chair, etc.) which covers many competition scenarios.
