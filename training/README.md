# Unified YOLO Training Workspace

这个 `training/` 目录现在只维护一套统一的 YOLO 检测工程，服务于：

- Handyman
- Interactive Cleanup

最终只训练一个 27 类检测模型，并分别部署到两个任务的运行时包。

## 目录结构

```text
training/
  README.md
  DATA_COLLECTION.md
  classes.txt
  dataset.yaml
  data/
    raw/
      images/
      labels/
      classes.txt
      notes.json
    images/{train,val,test}/
    labels/{train,val,test}/
  scripts/
    extract_frames.py
    split_dataset.py
    train.py
  tests/
    test_training_scripts.py
  runs/
  exports/
  weights/
```

## 快速开始

1. 准备原始图片和 Label Studio 标注导出：

   ```text
   training/data/raw/
     images/
     labels/
     classes.txt
     notes.json
   ```

2. 确保 `training/data/raw/classes.txt` 与 `training/classes.txt` 完全一致。

3. 划分数据集：

   ```bash
   pixi run python training/scripts/split_dataset.py --clean
   ```

4. 使用 YOLO26 基线开始训练：

   ```bash
   pixi run python training/scripts/train.py \
     --epochs 120 \
     --batch 16 \
     --name unified_baseline
   ```

5. 训练完成后，将最佳权重分别复制到两个运行时包：

   ```bash
   cp training/runs/unified_baseline/weights/best.pt \
     src/handyman_vision_ros2/models/handyman_model.pt

   cp training/runs/unified_baseline/weights/best.pt \
     src/cleanup_vision_ros2/models/cleanup_model.pt
   ```

## YOLO26 基线

- `train.py` 默认模型是 `yolo26n.pt`
- 你可以直接使用 Ultralytics 模型别名，让框架自动解析/下载
- 如果你已经手动下载了权重，也可以显式传入：

  ```bash
  pixi run python training/scripts/train.py \
    --model training/weights/yolo26n.pt
  ```

## 27 类列表

统一类别表是 Handyman 26 类与 Cleanup 独有的 `spray_bottle` 的并集：

`apple, bear_doll, camera, canned_juice, cigarette, cubic_clock, dog_doll, empty_ketchup, empty_plastic_bottle, filled_ketchup, filled_plastic_bottle, ground_pepper, hourglass, nursing_bottle, pink_cup, rabbit_doll, rubiks_cube, salt, sauce, soysauce, spray_bottle, sugar, toy_car, toy_duck, toy_penguin, tumbler, white_cup`

## 注意事项

- 现在不再使用 `session` 子目录。
- `split_dataset.py` 会按图像级别随机切分 `train/val/test`。
- 因为取消了 session 切分，导出前必须主动删除大量近重复帧，否则验证集会偏乐观。
- `training/data/raw/notes.json` 目前主要作为 Label Studio 导出的伴随文件保留，训练脚本不依赖其中的字段。
