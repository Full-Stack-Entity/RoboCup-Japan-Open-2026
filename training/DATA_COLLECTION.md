# Unified YOLO Training Guide

本文件只说明统一 `training/` 工程的使用方法、目录结构、27 类编号和注意事项。

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
    images/
      train/
      val/
      test/
    labels/
      train/
      val/
      test/
  scripts/
    extract_frames.py
    split_dataset.py
    train.py
  runs/
  exports/
  weights/
```

## 使用说明

### 1. 准备原始数据

原始数据目录直接使用 Label Studio 导出的 YOLO 结构：

```text
training/data/raw/
  images/
  labels/
  classes.txt
  notes.json
```

要求：

- `images/` 中存放全部待训练图片
- `labels/` 中存放同名 YOLO 标签文件
- `classes.txt` 必须与 `training/classes.txt` 完全一致
- `notes.json` 保留 Label Studio 导出元数据

### 2. 从视频抽帧

如果你先从视频生成图片，可以使用：

```bash
pixi run python training/scripts/extract_frames.py \
  --video /path/to/video.mp4 \
  --prefix sugar_table_near
```

默认输出到：

```text
training/data/raw/images/
```

如果要一次处理多个视频，可以重复传入 `--video`。

### 3. 划分训练集/验证集/测试集

在 `training/data/raw/` 准备完成后，运行：

```bash
pixi run python training/scripts/split_dataset.py --clean
```

默认会：

- 从 `training/data/raw/images/` 读取图片
- 从 `training/data/raw/labels/` 读取标签
- 校验 `training/data/raw/classes.txt` 与 `training/classes.txt`
- 将数据划分到：

```text
training/data/images/{train,val,test}/
training/data/labels/{train,val,test}/
```

如果需要自定义比例，可以使用：

```bash
pixi run python training/scripts/split_dataset.py \
  --clean \
  --train-ratio 0.7 \
  --val-ratio 0.2 \
  --test-ratio 0.1
```

### 4. 开始训练

统一训练入口：

```bash
pixi run python training/scripts/train.py \
  --epochs 120 \
  --batch 16 \
  --name unified_baseline
```

默认设置：

- 数据配置：`training/dataset.yaml`
- 输出目录：`training/runs`
- 基准模型：`yolo26n.pt`

如果你已经手动准备了本地权重，也可以显式指定：

```bash
pixi run python training/scripts/train.py \
  --model training/weights/yolo26n.pt \
  --epochs 120 \
  --batch 16 \
  --name unified_baseline
```

### 5. 训练结果位置

训练输出默认位于：

```text
training/runs/<name>/
```

常用文件：

- `training/runs/<name>/weights/best.pt`
- `training/runs/<name>/weights/last.pt`

如果要部署到两个任务的运行时包，可分别复制：

```bash
cp training/runs/unified_baseline/weights/best.pt \
  src/handyman_vision_ros2/models/handyman_model.pt

cp training/runs/unified_baseline/weights/best.pt \
  src/cleanup_vision_ros2/models/cleanup_model.pt
```

## 27 类物品编号

统一类别顺序必须固定如下：

| ID | 类别 | ID | 类别 | ID | 类别 |
|----|------|----|------|----|------|
| 0 | apple | 9 | filled_ketchup | 18 | sauce |
| 1 | bear_doll | 10 | filled_plastic_bottle | 19 | soysauce |
| 2 | camera | 11 | ground_pepper | 20 | spray_bottle |
| 3 | canned_juice | 12 | hourglass | 21 | sugar |
| 4 | cigarette | 13 | nursing_bottle | 22 | toy_car |
| 5 | cubic_clock | 14 | pink_cup | 23 | toy_duck |
| 6 | dog_doll | 15 | rabbit_doll | 24 | toy_penguin |
| 7 | empty_ketchup | 16 | rubiks_cube | 25 | tumbler |
| 8 | empty_plastic_bottle | 17 | salt | 26 | white_cup |

对应的完整类表文件见：

- `training/classes.txt`
- `training/data/raw/classes.txt`
- `training/dataset.yaml`

## 标注格式

统一使用 YOLO 检测格式：

```text
class_id center_x center_y width height
```

其中：

- `class_id` 必须使用上面的 27 类编号
- 坐标必须归一化到 `[0, 1]`
- 每张图片对应一个同名 `.txt`
- 没有目标的负样本可以保留空 `.txt`

## 注意事项

- 现在不再使用 `session/` 目录结构。
- `training/data/raw/` 必须是扁平的 Label Studio 导出结构。
- `split_dataset.py` 现在按图像级别随机切分，不再按 session 切分。
- 因为取消了 session 切分，导出前必须主动删除大量近重复帧，否则 `val/test` 会过于乐观。
- `raw/classes.txt` 与 `training/classes.txt` 只要顺序或命名有任何不一致，`split_dataset.py` 就会直接报错。
- 默认训练基准模型已经改为 `yolo26n.pt`，不再使用 `yolo12n.pt`。
- 所有命令都应在 `pixi` 环境中运行，例如使用 `pixi run python ...`。
