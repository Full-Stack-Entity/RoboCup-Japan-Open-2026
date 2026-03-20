# Interactive Cleanup 自定义 YOLO 训练工作区

本文件夹是用于训练自定义检测器的本地工作区，训练完成后可替换
`src/cleanup_vision_ros2/scripts/cleanup_detection_node.py` 中当前使用的
通用 COCO 模型。

运行时的 ROS 包无需改动。训练在此处进行，最终模型复制到：

`src/cleanup_vision_ros2/models/cleanup_model.pt`

## 目录结构

```text
training/cleanup_yolo/
  classes.txt
  dataset.yaml
  DATA_COLLECTION.md
  data/
    raw/                 # 数据集切分前的采集会话
    images/{train,val,test}/
    labels/{train,val,test}/
  scripts/
    extract_frames.py    # 从录制视频中抽帧
    split_dataset.py     # 按会话将原始数据切分为 YOLO train/val/test
    train.py             # 使用 Ultralytics 启动训练
  runs/                  # Ultralytics 训练输出
  weights/               # 可选：存放下载的基础权重
  exports/               # 可选：导出的模型格式
```

## 快速开始

1. 阅读 `training/cleanup_yolo/DATA_COLLECTION.md`。
2. 每次采集录制一段视频，然后抽帧：

   ```bash
   pixi run python training/cleanup_yolo/scripts/extract_frames.py \
     --video /path/to/session.mp4 \
     --session 20260318_soysauce_scan_01 \
     --every-seconds 0.4
   ```

3. 对 `training/cleanup_yolo/data/raw/<session>/images/` 下的图片进行标注，
   将 YOLO 格式的标签保存到 `training/cleanup_yolo/data/raw/<session>/labels/`。

4. 按会话构建 train/val/test 切分：

   ```bash
   pixi run python training/cleanup_yolo/scripts/split_dataset.py --clean
   ```

5. 基于预训练模型开始训练：

   ```bash
   pixi run python training/cleanup_yolo/scripts/train.py \
     --epochs 120 \
     --batch 16 \
     --name cleanup_baseline
   ```

6. 训练完成后，将最佳权重复制到运行时包中：

   ```bash
   cp training/cleanup_yolo/runs/cleanup_baseline/weights/best.pt \
     src/cleanup_vision_ros2/models/cleanup_model.pt
   ```

7. 重新启动：

   ```bash
   ros2 launch interactive_cleanup cleanup.launch.py
   ```

## 可抓取物体类别 (13 类)

`classes.txt` 和 `dataset.yaml` 基于源码分析和
`unity_reference/SIGVerseConfig` 中的 Interactive Cleanup 配置初始化。

| 分类 | 类别 |
|------|------|
| 玩具/玩偶 (4) | bear_doll, rabbit_doll, dog_doll, toy_penguin |
| 容器/餐具 (1) | tumbler |
| 饮品 (2) | canned_juice, filled_plastic_bottle |
| 调味品 (4) | soysauce, sauce, filled_ketchup, sugar |
| 清洁用品 (1) | spray_bottle |
| 杂物 (1) | hourglass |

放置目的地（桌子、架子、垃圾桶等）的坐标已硬编码在代码中，不需要视觉识别。

如果官方物体集发生变化，请在标注前同步更新这两个文件。

## 注意事项

- 本工作区不包含任何硬编码的地图坐标。
- 大型图像/视频数据和训练输出已通过
  `training/cleanup_yolo/.gitignore` 忽略。
- `.pt` 文件也被仓库根目录的 `.gitignore` 全局忽略。
