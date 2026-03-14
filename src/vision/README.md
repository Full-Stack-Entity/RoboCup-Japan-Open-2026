# Vision Package - ROS2 Humble

ROS2 vision package with YOLOv11 object detection for HSR robot manipulation tasks.

## Features

- **YOLOv11 Integration**: Latest YOLO version for improved detection accuracy and speed
- **Dual Camera Support**: Head camera and hand camera detection
- **Depth Integration**: 3D object localization using depth information
- **ROS2 Native**: Built for ROS2 Humble with modern Python3 APIs
- **Real-time Detection**: Optimized for real-time robot manipulation

## Upgrade from Original

- **YOLO Version**: Upgraded from YOLOv8 to YOLOv11
- **ROS Version**: Migrated from ROS1 to ROS2 Humble
- **Architecture**: Modern ROS2 node structure with parameters and launch files
- **Performance**: Improved detection speed and accuracy

## Installation

### Prerequisites

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Python dependencies
pip3 install -r requirements.txt
```

### Build

```bash
cd ~/ros2_ws/src
# Copy this package here
cd ~/ros2_ws
colcon build --packages-select vision
source install/setup.bash
```

## Usage

### Launch the vision node

```bash
ros2 launch vision vision.launch.py
```

### With custom model

```bash
ros2 launch vision vision.launch.py model_path:=path/to/your/model.pt
```

### Set detection target

```bash
ros2 topic pub /detection_target std_msgs/msg/String "data: 'apple'"
```

## Topics

### Subscribed Topics

- `/hsrb/head_rgbd_sensor/rgb/image_raw` (sensor_msgs/Image): Head camera RGB
- `/hsrb/hand_camera/image_raw` (sensor_msgs/Image): Hand camera RGB
- `/hsrb/head_rgbd_sensor/depth_registered/image_raw` (sensor_msgs/Image): Depth image
- `/detection_target` (std_msgs/String): Target object name

### Published Topics

- `/vision` (geometry_msgs/PoseStamped): Detected object pose in map frame
- `/detection_depth` (std_msgs/Float32): Object depth
- `/hand_detection` (std_msgs/Int32MultiArray): Hand camera detection [x, y, w, h]

## Parameters

- `model_path`: Path to YOLOv11 model (default: models/yolov11n.pt)
- `confidence_threshold`: Detection confidence threshold (default: 0.5)
- `hand_confidence_threshold`: Hand camera confidence threshold (default: 0.1)

## Supported Objects

30 object classes including:
- Household items: apple, clock, hourglass
- Toys: bear_doll, dog_doll, rabbit_doll, toy_car, toy_duck, toy_penguin
- Containers: cups, bottles, ketchup
- Condiments: salt, pepper, sugar, sauce, soysauce
- Furniture: white_side_table
- And more...

## Model Training

To train your own YOLOv11 model:

```python
from ultralytics import YOLO

# Load a model
model = YOLO('yolov11n.pt')

# Train the model
results = model.train(data='your_dataset.yaml', epochs=100, imgsz=640)
```

## License

AGPL-3.0

## Maintainer

mahmoud@todo.todo
