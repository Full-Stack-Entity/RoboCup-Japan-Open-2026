# Handyman Package - ROS2 Humble

ROS2 handyman package for HSR robot manipulation and navigation tasks.

## Features

- **ROS2 Native**: Full migration to ROS2 Humble
- **Nav2 Integration**: Uses Nav2 for navigation
- **Modern C++17**: Updated to modern C++ standards
- **Vision Integration**: Works with YOLOv11 vision package

## Installation

```bash
cd ~/ros2_ws/src
colcon build --packages-select handyman
source install/setup.bash
```

## Usage

```bash
ros2 launch handyman handyman.launch.py
```

## License

SIGVerse License
