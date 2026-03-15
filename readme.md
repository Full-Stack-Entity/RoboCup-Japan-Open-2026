# RoboCup@Home Simulation 2026 Japan Open

本仓库为 **RoboCup@Home Simulation** 竞赛的 ROS 2 工作空间，基于 [SIGVerse](http://www.sigverse.org/) 仿真平台（Unity + ROS 2 Humble），使用 Toyota HSR（Human Support Robot）完成三项家庭服务任务。

## 竞赛任务概览


| 任务                      | 说明                                          |
| ----------------------- | ------------------------------------------- |
| **Handyman**            | 机器人根据 Avatar 的自然语言指令，前往指定房间抓取目标物体并搬运回来      |
| **Human Navigation**    | 机器人通过生成自然语言引导指令，指导佩戴 VR 头显的测试者完成物体搬运        |
| **Interactive Cleanup** | 机器人根据 Avatar 的肢体指向，识别需要清理的物体和目标位置，自主完成抓取与放置 |


## 目录结构

以下为仓库中**受版本控制**的目录与文件，不包含 `.gitignore` 中忽略的 `build/`、`install/`、`log/`、`.pixi/`、`*.pt` 等。

```
RoboCup-Japan-Open-2026/
├── readme.md
├── .gitignore
├── pixi.toml                    # pixi 环境配置（Python 依赖，见下方「使用 pixi 配置环境」）
├── pixi.lock                    # 依赖锁定
├── rule_reference/              # 竞赛规则参考文档
│   ├── handyman/
│   ├── human_navigation/
│   └── interactive_cleanup/
│
└── src/                         # ROS 2 工作空间源码
    ├── handyman_ros2/           # Handyman 控制器（主控 + 键盘遥控）
    ├── vision_ros2/             # Handyman 视觉感知（YOLOv12 物体检测）
    ├── human_nav_ros2/          # Human Navigation 控制器
    ├── interactive_cleanup_ros2/# Interactive Cleanup 控制器
    ├── ros2-competition-msgs/    # 竞赛消息定义与测试工具（官方）
    │   ├── handyman_msgs/
    │   ├── human_navigation_msgs/
    │   ├── interactive_cleanup_msgs/
    │   └── competition_test_tools/
    └── sigverse_ros_package/     # SIGVerse ROS Bridge 与示例（官方）
```

## 各 Package 说明

### 自建 Package


| Package                    | 说明                                                    |
| -------------------------- | ----------------------------------------------------- |
| `handyman_ros2`            | Handyman 任务控制器（主控状态机、键盘遥控），依赖 `vision_ros2` 做物体检测     |
| `vision_ros2`              | Handyman 视觉模块（YOLOv12，ultralytics），需 Python 依赖（见环境配置） |
| `human_nav_ros2`           | Human Navigation 任务控制器                                |
| `interactive_cleanup_ros2` | Interactive Cleanup 任务控制器                             |


### 官方 Package


| Package                 | 说明                                                                                               |
| ----------------------- | ------------------------------------------------------------------------------------------------ |
| `ros2-competition-msgs` | 竞赛消息类型（`handyman_msgs`、`human_navigation_msgs`、`interactive_cleanup_msgs`）及 sample / teleop 测试节点 |
| `sigverse_ros_package`  | SIGVerse ROS Bridge（Unity ↔ ROS 2）及 HSR、TIAGo、TurtleBot3 等示例                                     |


## 系统架构

```
┌──────────────────────┐          ┌──────────────────────────────────┐
│   Windows (Unity)    │          │         Ubuntu (ROS 2)           │
│                      │  TCP/IP  │                                  │
│  SIGVerse 仿真环境    │◄────────►│  rosbridge + sigverse_ros_bridge │
│  (HSR + 场景 + Avatar)│          │  + 机器人控制器节点               │
└──────────────────────┘          └──────────────────────────────────┘
```

- **Windows 端**：运行 Unity 仿真程序，渲染场景、物理引擎、Avatar 交互  
- **Ubuntu 端**：运行 ROS 2 节点（rosbridge、SIGVerse ROS Bridge、任务控制器）

## HSR 机器人 ROS 话题


| 类别     | 话题                                                  | 方向             |
| ------ | --------------------------------------------------- | -------------- |
| 底盘速度   | `/hsrb/command_velocity`                            | ROS → SIGVerse |
| 头部控制   | `/hsrb/head_trajectory_controller/command`          | ROS → SIGVerse |
| 手臂控制   | `/hsrb/arm_trajectory_controller/command`           | ROS → SIGVerse |
| 夹爪控制   | `/hsrb/gripper_controller/command`                  | ROS → SIGVerse |
| 关节状态   | `/hsrb/joint_states`                                | SIGVerse → ROS |
| 激光雷达   | `/hsrb/base_scan`                                   | SIGVerse → ROS |
| RGB 相机 | `/hsrb/head_rgbd_sensor/rgb/image_raw`              | SIGVerse → ROS |
| 深度相机   | `/hsrb/head_rgbd_sensor/depth_registered/image_raw` | SIGVerse → ROS |
| 手部相机   | `/hsrb/hand_camera/image_raw`                       | SIGVerse → ROS |


## 环境搭建

### 1. 系统依赖（apt）

- ROS 2 Humble  
- rosbridge、slam-toolbox、Nav2、MoveIt 等  
- Mongo C/C++ Driver（SIGVerse ROS Bridge 依赖）

```bash
sudo rosdep init
rosdep update
sudo apt install -y git libncurses-dev python3-pip
sudo apt install -y ros-$ROS_DISTRO-rosbridge-suite
sudo apt install -y ros-$ROS_DISTRO-slam-toolbox
sudo apt install -y ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-octomap
sudo apt install -y ros-$ROS_DISTRO-hardware-interface
sudo apt install -y ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-controller-manager
sudo apt install -y ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-ros-perception ros-$ROS_DISTRO-moveit-ros-occupancy-map-monitor
```

**Mongo C/C++ 驱动安装**（SIGVerse ROS Bridge 依赖，主目录为 `~/下载`）：

```bash
cd ~/下载
wget https://github.com/mongodb/mongo-c-driver/releases/download/2.0.2/mongo-c-driver-2.0.2.tar.gz
tar zxf mongo-c-driver-2.0.2.tar.gz
cd mongo-c-driver-2.0.2/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DENABLE_UNINSTALL=ON
cmake --build .
sudo cmake --install .

cd ~/下载
wget https://github.com/mongodb/mongo-cxx-driver/releases/download/r4.1.1/mongo-cxx-driver-r4.1.1.tar.gz
tar zxf mongo-cxx-driver-r4.1.1.tar.gz
cd mongo-cxx-driver-r4.1.1/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_PREFIX_PATH=/usr/local
cmake --build .
sudo cmake --install .
sudo ldconfig
```

### 2. Python 依赖（vision_ros2 / Handyman 视觉）

**方式一：使用 pixi（推荐）**

[pixi](https://pixi.sh/) 在项目根目录管理 Python 环境，与 ROS2 的 Python 版本（3.10）和 NumPy 版本（<2）对齐，避免与 `cv_bridge`、`rclpy` 冲突。

```bash
# 安装 pixi
curl -fsSL https://pixi.sh/install.sh | bash

# 在仓库根目录安装依赖
cd /path/to/RoboCup-Japan-Open-2026
pixi install
```

运行 ROS2 时，先 source ROS2 与工作空间，再进入 pixi 环境：

```bash
cd /path/to/RoboCup-Japan-Open-2026
source install/setup.bash
pixi shell
ros2 launch handyman_ros2 hsr_nav.launch.py
```

**方式二：系统 pip**

```bash
pip3 install -r src/vision_ros2/requirements.txt
```

需保证 Python 3.10 且 `numpy<2`，以兼容 ROS Humble 的 `cv_bridge`。

### 3. 编译工作空间

```bash
cd /path/to/RoboCup-Japan-Open-2026
colcon build --symlink-install --packages-skip sigverse_turtlebot3
source install/setup.bash
```

可选：将上述两条 `source` 写入 `~/.bashrc`，避免每次手动执行。

## 快速启动

**竞赛官方 sample / teleop（competition_test_tools）：**

```bash
ros2 launch competition_test_tools handyman_sample_launch.xml
ros2 launch competition_test_tools interactive_cleanup_sample_launch.xml
ros2 launch competition_test_tools human_navigation_sample_launch.xml
```

键盘控制：

```bash
ros2 launch competition_test_tools teleop_handyman_launch.xml
ros2 launch competition_test_tools teleop_interactive_cleanup_launch.xml
ros2 launch competition_test_tools teleop_human_navigation_launch.xml
```

**自建 Handyman 全栈（主控 + 视觉 + RViz2 + rosbridge）：**

```bash
# 若使用 pixi：先 source ROS2 与 install/setup，再 pixi shell，然后：
ros2 launch handyman_ros2 hsr_nav.launch.py
```

更多 launch 与键盘遥控说明见 `src/handyman_ros2/README.md`。

## 参考链接

- [SIGVerse 官方文档](http://www.sigverse.org/)
- [ros2-competition-msgs](https://github.com/RoboCupatHomeSim/ros2-competition-msgs)
- [sigverse_ros_package](https://github.com/SIGVerse/sigverse_ros_package)
- [RoboCup@Home Simulation Wiki](https://github.com/RoboCupatHomeSim)

