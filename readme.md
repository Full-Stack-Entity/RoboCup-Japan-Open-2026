# RoboCup@Home Simulation 2026 Japan Open

本仓库为 **RoboCup@Home Simulation** 竞赛的 ROS 2 工作空间，基于 [SIGVerse](http://www.sigverse.org/) 仿真平台（Unity + ROS 2 Humble），使用 Toyota HSR（Human Support Robot）完成三项家庭服务任务。

## 竞赛任务概览

| 任务 | 说明 |
|------|------|
| **Handyman** | 机器人根据 Avatar 的自然语言指令，前往指定房间抓取目标物体并搬运回来 |
| **Human Navigation** | 机器人通过生成自然语言引导指令，指导佩戴 VR 头显的测试者完成物体搬运 |
| **Interactive Cleanup** | 机器人根据 Avatar 的肢体指向，识别需要清理的物体和目标位置，自主完成抓取与放置 |

## 目录结构

```
RoboCup_2026_Japan/
├── readme.md
├── rule_reference/                         # 竞赛规则参考文档
│   ├── handyman/                           #   Handyman 任务规则与消息定义
│   ├── human_navigation/                   #   Human Navigation 任务规则与消息定义
│   └── interactive_cleanup/                #   Interactive Cleanup 任务规则与消息定义
│
└── src/                                    # ROS 2 工作空间源码
    ├── interactive_cleanup_ros2/           # [自建] Interactive Cleanup 控制器
    ├── handyman_ros2/                      # [待上传] Handyman 控制器
    ├── human_nav_ros2/                     # [待上传] Human Navigation 控制器
    ├── rcup_vision/                        # [待上传] Handyman 专用视觉感知模块
    ├── ros2-competition-msgs/              # [官方] 竞赛消息定义与测试工具
    └── sigverse_ros_package/               # [官方] SIGVerse ROS Bridge 与示例
```

## 各 Package 说明

### 自建 Package

| Package | 状态 | 说明 |
|---------|------|------|
| `interactive_cleanup_ros2` | 开发中 | Interactive Cleanup 任务的状态机控制器，包含完整的初始化→等待指令→移动→抓取→搬运→释放流程 |
| `handyman_ros2` | 待上传 | Handyman 任务控制器 |
| `human_nav_ros2` | 待上传 | Human Navigation 任务控制器 |
| `rcup_vision` | 待上传 | Handyman 视觉感知模块（物体检测、指向识别等） |

### 官方 Package

| Package | 说明 |
|---------|------|
| `ros2-competition-msgs` | 竞赛消息类型定义（`handyman_msgs`、`human_navigation_msgs`、`interactive_cleanup_msgs`）及各任务的 sample / teleop 测试节点 |
| `sigverse_ros_package` | SIGVerse ROS Bridge（Unity ↔ ROS 2 通信桥接）以及 HSR、TIAGo、TurtleBot3 等机器人的示例代码 |

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
- **Ubuntu 端**：运行 ROS 2 节点，包括 rosbridge、SIGVerse ROS Bridge 和自定义的任务控制器

## HSR 机器人 ROS 话题

| 类别 | 话题 | 方向 |
|------|------|------|
| 底盘速度 | `/hsrb/command_velocity` | ROS → SIGVerse |
| 头部控制 | `/hsrb/head_trajectory_controller/command` | ROS → SIGVerse |
| 手臂控制 | `/hsrb/arm_trajectory_controller/command` | ROS → SIGVerse |
| 夹爪控制 | `/hsrb/gripper_controller/command` | ROS → SIGVerse |
| 关节状态 | `/hsrb/joint_states` | SIGVerse → ROS |
| 激光雷达 | `/hsrb/base_scan` | SIGVerse → ROS |
| RGB 相机 | `/hsrb/head_rgbd_sensor/rgb/image_raw` | SIGVerse → ROS |
| 深度相机 | `/hsrb/head_rgbd_sensor/depth_registered/image_raw` | SIGVerse → ROS |
| 手部相机 | `/hsrb/hand_camera/image_raw` | SIGVerse → ROS |

## 环境搭建

### 依赖项及其配置

- ROS 2 Humble
- rosbridge-suite、slam-toolbox、ros2-control、MoveIt
- Mongo C/C++ Driver（SIGVerse ROS Bridge 依赖）

```bash
sudo rosdep init
rosdep update
sudo apt install -y git
sudo apt install -y libncurses-dev
sudo apt install -y python3-pip
sudo apt install -y ros-$ROS_DISTRO-rosbridge-suite
sudo apt install -y ros-$ROS_DISTRO-slam-toolbox
sudo apt install -y ros-$ROS_DISTRO-xacro
sudo apt install -y ros-$ROS_DISTRO-octomap
sudo apt install -y ros-$ROS_DISTRO-hardware-interface
sudo apt install -y ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-controller-manager
sudo apt install -y ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-ros-perception ros-$ROS_DISTRO-moveit-ros-occupancy-map-monitor

cd ~/下载
wget https://github.com/mongodb/mongo-c-driver/releases/download/2.0.2/mongo-c-driver-2.0.2.tar.gz
tar zxf mongo-c-driver-2.0.2.tar.gz
cd mongo-c-driver-2.0.2/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DENABLE_UNINSTALL=ON
cmake --build .
sudo cmake --install .

cd ~/Downloads
wget https://github.com/mongodb/mongo-cxx-driver/releases/download/r4.1.1/mongo-cxx-driver-r4.1.1.tar.gz
tar zxf mongo-cxx-driver-r4.1.1.tar.gz
cd mongo-cxx-driver-r4.1.1/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_PREFIX_PATH=/usr/local
cmake --build .
sudo cmake --install .
sudo ldconfig
```


### 编译工作空间

```bash
cd RoboCup_2026_Japan
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 快速启动

快速启动示例：

```bash
ros2 launch competition_test_tools handyman_sample_launch.xml
ros2 launch competition_test_tools interactive_cleanup_sample_launch.xml
ros2 launch competition_test_tools human_navigation_sample_launch.xml
```

键盘控制启动示例：
```bash
ros2 launch competition_test_tools teleop_handyman_launch.xml
ros2 launch competition_test_tools teleop_interactive_cleanup_launch.xml
ros2 launch competition_test_tools teleop_human_navigation_launch.xml
```

## 参考链接

- [SIGVerse 官方文档](http://www.sigverse.org/)
- [ros2-competition-msgs](https://github.com/RoboCupatHomeSim/ros2-competition-msgs)
- [sigverse_ros_package](https://github.com/SIGVerse/sigverse_ros_package)
- [RoboCup@Home Simulation Wiki](https://github.com/RoboCupatHomeSim)
