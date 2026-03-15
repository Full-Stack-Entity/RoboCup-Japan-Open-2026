# handyman-ros2 — ROS 2 Humble 移植版

原稿（ROS1 Noetic）完整移植到 ROS2 Humble，包含 bug 修复。

本包负责机器人主控逻辑，配合 `rcup_vision` 包使用。

---

## 目录

1. [前置条件](#前置条件)
2. [环境搭建（第一次必做）](#环境搭建第一次必做)
3. [编译](#编译)
4. [准备模型文件](#准备模型文件)
5. [运行流程](#运行流程)
   - [第一步：建图（新场地必做）](#第一步建图新场地必做)
   - [第二步：正式运行比赛任务](#第二步正式运行比赛任务)
   - [其他运行模式](#其他运行模式)
6. [键盘遥控说明](#键盘遥控说明)
7. [常见问题](#常见问题)
8. [包结构](#包结构)
9. [话题对照表](#话题对照表)

---

## 前置条件

- **Ubuntu 22.04 LTS**
- **ROS2 Humble** — 安装教程：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- **Python 3.10+**（Ubuntu 22.04 自带）

安装检查：

```bash
ros2 --version        # 应输出 ros2 cli version x.x.x
python3 --version     # 应输出 Python 3.10.x 或更高
```

---

## 环境搭建（第一次必做）

### 1. 创建工作空间

```bash
mkdir -p ~/ros2_ws/src
```

### 2. 复制包到工作空间

**注意：** 两个包的文件夹名称分别是 `handyman-ros2` 和 `rcup_vision`（vision 包），复制时要使用正确的名称。

```bash
# 根据实际存放路径修改源路径
cp -r ~/Desktop/ros2改稿/handyman-ros2  ~/ros2_ws/src/handyman
cp -r ~/Desktop/ros2改稿/vision-ros2    ~/ros2_ws/src/rcup_vision
```

> **重要：** 目标路径中 `handyman-ros2` 必须复制为 `handyman`，`vision-ros2` 必须复制为 `rcup_vision`，否则 ROS2 找不到包。

### 3. 安装 ROS2 系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rosbridge-server \
  python3-colcon-common-extensions \
  python3-tf-transformations \
  xterm
```

> `python3-tf-transformations` — vision 包的 TF 坐标变换依赖。  
> `xterm` — 键盘遥控节点需要在独立终端窗口中运行。

### 4. 安装 Python 依赖

```bash
pip3 install -r ~/ros2_ws/src/rcup_vision/requirements.txt
```

国内加速：
```bash
pip3 install -r ~/ros2_ws/src/rcup_vision/requirements.txt \
  -i https://pypi.tuna.tsinghua.edu.cn/simple
```

---

## 编译

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

编译成功标志：
```
Summary: 2 packages finished [handyman, rcup_vision]
```

> **每次修改代码后**都需要重新执行 `colcon build` 和 `source install/setup.bash`。

建议将 source 命令加入 `~/.bashrc`，避免每次手动执行：

```bash
echo "source /opt/ros/humble/setup.bash"   >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 准备模型文件

视觉检测需要 YOLOv12 模型文件，**必须手动放置**：

```bash
# 将训练好的模型文件重命名为 last.pt，放到以下路径
cp /path/to/your/model.pt ~/ros2_ws/src/rcup_vision/models/last.pt
```

放置后**重新编译**，让 colcon 把模型安装到正确位置：

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

如果没有模型文件，视觉节点会自动下载 `yolo12n.pt` 预训练权重（需联网），但检测精度会低于训练好的自定义模型。

---

## 运行流程

> **每次新开终端**都需要先执行（已加入 `~/.bashrc` 则无需手动执行）：
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/ros2_ws/install/setup.bash
> ```

---

### 第一步：建图（新场地必做）

**如果已有场地地图（`maps/` 目录下有对应的 `.pgm` + `.yaml`），跳过此步。**

建图需要同时开两个终端。

**终端 1 — 启动建图程序：**

```bash
ros2 launch handyman make_map.launch.py
```

启动后会弹出两个窗口：
- **RViz2** — 实时显示构建中的地图
- **xterm 键盘遥控窗口** — 在此窗口中操作键盘控制机器人移动

**在 xterm 键盘遥控窗口中操作：**

用方向键控制机器人在整个场地内缓慢移动，直到 RViz2 中地图轮廓完整清晰。

**终端 2 — 保存地图：**

```bash
# 将地图保存到 handyman 包的 maps 目录，文件名与场地名一致
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/handyman/maps/MyMap
```

会生成两个文件：`MyMap.pgm`（地图图像）和 `MyMap.yaml`（地图配置）。

**注册新地图（修改代码）：**

打开 `handyman-ros2/src/handyman_sample.cpp`，找到 `registerDefaultEnvironments()` 函数，添加新地图：

```cpp
void registerDefaultEnvironments() {
    geometry_msgs::msg::Pose p;
    p.orientation.w = 1.0;
    std::string base = ament_index_cpp::get_package_share_directory("handyman") + "/maps/";
    // 已有的地图
    registerEnvironment("2019HM01", base + "2019HM01.yaml", p);
    // 添加新地图（环境名需与仿真器发送的 Environment 消息一致）
    registerEnvironment("MyMap", base + "MyMap.yaml", p);
}
```

修改后**重新编译**：

```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
```

---

### 第二步：正式运行比赛任务

**终端 1 — 启动 SIGVerse 仿真器**

按照 SIGVerse 的文档启动仿真环境，确认机器人处于初始位置。

**终端 2 — 启动 ROS2 程序：**

```bash
ros2 launch handyman hsr_nav.launch.py
```

此命令同时启动以下 4 个节点：

| 节点 | 功能 |
|------|------|
| `handyman_sample` | 主控状态机，自动执行搬运任务 |
| `object_detection_node` | YOLOv12 视觉检测（手部相机） |
| `rviz2` | 可视化界面，显示地图和导航路径 |
| `rosbridge_websocket` | 与 SIGVerse 通信（端口 9090） |

**启动顺序和预期日志：**

```
[handyman_sample] Handyman sample start!
[handyman_sample] Waiting for NavigateToPose action server...   ← Nav2 初始化中，正常等待
[handyman_sample] System ready, sending I_am_ready             ← 就绪，等待仿真器指令
```

然后在 SIGVerse 中开始任务，仿真器会发送 `Are_you_ready?` 消息，程序自动开始执行。

---

### 其他运行模式

**仅启动主控节点（不含视觉，用于调试）：**

```bash
ros2 launch handyman handyman.launch.py
```

**键盘遥控（手动操控机器人）：**

```bash
ros2 launch handyman teleop_key.launch.py
```

启动后会弹出 xterm 窗口，在该窗口中按键操作。详见 [键盘遥控说明](#键盘遥控说明)。

**键盘遥控 + RViz2（带可视化）：**

```bash
ros2 launch handyman teleop_key_with_rviz.launch.py
```

---

## 键盘遥控说明

| 按键 | 功能 |
|------|------|
| ↑ ↓ ← → | 前进 / 后退 / 左转 / 右转 |
| 空格 | 停止底盘 |
| `u` `i` `o` | 左前 / 正前 / 右前 移动 1m |
| `j` `k` `l` | 向左 / 停止 / 向右 移动 1m |
| `m` `,` `.` | 左后 / 正后 / 右后 移动 1m |
| `q` / `z` | 加速 / 减速（速度倍率 0.125x ~ 2x） |
| `y` / `h` / `n` | 躯干升高 / 停止 / 降低 |
| `a` | 手臂竖直（arm_flex=0, wrist=-1.57） |
| `b` | 手臂斜向上（arm_flex=-0.785, wrist=-0.785） |
| `c` | 手臂水平（arm_flex=-1.57, wrist=0） |
| `d` | 手臂向下（arm_flex=-2.2, wrist=0.35） |
| `g` | 切换抓取 / 松开夹爪 |
| `0` | 发送 `I_am_ready` |
| `1` | 发送 `Room_reached` |
| `2` | 发送 `Object_grasped` |
| `3` | 发送 `Task_finished` |
| `6` | 发送 `Does_not_exist` |
| `9` | 发送 `Give_up` |

---

## 常见问题

**Q: `colcon build` 报错 `package 'handyman' not found`**  
A: 确认包已按正确名称复制：源目录 `handyman-ros2` 必须复制为 `~/ros2_ws/src/handyman`。

**Q: `colcon build` 报错 `ament_index_cpp not found`**  
A: 执行 `sudo apt install ros-humble-ament-index-cpp`。

**Q: 启动后持续输出 `Waiting for NavigateToPose action server`**  
A: Nav2 初始化需要时间，正常等待 10~30 秒。如果超过 60 秒仍未就绪，检查 `nav2_params.yaml` 是否正确安装（执行 `colcon build` 后是否 source）。

**Q: 视觉节点报错 `Model file not found` 或自动下载 yolo12n.pt**  
A: 检查 `~/ros2_ws/src/rcup_vision/models/last.pt` 是否存在，且放置后需重新 `colcon build`。

**Q: RViz2 显示 `No map received`**  
A: `hsr_nav.launch.py` 不负责启动 map_server，地图由 `handyman_sample` 在收到 Environment 消息后通过 `LoadMapManager` 动态加载。若未收到 Environment 消息，地图不会加载。

**Q: 仿真器连不上 ROS2 / rosbridge**  
A: 确认 `rosbridge_websocket` 已启动（端口默认 9090），SIGVerse 中配置的 IP 和端口与之一致。可用 `ros2 node list | grep rosbridge` 确认节点在线。

**Q: xterm 窗口没有弹出（键盘遥控无响应）**  
A: 确认已安装 xterm：`sudo apt install xterm`。

**Q: 每次新开终端命令找不到**  
A: 执行 `source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash`，或参考[编译](#编译)章节把这两行加入 `~/.bashrc`。

---

## 包结构

```
handyman-ros2/          （复制到工作空间时命名为 handyman）
├── src/
│   ├── handyman_sample.cpp         # 主状态机节点
│   └── teleop_key_handyman.cpp     # 键盘遥控节点
├── launch/
│   ├── hsr_nav.launch.py           # 正式运行（主控+视觉+rviz2+rosbridge）
│   ├── handyman.launch.py          # 仅主控节点+rosbridge
│   ├── make_map.launch.py          # 建图（slam_toolbox+teleop+rviz2）
│   ├── teleop_key.launch.py        # 仅键盘遥控
│   └── teleop_key_with_rviz.launch.py
├── maps/
│   ├── 2019HM01.pgm / .yaml       # 场地地图（4个场地）
│   ├── 2019HM02.pgm / .yaml
│   ├── 2020HM01.pgm / .yaml
│   └── 2021HM01.pgm / .yaml
├── param/
│   └── nav2_params.yaml            # Nav2 导航参数（DWB规划器）
├── msg/
│   └── HandymanMsg.msg             # 自定义消息（message + detail 字段）
├── CMakeLists.txt
└── package.xml
```

---

## 话题对照表

| 功能 | 话题名 |
|------|--------|
| 主控消息（接收自仿真器） | `/handyman/message/to_robot` |
| 主控消息（发送到仿真器） | `/handyman/message/to_moderator` |
| 底盘速度控制 | `/hsrb/command_velocity` |
| 底盘轨迹控制（全向移动） | `/hsrb/omni_base_controller/command` |
| 手臂关节轨迹 | `/hsrb/arm_trajectory_controller/command` |
| 夹爪控制 | `/hsrb/gripper_controller/command` |
| 关节状态反馈 | `/hsrb/joint_states` |
| 激光雷达 | `/hsrb/base_scan` |
| 检测目标设定 | `/detection_target` |
| 手部相机检测结果 | `/hand_detection` |
| 视觉目标位姿 | `/vision` |
| AMCL 初始位姿 | `/initialpose` |
| Nav2 导航动作 | `navigate_to_pose` (action) |
 