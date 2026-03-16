# interactive_cleanup_ros2

RoboCup@Home Simulation — **Interactive Cleanup** 任务控制器（ROS 2 Humble）。根据 Avatar 的肢体指向识别待清理物体与目的地，控制 HSR 完成抓取与放置。

## 依赖

- ROS 2 Humble
- 工作空间已编译：`cleanup_vision_ros2`、`interactive_cleanup`、`sigverse_ros_bridge`
- 全栈启动还需：Python 依赖（含 `cleanup_vision_ros2` 的 ultralytics、mediapipe 等），建议使用项目根目录的 pixi 环境

## 启动方式

### 1. 全栈启动（推荐，含视觉）

同时启动控制器、cleanup 视觉节点、SIGVerse 桥接与 rosbridge，用于正式比赛或联调：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# 若使用 pixi 管理 Python 依赖，先进入：pixi shell
ros2 launch interactive_cleanup cleanup.launch.py
```

可选参数示例：

```bash
ros2 launch interactive_cleanup cleanup.launch.py use_sim_time:=true
```

### 2. 仅启动控制器

不启动视觉节点，用于仅测试状态机或与官方 sample 对比：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch interactive_cleanup sample.launch.py
```

## 相关话题

- 订阅：`/interactive_cleanup/message/to_robot`（比赛消息）、`/cleanup_vision/detected_objects`、`/cleanup_vision/pointing_direction`
- 发布：`/interactive_cleanup/message/to_moderator`、`/hsrb/command_velocity`、`/hsrb/arm_trajectory_controller/command`、`/hsrb/gripper_controller/command`、`/cleanup_vision/enable`

## 参考

- 竞赛流程与消息含义见：`rule_reference/interactive_cleanup/`
- 视觉实现见：`src/cleanup_vision_ros2/`
