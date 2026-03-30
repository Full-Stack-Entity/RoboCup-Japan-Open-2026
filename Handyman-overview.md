# Handyman Overview

##启动测试
模块化的版本：./run_and_log.sh "pixi run ros2 launch handyman_ros2 hsr_nav_orchestrated.launch.py"
原来版本：./run_and_log.sh "pixi run ros2 launch handyman_ros2 hsr_nav.launch.py"

## 文档目的

这份文档是给后续开发者、后续 Codex 会话、以及可能的新进程做交接用的总览文档，目标不是替代源码，而是让接手者能在较短时间内同时弄清楚下面几件事：

- 比赛本身要求机器人完成什么；
- 官方系统架构、消息流和 HSR 接口是什么；
- 当前仓库里 `handyman_ros2` 与 `handyman_vision_ros2` 的真实实现结构是什么；
- 用户构想的全任务流程中，哪些已经落地，哪些还是方向；
- 当前系统已经推进到哪里，主要风险和瓶颈又在哪里；
- 如果对文档内容有怀疑，应当回到哪些本地文件或官方资料核实。

维护原则：

- 只要 `src/handyman_ros2`、`src/handyman_vision_ros2`、launch 文件、消息定义、状态机流程、目的地数据库、或相关设计结论发生变化，这份文档就应当一起更新；
- 文档与代码冲突时，以代码为准，然后修正文档；
- 文档中凡是带有"推断"的内容，表示它不是官方文字原样结论，而是根据当前源码或资料归纳得到。

---

## ⚠️ 全任务构想阶段（待实现）

> 以下内容是**用户提出的全任务构想**，尚未在代码中完整实现。放在文档最前面，作为未来开发的导航目标。所有构想实现前均应先验证可行性。

### 构想总览

```
┌─────────────────────────────────────────────────────────────┐
│  构想全任务流程（待实现）                                    │
│                                                             │
│  1. 载入地图 ──→ 2. 判断是否在目标房间 ──→ 3. 导航模块      │
│                                           ↓                │
│  4. 旋转扫描 ──→ 5. 视觉识别模块 ──→ 6. 移动到物品0.5m     │
│                                           ↓                │
│  7. 机械臂模块（抓取）──→ 8. 导航到放置位置0.5m ──→ 9. 机械臂放置 │
└─────────────────────────────────────────────────────────────┘
```

### 构想1：房间到达判断标准

**构想内容**：按房间中心点边长为 2m 的矩形区域作为到达目标位置的判断标准。

**构想来源**：用户任务设想，`.cursorrules` 导航优化方案。

**实现状态**：⚠️ 尚未实现。当前代码使用 0.5m 阈值判断机器人是否在 patrol waypoint 附近，**与构想不符**。

**参考现有实现**：
- `handyman_sample.cpp` 中 `roomLocation()` 定义了各房间的 patrol waypoint
- `src/handyman_ros2/src/room_navigation.cpp` 中定义了 patrol 点结构
- 构想房间边界判断可参考 `.cursorrules` 中的 `RoomBoundary` 数据结构（未实现）

**TODO**：
1. 在 `room_navigation.cpp` 或 `map_nav_controller.cpp` 中定义各房间的 2m 边长矩形边界
2. 实现 `isRobotInRoom(const geometry_msgs::msg::Pose& robot_pose, const RoomBoundary& room)` 函数
3. 在 GoToRoom1 状态入口替换现有阈值判断逻辑

### 构想2：导航模块借助 PGM 文件进行路径规划和 Nav2 验证

**构想内容**：机器人导航时，使用 PGM 静态地图进行全局路径规划，并通过 Nav2 验证路径可行性。

**构想来源**：用户任务设想。

**实现状态**：⚠️ 已在基础层面实现，但与构想描述有偏差。

**当前实现情况**：
- `LoadMapManager` 在收到 `Environment` 后加载对应地图 YAML/PGM 文件
- `Nav2`（controller_server + planner_server）使用 `map_server` 发布的静态地图做全局规划
- `AMCL` 负责定位
- 导航通过 Nav2 action server `navigate_to_pose` 驱动

**与构想的偏差**：
- 现有实现中，Nav2 全局规划使用静态 PGM 地图 ✅ 已实现
- 但 PGM 地图的墙壁预检功能（WallMapAnalyzer）**尚未实现**（见下方"暂时跳过的功能"）
- 当前没有在导航前用 PGM 做"墙壁预检"的额外验证步骤

**TODO**：
1. 重构后的 `MapNavController` 中已预留 `isPathBlockedByWall()` 接口（当前返回 false）
2. 待 `WallMapAnalyzer` 重新集成后，实现直线段与墙壁交点检测

### 构想3：旋转扫描 + 视觉识别模块

**构想内容**：
- 到达目标房间后，启动原地 360° 旋转扫描
- 同时启动视觉识别模块（YOLO），检测目标物品是否存在
- 扫描到物品后停止旋转，移动机器人到物品位置 0.5m 处

**构想来源**：用户任务设想 + `.cursorrules` Phase 3 导航优化方案。

**实现状态**：⚠️ 部分实现。

**当前实现情况**：
- `MoveToInFrontOfTarget` 状态包含原地旋转扫描逻辑
- `VisionManipulationController` 包含房间扫描旋转（`startRoomScan()`）
- 扫描过程中持续检测 `/hand_detection`（YOLO bbox 结果）
- 检测到物体后切换到对准模式（`ALIGNING`）
- 对准完成后进入 `Grasp` 状态

**当前与构想的偏差**：
- 现有扫描使用头部相机/手部相机的 YOLO 检测 ✅ 已实现
- 构想要求"移动到物品位置 0.5m"——现有实现在对准完成后会微微前进 ✅ 部分实现
- 构想描述的"360° 旋转搜索"与现有 `ROOM_SCAN` 状态一致 ✅ 已实现
- **构想的系统性搜索**（每个 patrol waypoint 都做旋转搜索）尚未完整实现

**TODO**：
1. 确认 YOLO 模型是否已安装（`handyman_vision_ros2/scripts/object_detection_node.py`）
2. 当前测试模式（`TEST_MODE_ENABLED`）可辅助验证导航逻辑
3. 验证 YOLO 检测结果与 360° 扫描的协同

### 构想4：机械臂模块（抓取）

**构想内容**：视觉对准完成后，启动机械臂模块进行抓取。抓取后必须订阅 `/hsrb/joint_states` 验证夹爪状态。

**构想来源**：用户任务设想 + `.cursorrules` 强制约束第 3 条。

**实现状态**：✅ 已实现（需 YOLO 安装后验证）。

**当前实现情况**：
- `VisionManipulationController::closeGripper()` 发送夹爪闭合命令
- `VisionManipulationController::verifyGrasp()` 订阅 `/hsrb/joint_states`，读取 `hand_motor_joint` 实际位置
- 差值 > 0.03 时触发重试（最多 2 次）
- 验证成功后发送 `Object_grasped`

**`.cursorrules` 强制约束**：
> 抓取后必须订阅 `/hsrb/joint_states` 并验证夹爪状态——读取 `hand_motor_joint` 实际位置，与目标位置差 > 0.03 时触发重试（最多 2 次），不得跳过验证直接上报 `Object_grasped`。

### 构想5：导航到放置位置 + 机械臂放置

**构想内容**：
- 抓取成功后，导航到放置位置，到达距离目标 0.5m 处停下
- 启动机械臂模块进行放置

**构想来源**：用户任务设想。

**实现状态**：⚠️ 部分实现。

**当前实现情况**：
- `GoToRoom2` 状态负责导航到放置目标
- `destLocation()` 提供各环境下的放置点坐标
- `Release` 状态执行放置序列：先伸手臂 → 等 4 秒 → 张开夹爪 → 等 8 秒
- `placement_navigation.cpp` 中定义了放置点坐标

**与构想的偏差**：
- 构想中"到达 0.5m 停下"——当前实现是 Nav2 导航到目标点（实际到达精度取决于 Nav2）
- 构想中"启动机械臂放置"——现有 `Release` 状态有简单放置序列，但无放置成功验证

### 构想6：整体流程串联

**构想完整流程**：

```
Avatar → Are_you_ready? + Environment
    ↓
Robot → I_am_ready
    ↓
Avatar → Instruction（目标物品 + 放置位置）
    ↓
[地图加载] LoadMapManager 切换环境 + 启动 Nav2/AMCL
    ↓
[房间判断] 机器人是否已在目标房间？（2m矩形边界判断）
    ├─ 已在 → 发送 Room_reached → 跳过导航
    └─ 不在 → Nav2 导航到目标房间 patrol waypoint
    ↓
发送 Room_reached
    ↓
[旋转扫描] 原地 360° 旋转 + YOLO 视觉识别
    ├─ 找到物品 → 对准 → 移动到 0.5m → 机械臂抓取
    │            └─ 夹爪验证（joint_states）→ Object_grasped
    └─ 未找到 → Does_not_exist → 等待 Corrected_instruction
    ↓
[放置导航] Nav2 导航到放置位置（0.5m 阈值待确认）
    ↓
[机械臂放置] 放置序列 → Task_finished
    ↓
Avatar → Task_succeeded / Task_failed
```

---

## ⚠️ 暂时跳过的功能（Test Mode 相关）

> 以下功能在当前开发阶段被跳过/模拟，以优先验证导航逻辑。**安装 YOLO 后必须恢复完整实现**。

### 1. YOLO 视觉识别（最高优先级恢复）

**问题**：YOLO 视觉识别模型尚未安装，`object_detection_node.py` 无法提供真实检测结果。

**当前行为**：测试模式（`TEST_MODE_ENABLED = false`，默认关闭）下，若无检测结果，扫描超时后直接跳转到 `GoToRoom2`。

**恢复步骤**：
1. 在 `handyman_vision_ros2` 中配置 YOLO 模型权重
2. 将 `TEST_MODE_ENABLED` 改为 `false`
3. 删除测试模式相关代码块（见 `.cursorrules` "[测试模式]" 章节）
4. 验证 `/hand_detection` 能正常输出 bbox

### 2. WallMapAnalyzer 墙壁预检

**问题**：原 `LoadMapManager` 中的 `WallMapAnalyzer` 在提取到独立文件后未正确集成。

**当前行为**：`MapNavController::isPathBlockedByWall()` 返回 `false`（不做墙壁预检）。

**恢复步骤**：
1. 在 `load_map_manager.cpp` 中实现 `WallMapAnalyzer` 类
2. 确保 `loadFromYaml()` 正确加载 PGM 文件（yaml 文件所在目录查找 PGM）
3. 实现 `lineIntersectsWall()` 方法
4. 在 `MapNavController` 中调用墙壁预检

### 3. `Does_not_exist` / `Corrected_instruction` 分支

**问题**：根据 `.cursorrules` 强制约束第 2 条，这两个分支必须完整实现。

**当前行为**：`MoveToInFrontOfTarget` 扫描超时后直接跳转 `GoToRoom2`，未发送 `Does_not_exist`。

**恢复步骤**：在 `MoveToInFrontOfTarget` 状态中，扫描完成且未检测到物体时：
1. 发送 `MSG_DOES_NOT_EXIST`
2. 等待 Avatar 回复 `Corrected_instruction`
3. 若收到 `Corrected_instruction`，重置扫描状态重新搜索

### 4. `Give_up` / `Time_is_up` 分支

**问题**：根据 `.cursorrules` 强制约束第 4 条，`is_failed_` 必须走 `GiveUp` 状态。

**当前行为**：`give_up_sent_` 标志位已实现，`GiveUp` 状态发送 `MSG_GIVE_UP`。

**恢复步骤**：确保 `is_failed_` 在任何情况下都通过 `GiveUp` 状态跳转，而不是直接跳转到 `Initialize`。

### 5. `nav_goal_handle_` 重置

**问题**：根据 `.cursorrules` 强制约束第 5 条，每次任务重置时必须 `.reset()`。

**当前行为**：已实现（在 `LoadMapManager::resetNavGoalHandle()` 及相关位置）。

---

## 资料来源与可信度

本总览基于以下资料整理，按"优先级"理解：

### 官方/半官方资料

- `rule_reference/handyman/SystemOverview · RoboCupatHomeSimhandyman-unity2 Wiki.md`
- `rule_reference/handyman/RosMessage Handyman · RoboCupatHomeSimros2-competition-msgs Wiki.md`
- 在线 wiki:
  - <https://github.com/RoboCupatHomeSim/handyman-unity2/wiki/SystemOverview>
- HSR 描述仓库:
  - <https://github.com/ToyotaResearchInstitute/hsr_description>
- HSR ROS 接口:
  - <https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HSR>

### 当前仓库里的真实实现（模块化版本）

- `src/handyman_ros2/src/task_orchestrator_main.cpp` — 模块化入口
- `src/handyman_ros2/src/task_orchestrator.cpp` — 状态机主循环
- `src/handyman_ros2/src/protocol_handler.cpp` — 通信协议
- `src/handyman_ros2/src/map_nav_controller.cpp` — 导航封装
- `src/handyman_ros2/src/vision_manipulation_controller.cpp` — 视觉/机械臂
- `src/handyman_ros2/src/load_map_manager.cpp` — 地图加载
- `src/handyman_ros2/src/room_navigation.cpp` — patrol waypoint
- `src/handyman_ros2/src/placement_navigation.cpp` — 放置点坐标
- `src/handyman_ros2/src/handyman_sample.cpp` — 原始版本（回退用）
- `src/handyman_ros2/src/teleop_key_handyman.cpp` — 键盘遥控
- `src/handyman_ros2/launch/hsr_nav_orchestrated.launch.py` — 模块化全栈启动
- `src/handyman_ros2/launch/hsr_nav.launch.py` — 原始版本启动
- `src/handyman_ros2/param/nav2_params.yaml`
- `src/handyman_vision_ros2/scripts/object_detection_node.py`
- `src/handyman_vision_ros2/launch/vision.launch.py`

### 需要特别注意

- 仓库内部分文档存在"旧命名/旧赛季/旧能力描述"残留。
- 以后分析行为时，应优先相信源码，其次相信 `rule_reference`，最后再看 README。

---

## 项目一句话总结

当前 `Handyman` 系统是一个基于 ROS 2 Humble 的 HSR 仿真控制器：

- 通过 `sigverse_ros_bridge + rosbridge_server` 与 Unity/SIGVerse 比赛环境通信
- 通过 `handyman_msgs/msg/HandymanMsg` 与 Avatar/Moderator 交换比赛事件消息
- 通过 `Nav2 + AMCL + map_server` 在不同布局地图间切换并导航
- 通过一个 Python YOLO 节点从 HSR 手部相机上做目标检测（待安装）
- 由一个 C++ 单节点状态机串起"收指令 -> 去目标房间 -> 搜物体 -> 靠近抓取 -> 去目标位置 -> 放置 -> 上报完成"

从业务上看，它已经具备一个可跑通的端到端骨架，但距离一个完整、稳健、规则覆盖充分的比赛系统还有明显差距。

---

## 官方比赛视角下的 Handyman 任务

根据官方 SystemOverview 和 RosMessage Handyman，官方定义的流程如下：

1. Ubuntu 侧启动机器人控制器、SIGVerse rosbridge 等
2. Windows 侧启动 Handyman Unity app
3. 初始化机器人与目标物体位置
4. Avatar 发送 `Are_you_ready?`，同时发送 `Environment`
5. 机器人回复 `I_am_ready`
6. Avatar 发送 `Instruction`
7. 机器人前往指定房间
8. 机器人发送 `Room_reached`
9. Avatar 检查第一阶段是否成功（成功得分，失败任务结束）
10. 机器人寻找目标物体
    - 若能找到 → 继续抓取
    - 若判断物体不存在 → 发送 `Does_not_exist`
      - Avatar 确认物体存在 → 发送 `Corrected_instruction`，机器人搜索新物体
      - Avatar 确认不存在 → 任务结束
11. 机器人抓取物体
12. 机器人发送 `Object_grasped`
13. Avatar 检查抓取是否正确（成功得分，失败任务结束）
14. 机器人执行抓取后的搬运/放置任务
15. 机器人发送 `Task_finished`
16. Avatar 检查最终任务是否完成（成功得分，进入下一题）
17. Avatar 发送 `Task_succeeded` / `Task_failed` / `Mission_complete`

**特殊分支**：
- 机器人可主动发送 `Give_up`，Avatar 收到后回复 `Task_failed`
- 超时收到 `Task_failed`，detail 为 `Time_is_up`

**放置目标规则**（官方明确）：
- 基本规则：将物体放到桌面或类似位置
- 特殊规则：若目标是人，将手臂移到使物体中心处于球形区域内

---

## 官方消息接口

### HandymanMsg

```text
string message
string detail
```

### 比赛事件 topic

| Topic | 方向 | 消息类型 | 说明 |
|-------|------|----------|------|
| `/handyman/message/to_robot` | Avatar → Robot | `HandymanMsg` | 接收 `Are_you_ready?` / `Environment` / `Instruction` / `Task_*` |
| `/handyman/message/to_moderator` | Robot → Avatar | `HandymanMsg` | 回传 `I_am_ready` / `Room_reached` / `Object_grasped` / `Task_finished` |

### 完整事件消息表

| No | 事件 | 方向 | message | detail |
|----|------|------|---------|--------|
| 1 | 发送 Are_you_ready? | Avatar ⇒ | `Are_you_ready?` | (blank) |
| 2 | 发送 Environment info | Avatar ⇒ | `Environment` | 环境名（如 LayoutA）或 "unknown" |
| 3 | 发送 I_am_ready | Robot ⇒ | `I_am_ready` | (blank) |
| 4 | 发送任务消息 | Avatar ⇒ | `Instruction` | 任务描述（如 "Go to the XXXX, grasp the YYYY and bring it here."） |
| 5 | 发送 Room_reached | Robot ⇒ | `Room_reached` | (blank) |
| 6 | Room_reached 失败 | Avatar ⇒ | `Task_failed` | `Room_reached` |
| 7 | 发送 Does_not_exist | Robot ⇒ | `Does_not_exist` | (blank) |
| 8 | Does_not_exist 失败 | Avatar ⇒ | `Task_failed` | `The target exist` |
| 9 | 发送纠正指令 | Avatar ⇒ | `Corrected_instruction` | 新的任务描述 |
| 10 | 发送 Object_grasped | Robot ⇒ | `Object_grasped` | (blank) |
| 11 | Object_grasped 失败 | Avatar ⇒ | `Task_failed` | `Object_grasped` |
| 12 | Object_grasped 失败（目标不存在） | Avatar ⇒ | `Task_failed` | `The target doesn't exist` |
| 13 | 发送 Task_finished | Robot ⇒ | `Task_finished` | (blank) |
| 14 | Task_finished 失败 | Avatar ⇒ | `Task_failed` | `Task_finished` |
| 15 | 任务成功 | Avatar ⇒ | `Task_succeeded` | (blank) |
| 16 | 所有任务完成 | Avatar ⇒ | `Mission_complete` | (blank) |
| 17 | 发送 Give_up | Robot ⇒ | `Give_up` | (blank) |
| 18 | 放弃 | Avatar ⇒ | `Task_failed` | `Give_up` |
| 19 | 时间到 | Avatar ⇒ | `Task_failed` | `Time_is_up` |

信息来源：`rule_reference/handyman/RosMessage Handyman · RoboCupatHomeSimros2-competition-msgs Wiki.md`

---

## HSR 机器人接口

### 控制接口

| No | Topic | Direction | Message Type | Description |
|----|-------|-----------|--------------|-------------|
| 1 | `/hsrb/command_velocity` | ROS ⇒ | `geometry_msgs/Twist` | 底盘运动控制 |
| 2 | `/hsrb/head_trajectory_controller/command` | ROS ⇒ | `trajectory_msgs/JointTrajectory` | 头部关节（head_pan_joint, head_tilt_joint） |
| 3 | `/hsrb/arm_trajectory_controller/command` | ROS ⇒ | `trajectory_msgs/JointTrajectory` | 机械臂关节 |
| 4 | `/hsrb/gripper_controller/command` | ROS ⇒ | `trajectory_msgs/JointTrajectory` | 夹爪关节 |

### 状态与传感器

| No | Topic | Direction | Description |
|----|-------|-----------|-------------|
| 5 | `/hsrb/joint_states` | SIGVerse ⇒ | 关节状态（用于抓取验证） |
| 6 | `/hsrb/base_scan` | SIGVerse ⇒ | 激光雷达（用于 AMCL 定位） |
| 7 | `/hsrb/head_rgbd_sensor/rgb/image_raw` | SIGVerse ⇒ | 头部 RGB 相机（暂未用于主流程） |
| 8 | `/hsrb/head_rgbd_sensor/depth_registered/image_raw` | SIGVerse ⇒ | 头部深度相机（暂未用于主流程） |
| 9 | `/hsrb/hand_camera/image_raw` | SIGVerse ⇒ | 手部相机（当前用于 YOLO 目标检测） |
| 13 | `/hsrb/hand_camera/camera_info` | SIGVerse ⇒ | 手部相机内参 |

信息来源：<https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HSR>

### HSR 关节名

机械臂：`arm_lift_joint`、`arm_flex_joint`、`arm_roll_joint`、`wrist_flex_joint`、`wrist_roll_joint`  
夹爪：`hand_motor_joint`（当前代码使用此命名，与官方 `hand_l_proximal_joint` / `hand_r_proximal_joint` 有偏差）

### topic 命名偏差

HSR Gazebo 默认值与当前代码实际使用的话题名存在偏差，说明当前工程依赖 SIGVerse/比赛环境自己的 topic 映射，而不是直接照搬 `hsr_description` 的 Gazebo 默认话题。

---

## 当前仓库中的相关包

### `src/handyman_ros2`

这是主控包，核心内容如下：

**主入口（两套并存）**：
- `src/task_orchestrator_main.cpp` — 模块化版本入口（当前推荐）
- `src/handyman_sample.cpp` — 原始单文件版本（保留回退）

**重构模块（4 个，已编译为 `handyman_modules` 静态库）**：
- `src/load_map_manager.cpp` → `handyman_map_lib`（地图加载与 Nav2 生命周期）
- `src/protocol_handler.cpp` → `handyman_modules`（与 Avatar/Moderator 通信协议）
- `src/map_nav_controller.cpp` → `handyman_modules`（导航封装与房间 patrol）
- `src/vision_manipulation_controller.cpp` → `handyman_modules`（视觉对准与机械臂控制）
- `src/task_orchestrator.cpp` → `handyman_modules`（任务编排层，串联各模块）

**辅助文件**：
- `src/room_navigation.cpp` — patrol waypoint 定义（编译进 `handyman_nav_lib`）
- `src/placement_navigation.cpp` — 放置点坐标定义（编译进 `handyman_nav_lib`）

**Launch 文件**：
- `launch/hsr_nav_orchestrated.launch.py` — **当前推荐**：模块化全栈（rosbridge + sigverse_bridge + RViz + 视觉 + task_orchestrator）
- `launch/hsr_nav.launch.py` — 原始全栈（rosbridge + sigverse_bridge + RViz + 视觉 + handyman_sample）
- `launch/handyman.launch.py` — 不带视觉和 RViz 的轻量启动
- `launch/task_orchestrator.launch.py` — 仅 task_orchestrator（无视觉、RViz）
- `launch/amcl.launch.py` — AMCL
- `launch/move_base.launch.py` — Nav2 controller/planner/behavior/bt_navigator

**配置**：
- `maps/*.yaml` — 各布局静态地图（`2019HM01`、`2019HM02`、`2020HM01`、`2021HM01`）
- `param/nav2_params.yaml` — Nav2 参数

### `src/handyman_vision_ros2`

**⚠️ YOLO 模型待安装**，核心内容如下：

- `scripts/object_detection_node.py`
  - Python 检测节点，使用 Ultralytics YOLO
  - 目前订阅手部相机 `/hand_camera/image_raw`
  - 发布 `/hand_detection`（bbox xywh）
  - 发布 `/vision`（目标位姿，暂未用于主流程）
- `launch/vision.launch.py` — 启动检测节点
- `models/` — 模型目录（当前为空，待放置 YOLO 权重）

---

## 整体系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│                        Windows                                    │
│                    Handyman Unity App                            │
│         (Avatar / 场景 / HSR 仿真 / 传感器/关节数据)               │
└─────────────────────┬────────────────────────────────────────────┘
                      │ SIGVerse rosbridge（大数据量）
                      │ rosbridge_websocket（轻量控制）
┌─────────────────────▼────────────────────────────────────────────┐
│                        Ubuntu                                     │
│                                                                   │
│  ┌────────────────────────┐    ┌──────────────────┐    ┌────────────────┐ │
│  │ task_orchestrator      │    │  Nav2 Stack      │    │ handyman_      │ │
│  │ (模块化状态机)          │───▶│  map_server     │    │ vision_ros2    │ │
│  │                        │    │  amcl           │    │ object_        │ │
│  │  ┌──────────────────┐  │    │  controller     │    │ detection_node │ │
│  │  │ ProtocolHandler   │  │    │  planner        │    │ (YOLO)         │ │
│  │  ├──────────────────┤  │    └────────▲─────────┘    └───────┬────────┘ │
│  │  │MapNavController  │  │             │                   │           │
│  │  ├──────────────────┤  │             │   /hand_detection │           │
│  │  │VisionManipCtrl   │  │             │◀──────────────────│           │
│  │  └──────────────────┘  │             │                   │           │
│  └──────┬───────────────────┘             │                   │           │
│         │   /handyman/message             │                   │           │
│         │◀────────────────────────────────────────────────────│           │
└─────────┼─────────────────────────────────────────────────────┼───────────┘
          │                                                       │
  ┌───────▼───────────────────────────────────────────────────────▼───────────┐
  │              handyman_msgs / HandymanMsg                                     │
  │         /handyman/message/to_robot & /to_moderator                        │
  └───────────────────────────────────────────────────────────────────────────┘
```

**模块化版本（`task_orchestrator`）** 调用链：
```
task_orchestrator_main.cpp
 └─ TaskOrchestrator（状态机主循环）
     ├─ ProtocolHandler          ← 通信协议
     ├─ MapNavController         ← 导航 + patrol + LoadMapManager
     └─ VisionManipulationController ← 视觉对准 + 机械臂控制
```

**原始版本（`handyman_sample`，保留回退）** 调用链：
```
handyman_sample.cpp（单文件，所有逻辑内联）
 ├─ LoadMapManager（内联）
 ├─ Nav2 action（内联）
 ├─ 视觉对准（内联）
 └─ 机械臂控制（内联）
```

---

## 当前启动方式

### 方式一：模块化版本（当前推荐）
```bash
./run_and_log.sh "pixi run ros2 launch handyman_ros2 hsr_nav_orchestrated.launch.py"
```
启动可执行文件：`task_orchestrator`（模块化状态机）
调用的代码：**全部为分模块后的代码**，由 `task_orchestrator_main.cpp` 入口持有并协调：
- `ProtocolHandler` — 与 Avatar/Moderator 通信协议
- `MapNavController` — 导航封装与房间 patrol
- `VisionManipulationController` — 视觉对准与机械臂控制
- `LoadMapManager`（通过 `MapNavController` 间接调用）

### 方式二：原始版本（保留回退）
```bash
./run_and_log.sh "pixi run ros2 launch handyman_ros2 hsr_nav.launch.py"
```
启动可执行文件：`handyman_sample`（单文件状态机，所有逻辑内联）

其他可用 launch：
```bash
ros2 launch handyman_ros2 handyman.launch.py    # 轻量：rosbridge + sigverse_bridge（无视觉、RViz）
ros2 launch handyman_ros2 task_orchestrator.launch.py  # 仅 task_orchestrator（无视觉、RViz）
```

**注意**：`map_server`、`amcl`、`Nav2` 不在 launch 里一次性常驻启动，当前主控在收到环境并进入 ready 流程后，通过 `system()` 调 shell 命令动态启动/停止这些导航组件。

---

## 主控节点 `task_orchestrator`（模块化版本，当前推荐）

核心文件：`src/handyman_ros2/src/task_orchestrator.cpp` + `src/task_orchestrator_main.cpp`

> 注意：原始版本 `handyman_sample.cpp` 仍保留于 `hsr_nav.launch.py` 中作为回退。

### 内部状态

| 状态 | 说明 | 官方对应 |
|------|------|----------|
| `Initialize` | 初始化 | — |
| `Ready` | 等待 `Are_you_ready?` 确认 | 收到后回复 `I_am_ready` |
| `WaitForInstruction` | 等待 `Instruction` | 缓存指令文本 |
| `GoToRoom1` | 导航到目标房间 patrol | 到达后发送 `Room_reached` |
| `MoveToInFrontOfTarget` | 原地旋转扫描 + 对准 | 检测到物体对准后进入抓取 |
| `Grasp` | 抓取 + 夹爪验证 | 成功后发送 `Object_grasped` |
| `GoToRoom2` | 导航到放置位置 | — |
| `Release` | 放置序列 | 成功后发送 `Task_finished` |
| `TaskFinished` | 等待 Avatar 确认 | 收到 `Task_succeeded` 后结束 |
| `GiveUp` | 放弃流程 | 发送 `Give_up`，等待 `Task_failed` |

> ⚠️ 以下官方分支当前**已实现**（根据 `.cursorrules` 强制约束）：
> - `Does_not_exist` / `Corrected_instruction` — ⚠️ 部分实现（见"暂时跳过的功能"）
> - `Give_up` / `Time_is_up` — ✅ 已实现（`give_up_sent_` 标志位）
> - 夹爪抓取验证（`joint_states` 订阅）— ✅ 已实现

### 消息回调处理

| 接收消息 | 当前处理 | 实现状态 |
|----------|----------|----------|
| `Environment` | 记录环境名（映射 LayoutX → LayoutYYYYMMZZ） | ✅ |
| `Are_you_ready?` | 合适状态下置 `is_started_ = true` | ✅ |
| `Instruction` | 仅在 `WaitForInstruction` 下缓存并关键词解析 | ✅ |
| `Corrected_instruction` | ⚠️ 尚未完整处理 | 待完善 |
| `Task_succeeded` | 在 `TaskFinished` 下标记结束 | ✅ |
| `Task_failed` | ⚠️ `is_failed_ = true`，走 `GiveUp` 状态 | ✅ |
| `Mission_complete` | 直接退出进程 | ✅ |

### Environment 名称映射

| Unity 环境名 | 内部环境名 | 地图文件 |
|-------------|-----------|----------|
| `LayoutA` | `Layout2019HM01` | `2019HM01.yaml` |
| `LayoutB` | `Layout2019HM02` | `2019HM02.yaml` |
| `LayoutC` | `Layout2020HM01` | `2020HM01.yaml` |
| `LayoutD` | `Layout2021HM01` | `2021HM01.yaml` |

---

## 地图与导航机制

### `LoadMapManager` 职责

环境切换时重建导航系统：

1. 停掉旧的 `map_server` / `amcl` / Nav2 节点（`system("pkill ...") &`）
2. 启动新的 `map_server`
3. 通过 lifecycle 切换 map_server 到 active
4. 启动 `amcl`
5. 发布 `/initialpose`
6. 启动 Nav2
7. 清空 costmap
8. 检查 `/map` 和 TF 是否可用

### 工程特点

- 不是通过 launch API 管理，而是通过 `system("ros2 ... &")` 与 `pkill`
- 这很直接，但工程上比较脆弱——并发启动、进程残留、名字冲突是高风险点

### Nav2 参数特征

- 全局坐标系 `map`，底座坐标系 `base_footprint`
- 雷达 topic `/hsrb/base_scan`
- `controller_frequency: 5.0`
- local/global costmap 设置了 robot footprint 和 inflation radius
- local planner 使用 `dwb_core::DWBLocalPlanner`

整体是一个保守的、低速的室内二维导航配置。

---

## 指令理解方式

当前主控的 NLP 是**基于字符串切词和子串匹配**的关键词 spotter，不是语义解析。

相关逻辑：`extractInfo(...)`

它把指令按空格拆开，然后在 token 里分别找：

- 房间关键词：`living` / `bedroom` / `lobby` / `kitchen`
- 物体关键词：一长串对象名（apple, toy_penguin, canned_juice 等）
- 目标位置关键词：`white_side_table` / `corner_sofa` / `dining_table` / `wooden_bed` / `Avatar` 等

**重要限制**：
- 依赖英文关键词与官方指令文本高度一致
- 没有句法分析、错误恢复、歧义消解
- 没有处理 `Corrected_instruction` 的完整重解析流程

---

## 房间巡航与目标位置逻辑

当前主控使用**手工硬编码的房间巡航点与放置点**，而不是实时建图定位。

### `roomLocation(room, variation)`

对每个环境和房间，写死了一组 patrol 点：

- `living` / `bedroom` / `lobby` / `kitchen`
- 各环境点数不均：`Layout2019HM01` 多数房间 2 个点，`Layout2020HM01` 的 living 最多 4 个点

### `destLocation(dest, room)`

放置目标同样按环境写死坐标：桌面类、家具旁类、垃圾桶类、Avatar 位置等。

### ⚠️ 构想与现状的偏差

用户构想要求"按房间中心点边长为 2m 的矩形区域作为到达目标位置的判断标准"，而当前实现使用 0.5m 阈值判断是否在 patrol waypoint 附近。**这是需要优先对齐的差异**。

---

## 视觉模块的真实能力

### 订阅

- `/hsrb/hand_camera/image_raw` — 手部相机图像
- `/hsrb/head_rgbd_sensor/rgb/image_raw` — 头部 RGB（暂未用于主流程）
- `/detection_target` — 主控告诉 YOLO 当前关注哪个类别

### 发布

- `/hand_detection` — bbox xywh（供近距离对准）
- `/vision` — 目标位姿（**暂未用于主流程**）
- `/detection_depth` — 深度信息（**暂未用于主流程**）

### ⚠️ 真实定位

当前视觉模块更像是"基于手部相机的近距离目标对准辅助"，而不是"完整的房间级搜索、检测、3D 定位感知系统"。

**与用户构想的偏差**：构想的"旋转扫描 + 视觉识别模块协同搜索"在现有代码中部分实现（ROOM_SCAN + YOLO 检测），但：
- 头部 RGB 检测未串到主流程
- 深度信息未用于距离估计
- 每个 patrol waypoint 的系统性搜索尚未完整实现

---

## 抓取与放置逻辑

### 抓取前对准（`MoveToInFrontOfTarget` 状态）

- 最近 1 秒内有 `/hand_detection` → 根据框中心调底盘和手臂高度
- 框宽 ≥ 450px → 认为 "grasp_ready"
- 对准后微微前进，直到对准完成，进入 `Grasp`

### 抓取动作（`Grasp` 状态）

1. 发送 gripper close（`hand_motor_joint` → -0.105）
2. 等 8 秒
3. **✅ 订阅 `/hsrb/joint_states` 验证**（差值 > 0.03 则重试，最多 2 次）
4. 发送 `Object_grasped`
5. 转入去目标位置导航

### 放置动作（`Release` 状态）

1. 伸手臂到固定姿态
2. 等 4 秒
3. 张开 gripper（`hand_motor_joint` → 1.239）
4. 等 8 秒
5. 发送 `Task_finished`

**⚠️ 放置目标为 Avatar 时**：现有代码没有实现官方描述的"球形区域 handover"逻辑。

---

## Topic / Interface 总表

| 类别 | Topic | 方向 | 当前用途 |
|------|-------|------|----------|
| 比赛消息 | `/handyman/message/to_robot` | Avatar → 主控 | 接收 `Are_you_ready?` / `Environment` / `Instruction` / `Task_*` |
| 比赛消息 | `/handyman/message/to_moderator` | 主控 → Avatar | 回传 `I_am_ready` / `Room_reached` / `Object_grasped` / `Task_finished` |
| 目标检测目标 | `/detection_target` | 主控 → 视觉 | 告诉 YOLO 当前只关注哪个类别 |
| 手部检测结果 | `/hand_detection` | 视觉 → 主控 | bbox xywh，供近距离对准 |
| 目标位姿 | `/vision` | 视觉 → 主控 | 当前订阅了，但主流程基本没用 |
| 底盘控制 | `/hsrb/command_velocity` | 主控 → HSR | 底盘速度控制 |
| 手臂轨迹 | `/hsrb/arm_trajectory_controller/command` | 主控 → HSR | 手臂关节控制 |
| 夹爪控制 | `/hsrb/gripper_controller/command` | 主控 → HSR | 夹爪开合 |
| 关节状态 | `/hsrb/joint_states` | HSR → 主控 | **用于抓取验证** |
| 激光 | `/hsrb/base_scan` | HSR → Nav2/AMCL | 导航定位 |
| 手部相机 | `/hsrb/hand_camera/image_raw` | HSR → 视觉 | YOLO 目标检测输入 |
| 地图 | `/map` | map_server → 全局 | 静态地图 |
| 导航动作 | `navigate_to_pose` | 主控 ↔ Nav2 | 前往房间和放置点 |
| 初始位姿 | `/initialpose` | 主控 → AMCL | 切图后重定位 |

---

## 当前实现与文档/README 的主要不一致

### ROS1/旧项目表述残留

`src/handyman_ros2/README.md` 中仍有 ROS1 / `catkin_make` / 旧包名表述，不应作为当前实现事实来源。

### 视觉能力描述偏乐观

README 里描述成头相机、手相机、深度相机全面协同，但真实代码目前主要用的是手相机目标检测 + bbox 对准。

### 包名不一致

部分文档写成 `vision_ros2`，实际当前路径是 `src/handyman_vision_ros2`。

---

## 当前实现的明显短板

### 1. 规则覆盖不完整（⚠️ 与构想差距最大）

- `Does_not_exist` / `Corrected_instruction` — 部分实现
- Avatar handover 的特殊要求（球形区域）— 未实现
- `Time_is_up` 超时检测 — 未实现

### 2. 感知链不完整

- 房间级目标搜索感知闭环依赖 YOLO（未安装）
- `/vision` 的 3D 位姿没有真正支撑抓取
- 深度信息几乎没用上

### 3. 工程结构偏脆弱

- `handyman_sample.cpp` 单文件过大
- 通过 `system()` + `pkill` 管理导航进程
- 状态和副作用耦合重

### 4. 场景知识全靠硬编码坐标

- 一旦环境或对象布局有偏移，鲁棒性会迅速下降

---

## 后续阅读建议

如果新的进程需要快速进入状态，推荐按这个顺序看：

1. `Handyman-overview.md`（即本文档）
2. `.cursorrules`（强制约束与当前已知问题）
3. `rule_reference/handyman/SystemOverview · RoboCupatHomeSimhandyman-unity2 Wiki.md`
4. `rule_reference/handyman/RosMessage Handyman · RoboCupatHomeSimros2-competition-msgs Wiki.md`
5. `src/handyman_ros2/src/task_orchestrator_main.cpp`（模块化版本入口）
6. `src/handyman_ros2/src/task_orchestrator.cpp`（模块化状态机主循环）
7. `src/handyman_ros2/src/protocol_handler.cpp`（通信协议）
8. `src/handyman_ros2/src/map_nav_controller.cpp`（导航封装）
9. `src/handyman_ros2/src/vision_manipulation_controller.cpp`（视觉/机械臂）
10. `src/handyman_vision_ros2/scripts/object_detection_node.py`
11. `src/handyman_ros2/launch/hsr_nav_orchestrated.launch.py`
12. `src/handyman_ros2/param/nav2_params.yaml`

如果要查原始单文件实现（回退参考），再看：
- `src/handyman_ros2/src/handyman_sample.cpp`

如果要查官方消息基线 sample，再看：
- `src/ros2-competition-msgs/handyman_msgs/msg/HandymanMsg.msg`
- `src/ros2-competition-msgs/competition_test_tools/src/handyman_sample.cpp`

---

## 关键文件清单

### 模块化版本（当前推荐）

- `src/handyman_ros2/src/task_orchestrator_main.cpp` — 模块化版本入口
- `src/handyman_ros2/src/task_orchestrator.cpp` — 任务编排层（状态机主循环）
- `src/handyman_ros2/src/protocol_handler.cpp` — 通信协议（Avatar/Moderator）
- `src/handyman_ros2/src/map_nav_controller.cpp` — 导航控制与房间 patrol
- `src/handyman_ros2/src/vision_manipulation_controller.cpp` — 视觉对准与机械臂控制
- `src/handyman_ros2/src/load_map_manager.cpp` — 地图加载与 Nav2 生命周期
- `src/handyman_ros2/launch/hsr_nav_orchestrated.launch.py` — 模块化全栈启动
- `src/handyman_ros2/launch/task_orchestrator.launch.py` — 仅 task_orchestrator

### 原始版本（保留回退）

- `src/handyman_ros2/src/handyman_sample.cpp` — 单文件状态机（原始版本）
- `src/handyman_ros2/launch/hsr_nav.launch.py` — 原始全栈启动

### 辅助与共享文件（两版本共用）

- `src/handyman_ros2/src/room_navigation.cpp` — patrol waypoint 定义
- `src/handyman_ros2/src/placement_navigation.cpp` — 放置点坐标
- `src/handyman_ros2/src/teleop_key_handyman.cpp` — 键盘遥控工具
- `src/handyman_ros2/launch/handyman.launch.py` — 轻量启动
- `src/handyman_ros2/launch/amcl.launch.py` — AMCL
- `src/handyman_ros2/launch/move_base.launch.py` — Nav2
- `src/handyman_ros2/param/nav2_params.yaml` — Nav2 参数
- `src/handyman_ros2/maps/*.yaml` — 静态地图配置
- `src/handyman_vision_ros2/scripts/object_detection_node.py` — YOLO 视觉（⚠️ 待安装模型）
- `src/handyman_vision_ros2/launch/vision.launch.py` — 视觉节点启动
- `rule_reference/handyman/SystemOverview · RoboCupatHomeSimhandyman-unity2 Wiki.md`
- `rule_reference/handyman/RosMessage Handyman · RoboCupatHomeSimros2-competition-msgs Wiki.md`
- `.cursorrules` — 开发约束与强制规则
