#!/bin/bash
# ROS2 改稿1 — 自动安装脚本
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "=========================================="
echo "ROS2 改稿1 — 自动安装脚本"
echo "=========================================="

# 1. 检查 ROS2 Humble
echo -e "${GREEN}[1/6] 检查 ROS2 Humble...${NC}"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "✓ ROS2 Humble 已安装"
    source /opt/ros/humble/setup.bash
else
    echo -e "${RED}✗ ROS2 Humble 未安装，请先安装：https://docs.ros.org/en/humble/Installation.html${NC}"
    exit 1
fi

# 2. 安装 ROS2 系统依赖
echo -e "${GREEN}[2/6] 安装 ROS2 系统依赖...${NC}"
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
echo "✓ ROS2 系统依赖安装完成"

# 3. 安装 Python 依赖
echo -e "${GREEN}[3/6] 安装 Python 依赖...${NC}"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pip3 install -r "$SCRIPT_DIR/vision_ros2/requirements.txt"
echo "✓ Python 依赖安装完成"

# 4. 创建工作空间
echo -e "${GREEN}[4/6] 创建 ROS2 工作空间...${NC}"
WORKSPACE_DIR="$HOME/ros2_ws"
mkdir -p "$WORKSPACE_DIR/src"
echo "✓ 工作空间: $WORKSPACE_DIR"

# 5. 复制包到工作空间（使用正确的包名）
echo -e "${GREEN}[5/6] 复制包到工作空间...${NC}"

if [ -d "$SCRIPT_DIR/handyman_msgs" ]; then
    rm -rf "$WORKSPACE_DIR/src/handyman_msgs"
    cp -r "$SCRIPT_DIR/handyman_msgs" "$WORKSPACE_DIR/src/handyman_msgs"
    echo "✓ handyman_msgs 包已复制"
else
    echo -e "${RED}✗ handyman_msgs 包未找到，请确认目录结构${NC}"
    exit 1
fi

if [ -d "$SCRIPT_DIR/handyman_ros2" ]; then
    rm -rf "$WORKSPACE_DIR/src/handyman_ros2"
    cp -r "$SCRIPT_DIR/handyman_ros2" "$WORKSPACE_DIR/src/handyman_ros2"
    echo "✓ handyman_ros2 包已复制"
else
    echo -e "${RED}✗ handyman_ros2 包未找到${NC}"
    exit 1
fi

if [ -d "$SCRIPT_DIR/vision_ros2" ]; then
    rm -rf "$WORKSPACE_DIR/src/vision_ros2"
    cp -r "$SCRIPT_DIR/vision_ros2" "$WORKSPACE_DIR/src/vision_ros2"
    echo "✓ vision_ros2 包已复制"
else
    echo -e "${RED}✗ vision_ros2 包未找到${NC}"
    exit 1
fi

# 6. 编译所有包
echo -e "${GREEN}[6/6] 编译所有包...${NC}"
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash

# handyman_msgs 必须先编译，handyman_ros2 依赖它
if colcon build --packages-select handyman_msgs; then
    echo "✓ handyman_msgs 编译成功"
else
    echo -e "${RED}✗ handyman_msgs 编译失败${NC}"
    exit 1
fi

source install/setup.bash

if colcon build --packages-select handyman_ros2 vision_ros2; then
    echo "✓ handyman_ros2 + vision_ros2 编译成功"
else
    echo -e "${RED}✗ 编译失败，请检查错误信息${NC}"
    exit 1
fi

source install/setup.bash

echo ""
echo "=========================================="
echo -e "${GREEN}✓ 安装完成！${NC}"
echo "=========================================="
echo ""
echo "下一步："
echo "  1. 把模型文件放到工作空间："
echo "     cp your_model.pt ~/ros2_ws/src/vision_ros2/models/last.pt"
echo "     cd ~/ros2_ws && colcon build --packages-select vision_ros2"
echo ""
echo "  2. 建图（新场地）："
echo "     source ~/ros2_ws/install/setup.bash"
echo "     ros2 launch handyman_ros2 make_map.launch.py"
echo ""
echo "  3. 正式运行："
echo "     ros2 launch handyman_ros2 hsr_nav.launch.py"
echo ""
echo "建议把以下两行加入 ~/.bashrc："
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/ros2_ws/install/setup.bash"
