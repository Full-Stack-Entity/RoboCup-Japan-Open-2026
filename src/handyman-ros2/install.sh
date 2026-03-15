#!/bin/bash

# ROS2 改稿 - 自动安装脚本
# 用于快速设置开发环境

set -e

echo "=========================================="
echo "ROS2 改稿 - 自动安装脚本"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查是否为 Ubuntu 22.04
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "22.04" ]; then
        echo -e "${YELLOW}警告: 建议使用 Ubuntu 22.04${NC}"
    fi
fi

# 1. 检查 ROS2 Humble
echo -e "${GREEN}[1/6] 检查 ROS2 Humble...${NC}"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "✓ ROS2 Humble 已安装"
    source /opt/ros/humble/setup.bash
else
    echo -e "${RED}✗ ROS2 Humble 未安装${NC}"
    echo "请先安装 ROS2 Humble:"
    echo "  https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# 2. 安装 ROS2 依赖
echo -e "${GREEN}[2/6] 安装 ROS2 依赖...${NC}"
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-cv-bridge \
    python3-pip \
    python3-colcon-common-extensions

echo "✓ ROS2 依赖安装完成"

# 3. 安装 Python 依赖
echo -e "${GREEN}[3/6] 安装 Python 依赖...${NC}"
pip3 install --upgrade pip
pip3 install \
    ultralytics \
    torch \
    torchvision \
    opencv-python \
    numpy \
    PyYAML

echo "✓ Python 依赖安装完成"

# 4. 创建工作空间
echo -e "${GREEN}[4/6] 创建 ROS2 工作空间...${NC}"
WORKSPACE_DIR="$HOME/ros2_ws"

if [ ! -d "$WORKSPACE_DIR" ]; then
    mkdir -p "$WORKSPACE_DIR/src"
    echo "✓ 工作空间创建于: $WORKSPACE_DIR"
else
    echo "✓ 工作空间已存在: $WORKSPACE_DIR"
fi

# 5. 复制包到工作空间
echo -e "${GREEN}[5/6] 复制包到工作空间...${NC}"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ -d "$SCRIPT_DIR/vision" ]; then
    cp -r "$SCRIPT_DIR/vision" "$WORKSPACE_DIR/src/"
    echo "✓ Vision 包已复制"
else
    echo -e "${YELLOW}⚠ Vision 包未找到${NC}"
fi

if [ -d "$SCRIPT_DIR/handyman" ]; then
    cp -r "$SCRIPT_DIR/handyman" "$WORKSPACE_DIR/src/"
    echo "✓ Handyman 包已复制"
else
    echo -e "${YELLOW}⚠ Handyman 包未找到${NC}"
fi

# 6. 编译 Vision 包
echo -e "${GREEN}[6/6] 编译 Vision 包...${NC}"
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash

if colcon build --packages-select vision; then
    echo "✓ Vision 包编译成功"
else
    echo -e "${RED}✗ Vision 包编译失败${NC}"
    exit 1
fi

# 完成
echo ""
echo "=========================================="
echo -e "${GREEN}安装完成！${NC}"
echo "=========================================="
echo ""
echo "下一步操作:"
echo ""
echo "1. Source 工作空间:"
echo "   source ~/ros2_ws/install/setup.bash"
echo ""
echo "2. 测试 Vision 包:"
echo "   ros2 launch vision vision.launch.py"
echo ""
echo "3. 完成 Handyman 包:"
echo "   - 参考 CPP_MIGRATION_GUIDE.md"
echo "   - 从原稿复制 maps/ 和 param/ 目录"
echo "   - 完成 C++ 代码迁移"
echo ""
echo "4. 编译 Handyman 包:"
echo "   cd ~/ros2_ws"
echo "   colcon build --packages-select handyman"
echo ""
echo "详细文档请查看:"
echo "  - README.md"
echo "  - QUICKSTART.md"
echo "  - CPP_MIGRATION_GUIDE.md"
echo "  - PROJECT_SUMMARY.md"
echo ""
