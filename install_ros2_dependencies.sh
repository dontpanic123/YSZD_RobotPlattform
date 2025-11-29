#!/bin/bash

echo "📦 安装 ROS2 Humble 依赖包"
echo "=========================="
echo ""

# 检查ROS2是否已安装
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "❌ ROS2 Humble 未安装！"
    echo "💡 请先运行: ./install_ros2_humble.sh"
    exit 1
fi

echo "✅ 检测到 ROS2 Humble"
echo ""

# 更新包列表
echo "🔄 更新包列表..."
sudo apt update

# 安装基础ROS2包
echo ""
echo "📦 安装基础ROS2包..."
sudo apt install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs

# 安装Nav2相关包
echo ""
echo "🗺️  安装Nav2导航包..."
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-planner \
    ros-humble-nav2-controller \
    ros-humble-nav2-recoveries \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-core \
    ros-humble-nav2-msgs \
    ros-humble-nav2-util \
    ros-humble-nav2-common

# 安装图像处理相关包
echo ""
echo "📷 安装图像处理包..."
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-camera-calibration-parsers

# 安装Python依赖
echo ""
echo "🐍 安装Python依赖..."
sudo apt install -y \
    python3-opencv \
    python3-numpy \
    python3-opencv-contrib-python

# 验证安装
echo ""
echo "🧪 验证安装..."
source /opt/ros/humble/setup.bash

echo ""
echo "检查已安装的包:"
echo "---------------"

packages=(
    "robot_state_publisher"
    "joint_state_publisher"
    "rviz2"
    "nav2_map_server"
    "nav2_bringup"
)

for pkg in "${packages[@]}"; do
    if ros2 pkg list | grep -q "^$pkg$"; then
        echo "✅ $pkg"
    else
        echo "❌ $pkg (未找到)"
    fi
done

echo ""
echo "✅ 依赖包安装完成！"
echo ""
echo "💡 现在可以运行您的机器人系统了:"
echo "   cd web && ./start_simple_tabs.sh"


