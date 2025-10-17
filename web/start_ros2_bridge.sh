#!/bin/bash

echo "🌉 启动ROS2 WebSocket Bridge"
echo "============================"

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source /home/bd/Documents/Robot/agv_sim/install/setup.bash

echo "✅ ROS2环境已设置"

# 检查Python依赖
echo "📦 检查Python依赖..."
python3 -c "import websockets" 2>/dev/null || {
    echo "📦 安装websockets..."
    pip3 install websockets
}

echo "🚀 启动ROS2 WebSocket Bridge..."
echo "🔗 WebSocket地址: ws://localhost:9090"
echo "📡 支持的话题: /cmd_vel, /camera/image_raw, /apriltag_*"

# 启动Python WebSocket桥接器
cd /home/bd/Documents/Robot/agv_sim
python3 scripts/ros2_websocket_bridge.py

echo "👋 ROS2 WebSocket Bridge已停止"
