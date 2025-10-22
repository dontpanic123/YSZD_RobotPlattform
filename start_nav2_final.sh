#!/bin/bash

echo "🤖 启动Nav2导航系统（最终版本）"
echo "=============================="

# 设置工作目录
cd /home/bd/Documents/Robot/agv_sim

# 停止所有现有进程
echo "🛑 停止现有进程..."
pkill -f "nav2" 2>/dev/null || true
pkill -f "costmap" 2>/dev/null || true
pkill -f "lifecycle_manager" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
sleep 2

# 修复所有环境变量
echo "🔧 修复环境变量..."
export AMENT_TRACE_SETUP_FILES=""
export AMENT_PYTHON_EXECUTABLE=""
export COLCON_TRACE=""
export COLCON_PREFIX_PATH=""
export COLCON_PYTHON_EXECUTABLE=""
export CMAKE_PREFIX_PATH=""

# 加载ROS2环境
echo "📦 加载ROS2环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "✅ 环境设置完成！"

# 启动基础服务
echo "🚀 启动基础服务..."

# 1. 启动地图服务器
echo "🗺️  启动地图服务器..."
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/bd/Documents/Robot/agv_sim/maps/map/test_map.yaml &
sleep 3

# 2. 启动TF变换
echo "🔗 启动TF变换..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
sleep 2

# 3. 启动机器人节点
echo "🤖 启动机器人节点..."
ros2 run mecanum_robot robot_node.py &
sleep 2

# 4. 启动遥控节点
echo "🎮 启动遥控节点..."
ros2 run mecanum_robot remote_control.py &
sleep 2

# 检查基础服务
echo "🔍 检查基础服务状态..."
ros2 node list

echo "✅ 基础服务启动完成！"
echo "💡 现在可以检查节点是否正常运行"
echo "🛑 按Ctrl+C停止所有服务"








