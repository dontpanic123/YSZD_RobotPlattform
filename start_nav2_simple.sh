#!/bin/bash

echo "🤖 启动Nav2导航系统 (简化版本)"
echo "=============================="

# 设置工作目录
cd /home/bd/Documents/Robot/agv_sim

# 停止现有进程
echo "🛑 停止现有进程..."
pkill -f "nav2" 2>/dev/null || true
pkill -f "costmap" 2>/dev/null || true
pkill -f "lifecycle_manager" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
sleep 2

# 修复环境变量
export AMENT_TRACE_SETUP_FILES=""
export AMENT_PYTHON_EXECUTABLE=""
export COLCON_TRACE=""
export COLCON_PREFIX_PATH=""
export COLCON_PYTHON_EXECUTABLE=""
export CMAKE_PREFIX_PATH=""

# 加载ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "🚀 启动核心服务..."

# 1. 启动地图服务器
echo "🗺️  启动地图服务器..."
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/bd/Documents/Robot/agv_sim/maps/map/test_map.yaml &
sleep 3

# 2. 配置和激活地图服务器
echo "🔧 配置地图服务器..."
ros2 lifecycle set /map_server configure
sleep 1
echo "✅ 激活地图服务器..."
ros2 lifecycle set /map_server activate
sleep 2

# 3. 启动TF变换
echo "🔗 启动TF变换..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
sleep 1

# 4. 启动机器人状态发布器
echo "🤖 启动机器人状态发布器..."
ros2 run robot_state_publisher robot_state_publisher urdf/mecanum_robot.urdf &
sleep 1

# 5. 启动关节状态发布器
echo "🔗 启动关节状态发布器..."
ros2 run joint_state_publisher joint_state_publisher &
sleep 1

# 6. 启动机器人节点
echo "🤖 启动机器人节点..."
ros2 run mecanum_robot robot_node.py &
sleep 1

# 7. 启动RViz
echo "👁️  启动RViz可视化..."
ros2 run rviz2 rviz2 -d rviz/nav2_with_obstacles.rviz &
sleep 2

# 检查服务状态
echo "🔍 检查服务状态..."
ros2 node list | head -10

echo ""
echo "✅ 核心服务启动完成！"
echo "💡 现在可以查看地图和机器人模型"
echo "🛑 按Ctrl+C停止所有服务"








