#!/bin/bash

echo "🧪 测试地图和模型加载"
echo "===================="

# 获取脚本所在目录，然后获取工作空间根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"

# 设置工作目录
cd "$WORKSPACE_ROOT"

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

echo "🚀 启动基础服务..."

# 1. 启动地图服务器
echo "🗺️  启动地图服务器..."
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$WORKSPACE_ROOT/maps/map/test_map.yaml" &
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
ros2 run robot_state_publisher robot_state_publisher &
sleep 1

# 5. 启动关节状态发布器
echo "🔗 启动关节状态发布器..."
ros2 run joint_state_publisher joint_state_publisher &
sleep 1

# 6. 启动机器人节点
echo "🤖 启动机器人节点..."
ros2 run mecanum_robot robot_node.py &
sleep 1

# 检查服务状态
echo "🔍 检查服务状态..."
echo "节点列表:"
ros2 node list | head -10

echo ""
echo "话题列表:"
ros2 topic list | grep -E "(map|robot|tf)" | head -10

echo ""
echo "地图数据检查:"
ros2 topic echo /map --once | head -5

echo ""
echo "机器人描述检查:"
ros2 topic echo /robot_description --once | head -3

echo ""
echo "TF变换检查:"
ros2 run tf2_ros tf2_echo map odom --once

echo ""
echo "✅ 测试完成！"
echo "💡 如果看到地图和机器人模型，说明加载成功"
echo "🛑 按Ctrl+C停止所有服务"















