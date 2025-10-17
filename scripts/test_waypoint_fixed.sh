#!/bin/bash

echo "🎯 测试修复后的Waypoint Follower"
echo "==============================="

# 设置工作目录
cd /home/bd/Documents/Robot/agv_sim

# 检查ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "✅ ROS2环境已设置"

# 使用绝对路径
WAYPOINTS_FILE="/home/bd/Documents/Robot/agv_sim/waypoints/waypoints_20251008_152959.json"

echo "📁 使用waypoints文件: $WAYPOINTS_FILE"

# 检查文件是否存在
if [ ! -f "$WAYPOINTS_FILE" ]; then
    echo "❌ Waypoints文件不存在: $WAYPOINTS_FILE"
    exit 1
fi

echo "✅ Waypoints文件存在"

# 启动waypoint follower
echo "🚀 启动简单Waypoint Follower..."
cd scripts
python3 simple_waypoint_follower.py --ros-args -p waypoints_file:="$WAYPOINTS_FILE" &

# 等待服务启动
sleep 3

echo "✅ 简单Waypoint Follower已启动"
echo ""
echo "🎮 测试控制命令:"
echo "   # 开始跟踪waypoints"
echo "   ros2 service call /start_following std_srvs/srv/Empty"
echo ""
echo "   # 停止跟踪waypoints"
echo "   ros2 service call /stop_following std_srvs/srv/Empty"
echo ""

# 等待用户输入
echo "按Enter键开始跟踪waypoints，或Ctrl+C退出..."
read

echo "🎯 开始跟踪waypoints..."
ros2 service call /start_following std_srvs/srv/Empty

echo "✅ 跟踪已启动！"
echo "💡 观察机器人移动，使用 'ros2 topic echo /cmd_vel' 监控速度"
echo "🛑 使用 'ros2 service call /stop_following std_srvs/srv/Empty' 停止跟踪"
