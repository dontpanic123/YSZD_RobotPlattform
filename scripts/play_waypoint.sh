#!/bin/bash

echo "🎯 Waypoint重播脚本"
echo "=================="

# 设置工作目录
cd /home/bd/Documents/Robot/agv_sim

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2环境未设置，请先设置ROS2环境"
    echo "   source /opt/ros/humble/setup.bash"
    echo "   source install/setup.bash"
    exit 1
fi

echo "✅ ROS2环境已设置: $ROS_DISTRO"

# 显示可用的waypoints文件
echo ""
echo "📁 可用的Waypoints文件:"
ls -la waypoints/*.json 2>/dev/null || echo "   没有找到waypoints文件"

# 如果没有指定文件，使用最新的
if [ -z "$1" ]; then
    LATEST_FILE=$(ls -t waypoints/*.json 2>/dev/null | head -1)
    if [ -n "$LATEST_FILE" ]; then
        WAYPOINTS_FILE="$LATEST_FILE"
        echo "🎯 使用最新的waypoints文件: $WAYPOINTS_FILE"
    else
        echo "❌ 没有找到waypoints文件"
        exit 1
    fi
else
    WAYPOINTS_FILE="$1"
fi

# 检查文件是否存在
if [ ! -f "$WAYPOINTS_FILE" ]; then
    echo "❌ Waypoints文件不存在: $WAYPOINTS_FILE"
    exit 1
fi

echo "📁 使用waypoints文件: $WAYPOINTS_FILE"

# 检查waypoint follower是否在运行
if ! pgrep -f "waypoint_follower.py" > /dev/null; then
    echo "🚀 启动Waypoint Follower..."
    cd scripts
    python3 waypoint_follower.py --ros-args -p waypoints_file:="$WAYPOINTS_FILE" &
    FOLLOWER_PID=$!
    echo "✅ Waypoint Follower已启动 (PID: $FOLLOWER_PID)"
    cd ..
else
    echo "✅ Waypoint Follower已在运行"
fi

echo ""
echo "🎮 重播控制:"
echo "   # 开始重播"
echo "   ros2 service call /start_following std_srvs/srv/Empty"
echo ""
echo "   # 停止重播"
echo "   ros2 service call /stop_following std_srvs/srv/Empty"
echo ""
echo "📊 监控状态:"
echo "   # 查看重播状态"
echo "   ros2 topic echo /follow_waypoints/feedback"
echo ""
echo "   # 查看重播结果"
echo "   ros2 topic echo /follow_waypoints/result"
echo ""

# 等待用户输入
echo "按Enter键开始重播，或Ctrl+C退出..."
read

echo "🎯 开始重播waypoints..."
ros2 service call /start_following std_srvs/srv/Empty

echo "✅ 重播已启动！"
echo "💡 使用 'ros2 topic echo /follow_waypoints/feedback' 监控进度"
echo "🛑 使用 'ros2 service call /stop_following std_srvs/srv/Empty' 停止重播"
