#!/bin/bash

echo "🎯 快速重播Waypoint"
echo "=================="

# 获取脚本所在目录，然后获取工作空间根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# 设置工作目录
cd "$WORKSPACE_ROOT"

# 检查ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 使用最新的waypoints文件
LATEST_FILE=$(ls -t "$WORKSPACE_ROOT/waypoints"/*.json 2>/dev/null | head -1)

if [ -z "$LATEST_FILE" ]; then
    echo "❌ 没有找到waypoints文件"
    exit 1
fi

echo "📁 使用waypoints文件: $LATEST_FILE"

# 启动waypoint follower
echo "🚀 启动Waypoint Follower..."
cd scripts
python3 waypoint_follower.py --ros-args -p waypoints_file:="$LATEST_FILE" &

# 等待一下让服务启动
sleep 3

echo "🎯 开始重播waypoints..."
ros2 service call /start_following std_srvs/srv/Empty

echo "✅ 重播已启动！"
echo "💡 监控进度: ros2 topic echo /follow_waypoints/feedback"
echo "🛑 停止重播: ros2 service call /stop_following std_srvs/srv/Empty"
