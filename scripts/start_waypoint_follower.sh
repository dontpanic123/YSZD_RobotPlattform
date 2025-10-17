#!/bin/bash

# Nav2 Waypoint Follower 启动脚本

echo "🎯 启动Nav2 Waypoint Follower..."

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2环境未设置，请先source ROS2环境"
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

# 检查是否在正确的目录
if [ ! -f "waypoint_follower.py" ]; then
    echo "❌ 请在scripts目录下运行此脚本"
    exit 1
fi

# 检查waypoints文件
if [ -z "$1" ]; then
    echo "❌ 请指定waypoints文件路径"
    echo "   用法: $0 <waypoints_file.json>"
    echo ""
    echo "📁 可用的waypoints文件:"
    ls -la /home/bd/Documents/Robot/agv_sim/waypoints/*.json 2>/dev/null || echo "   没有找到waypoints文件"
    exit 1
fi

WAYPOINTS_FILE="$1"

# 检查文件是否存在
if [ ! -f "$WAYPOINTS_FILE" ]; then
    echo "❌ Waypoints文件不存在: $WAYPOINTS_FILE"
    exit 1
fi

echo "📁 使用waypoints文件: $WAYPOINTS_FILE"
echo ""
echo "🎮 使用方法:"
echo "   1. 确保Nav2系统正在运行"
echo "   2. 确保机器人已定位"
echo "   3. 启动waypoint follower"
echo "   4. 机器人将自动跟踪waypoints"
echo ""
echo "🔧 参数设置:"
echo "   - Waypoints文件: $WAYPOINTS_FILE"
echo "   - 循环模式: 关闭"
echo "   - Waypoint超时: 30秒"
echo ""
echo "📊 状态监控:"
echo "   # 查看waypoint跟踪状态"
echo "   ros2 topic echo /follow_waypoints/feedback"
echo ""
echo "   # 查看waypoint跟踪结果"
echo "   ros2 topic echo /follow_waypoints/result"
echo ""

# 启动waypoint follower
python3 waypoint_follower.py --ros-args -p waypoints_file:="$WAYPOINTS_FILE"
