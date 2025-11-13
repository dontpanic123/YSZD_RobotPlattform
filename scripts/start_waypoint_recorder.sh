#!/bin/bash

# Waypoint Recorder 启动脚本

echo "🎯 启动Waypoint Recorder..."

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2环境未设置，请先source ROS2环境"
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

# 检查是否在正确的目录
if [ ! -f "waypoint_recorder.py" ]; then
    echo "❌ 请在scripts目录下运行此脚本"
    exit 1
fi

# 获取脚本所在目录，然后获取工作空间根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# 创建waypoints输出目录
mkdir -p "$WORKSPACE_ROOT/waypoints"

echo "📁 Waypoints输出目录: $WORKSPACE_ROOT/waypoints"
echo ""
echo "🎮 使用方法:"
echo "   1. 启动机器人系统"
echo "   2. 手动控制机器人移动"
echo "   3. 使用服务开始/停止录制"
echo "   4. 保存waypoints文件"
echo ""
echo "🔧 服务命令:"
echo "   # 开始录制"
echo "   ros2 service call /start_recording std_srvs/srv/Empty"
echo ""
echo "   # 停止录制"
echo "   ros2 service call /stop_recording std_srvs/srv/Empty"
echo ""
echo "   # 保存waypoints"
echo "   ros2 service call /save_waypoints std_srvs/srv/Empty"
echo ""
echo "📊 参数设置:"
echo "   - 距离阈值: 0.5m (waypoint_distance_threshold)"
echo "   - 角度阈值: 0.2rad (waypoint_angle_threshold)"
echo "   - 里程计话题: /odom"
echo ""
echo "🎯 输出文件格式:"
echo "   - JSON格式，包含位置、方向、时间戳"
echo "   - Nav2兼容的waypoints格式"
echo "   - 可用于Nav2 Waypoint Follower"
echo ""

# 启动waypoint recorder
python3 waypoint_recorder.py
