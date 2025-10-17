#!/bin/bash

echo "🎯 启动简单Waypoint Follower"
echo "=========================="

# 设置工作目录
cd /home/bd/Documents/Robot/agv_sim

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2环境未设置，请先设置ROS2环境"
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✅ ROS2环境已设置: $ROS_DISTRO"

# 检查waypoints文件
if [ -z "$1" ]; then
    LATEST_FILE=$(ls -t ../waypoints/*.json 2>/dev/null | head -1)
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

# 检查机器人系统是否运行
echo "🔍 检查机器人系统..."
if ! ros2 topic list | grep -q "/cmd_vel"; then
    echo "⚠️ 没有找到/cmd_vel话题，机器人控制系统可能未启动"
    echo "💡 请先启动机器人系统:"
    echo "   cd web && ./start_complete_system.sh"
    echo ""
fi

if ! ros2 topic list | grep -q "/odom"; then
    echo "⚠️ 没有找到/odom话题，里程计可能未启动"
    echo "💡 请确保机器人系统正在运行"
    echo ""
fi

echo "🚀 启动简单Waypoint Follower..."
cd scripts
python3 simple_waypoint_follower.py --ros-args -p waypoints_file:="$WAYPOINTS_FILE" &

# 等待服务启动
sleep 3

echo "✅ 简单Waypoint Follower已启动"
echo ""
echo "🎮 控制命令:"
echo "   # 开始跟踪waypoints"
echo "   ros2 service call /start_following std_srvs/srv/Empty"
echo ""
echo "   # 停止跟踪waypoints"
echo "   ros2 service call /stop_following std_srvs/srv/Empty"
echo ""
echo "📊 监控状态:"
echo "   # 查看机器人速度"
echo "   ros2 topic echo /cmd_vel"
echo ""
echo "   # 查看机器人位置"
echo "   ros2 topic echo /odom"
echo ""
echo "💡 提示:"
echo "   - 确保机器人系统正在运行"
echo "   - 确保机器人已正确定位"
echo "   - 观察机器人移动情况"
echo ""

# 等待用户输入
echo "按Enter键开始跟踪waypoints，或Ctrl+C退出..."
read

echo "🎯 开始跟踪waypoints..."
ros2 service call /start_following std_srvs/srv/Empty

echo "✅ 跟踪已启动！"
echo "💡 观察机器人移动，使用 'ros2 topic echo /cmd_vel' 监控速度"
echo "🛑 使用 'ros2 service call /stop_following std_srvs/srv/Empty' 停止跟踪"
