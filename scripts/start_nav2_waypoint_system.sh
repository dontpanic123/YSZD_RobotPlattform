#!/bin/bash

# Nav2 Waypoint 完整系统启动脚本

echo "🚀 启动Nav2 Waypoint完整系统..."

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2环境未设置，请先source ROS2环境"
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

# 创建必要的目录
mkdir -p /home/bd/Documents/Robot/agv_sim/waypoints
mkdir -p /home/bd/Documents/Robot/agv_sim/logs

echo "📁 创建目录:"
echo "   - Waypoints: /home/bd/Documents/Robot/agv_sim/waypoints"
echo "   - Logs: /home/bd/Documents/Robot/agv_sim/logs"
echo ""

# 检查参数
MODE=${1:-"recorder"}  # recorder, follower, 或 both
WAYPOINTS_FILE=${2:-""}

case $MODE in
    "recorder")
        echo "🎯 启动Waypoint Recorder模式"
        echo "📋 录制步骤:"
        echo "   1. 启动机器人系统"
        echo "   2. 手动控制机器人移动"
        echo "   3. 使用服务开始/停止录制"
        echo "   4. 保存waypoints文件"
        echo ""
        echo "🔧 服务命令:"
        echo "   ros2 service call /start_recording std_srvs/srv/Empty"
        echo "   ros2 service call /stop_recording std_srvs/srv/Empty"
        echo "   ros2 service call /save_waypoints std_srvs/srv/Empty"
        echo ""
        python3 waypoint_recorder.py
        ;;
        
    "follower")
        if [ -z "$WAYPOINTS_FILE" ]; then
            echo "❌ Follower模式需要指定waypoints文件"
            echo "   用法: $0 follower <waypoints_file.json>"
            echo ""
            echo "📁 可用的waypoints文件:"
            ls -la /home/bd/Documents/Robot/agv_sim/waypoints/*.json 2>/dev/null || echo "   没有找到waypoints文件"
            exit 1
        fi
        
        echo "🎯 启动Waypoint Follower模式"
        echo "📁 使用waypoints文件: $WAYPOINTS_FILE"
        echo ""
        echo "📋 跟踪步骤:"
        echo "   1. 确保Nav2系统正在运行"
        echo "   2. 确保机器人已定位"
        echo "   3. 机器人将自动跟踪waypoints"
        echo ""
        python3 waypoint_follower.py --ros-args -p waypoints_file:="$WAYPOINTS_FILE"
        ;;
        
    "both")
        echo "🎯 启动完整Waypoint系统"
        echo "📋 系统包含:"
        echo "   - Waypoint Recorder (录制)"
        echo "   - Waypoint Follower (跟踪)"
        echo "   - 可视化工具"
        echo ""
        echo "🔧 使用方法:"
        echo "   1. 先录制waypoints"
        echo "   2. 再使用waypoints进行自动导航"
        echo ""
        
        # 启动recorder
        echo "启动Waypoint Recorder..."
        python3 waypoint_recorder.py &
        RECORDER_PID=$!
        
        # 等待用户输入
        echo "按Enter键继续启动Follower..."
        read
        
        # 启动follower
        if [ -n "$WAYPOINTS_FILE" ]; then
            echo "启动Waypoint Follower..."
            python3 waypoint_follower.py --ros-args -p waypoints_file:="$WAYPOINTS_FILE" &
            FOLLOWER_PID=$!
        fi
        
        # 等待用户中断
        echo "按Ctrl+C停止所有服务..."
        wait
        ;;
        
    *)
        echo "❌ 未知模式: $MODE"
        echo "   可用模式: recorder, follower, both"
        exit 1
        ;;
esac
