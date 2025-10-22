#!/bin/bash

# AprilTag机器人系统停止脚本
# 安全停止所有AprilTag相关进程并释放摄像头

echo "🛑 正在停止AprilTag机器人系统..."

# 停止启动脚本
echo "📋 停止启动脚本..."
pkill -f "start_apriltag_robot.sh"

# 停止ROS2启动器
echo "📋 停止ROS2启动器..."
pkill -f "apriltag_robot_fixed.launch.py"

# 停止摄像头节点
echo "📷 停止摄像头节点..."
pkill -f "camera_node"

# 停止AprilTag检测节点
echo "🏷️  停止AprilTag检测节点..."
pkill -f "apriltag_detector"

# 停止机器人节点
echo "🤖 停止机器人节点..."
pkill -f "robot_node.py"

# 停止位置计算节点
echo "🧮 停止位置计算节点..."
pkill -f "position_calculator.py"

# 停止路径发布节点
echo "📊 停止路径发布节点..."
pkill -f "path_publisher.py"

# 停止机器人状态发布器
echo "📡 停止机器人状态发布器..."
pkill -f "robot_state_publisher"

# 停止关节状态发布器
echo "🔗 停止关节状态发布器..."
pkill -f "joint_state_publisher"

# 停止RViz
echo "📊 停止RViz..."
pkill -f "rviz2"

# 等待进程完全停止
echo "⏳ 等待进程停止..."
sleep 2

# 检查摄像头是否释放
echo "📷 检查摄像头状态..."
if lsof /dev/video0 2>/dev/null; then
    echo "⚠️  摄像头仍被占用，强制释放..."
    fuser -k /dev/video0 2>/dev/null
else
    echo "✅ 摄像头已释放"
fi

# 检查剩余进程
echo "🔍 检查剩余进程..."
remaining=$(ps aux | grep -E "(camera|apriltag|mecanum|robot)" | grep -v grep | wc -l)

if [ $remaining -eq 0 ]; then
    echo "✅ 所有AprilTag相关进程已停止"
    echo "📷 摄像头已释放"
    echo "🎯 系统已安全退出"
else
    echo "⚠️  仍有 $remaining 个相关进程在运行"
    echo "🔍 剩余进程:"
    ps aux | grep -E "(camera|apriltag|mecanum|robot)" | grep -v grep
fi

echo ""
echo "🎮 系统已停止，摄像头已释放"
echo "💡 如需重新启动，请运行: ./start_apriltag_robot.sh"
