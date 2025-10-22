#!/bin/bash

# AprilTag机器人测试启动脚本
# 启动完整的AprilTag检测系统

echo "🤖 启动AprilTag机器人测试系统..."

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2环境未设置，正在设置..."
    source /opt/ros/humble/setup.bash
fi

# 设置ROS2环境变量以避免错误
export AMENT_TRACE_SETUP_FILES=0
export COLCON_TRACE=0

# 进入工作目录
cd /home/bd/Documents/Robot/agv_sim

# 设置环境变量 - 确保使用默认域ID
unset ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0
# 使用默认的RMW实现，不强制指定
# export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# 检查并安装依赖
echo "📦 检查依赖..."
if [ ! -f "install/setup.bash" ]; then
    echo "⚠️  项目未编译，正在编译..."
    colcon build
fi

# 设置环境
source install/setup.bash

# 检查摄像头
echo "📷 检查摄像头..."
if [ ! -e /dev/video0 ]; then
    echo "⚠️  警告: 未检测到摄像头设备 /dev/video0"
    echo "请确保摄像头已连接并可用"
fi

# 检查AprilTag图片
echo "🏷️  检查AprilTag资源..."
if [ -d "apriltags" ]; then
    echo "✅ AprilTag图片资源已找到"
    ls -la apriltags/
else
    echo "⚠️  警告: AprilTag图片目录不存在"
fi

echo ""
echo "🚀 启动AprilTag机器人系统..."
echo "📋 系统组件:"
echo "   - 麦克纳姆轮机器人节点"
echo "   - 摄像头节点 (修复版)"
echo "   - AprilTag检测节点"
echo "   - 位置计算节点"
echo "   - RViz可视化"
echo ""

# 启动系统
ros2 launch mecanum_robot apriltag_robot_fixed.launch.py

echo ""
echo "🎯 使用说明:"
echo "1. 在RViz中查看机器人模型和摄像头图像"
echo "2. 将AprilTag标签放在摄像头前方"
echo "3. 观察AprilTag检测结果"
echo "4. 使用键盘控制机器人移动"
echo ""
echo "⌨️  键盘控制:"
echo "   w/s: 前进/后退"
echo "   a/d: 左移/右移" 
echo "   q/e: 左转/右转"
echo "   x: 停止"
echo "   +/-: 调节速度"
echo ""
echo "🔍 调试命令:"
echo "   ros2 topic echo /apriltag_pose"
echo "   ros2 topic echo /robot_relative_pose"
echo "   ros2 topic echo /apriltag_status"
