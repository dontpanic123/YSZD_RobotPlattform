#!/bin/bash

# 检查摄像头话题的脚本

echo "🔍 检查摄像头话题..."

# 设置环境变量以避免错误
export AMENT_TRACE_SETUP_FILES=0
export COLCON_TRACE=0
export AMENT_PYTHON_EXECUTABLE=python3

# 进入工作目录
cd /home/bd/Documents/Robot/agv_sim

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "📋 所有话题列表:"
ros2 topic list

echo ""
echo "📷 摄像头相关话题:"
ros2 topic list | grep camera

echo ""
echo "📊 话题详细信息:"
echo "摄像头图像话题:"
ros2 topic info /camera/image_raw

echo ""
echo "摄像头信息话题:"
ros2 topic info /camera/camera_info

echo ""
echo "AprilTag检测话题:"
ros2 topic list | grep apriltag

echo ""
echo "🎯 实时查看摄像头图像:"
echo "运行以下命令查看实时图像:"
echo "ros2 run rqt_image_view rqt_image_view"
echo ""
echo "或者查看话题数据:"
echo "ros2 topic echo /camera/image_raw"
echo "ros2 topic echo /camera/camera_info"
