#!/bin/bash

echo "🔍 检查摄像头话题..."

# 设置环境变量
export AMENT_TRACE_SETUP_FILES=0
export COLCON_TRACE=0
export AMENT_PYTHON_EXECUTABLE=python3

# 进入工作目录
cd /home/bd/Documents/Robot/agv_sim

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "📋 所有话题:"
ros2 topic list

echo ""
echo "📷 摄像头相关话题:"
ros2 topic list | grep -E "(camera|image|apriltag)" || echo "未找到摄像头话题"

echo ""
echo "🎯 话题详细信息:"
if ros2 topic list | grep -q "/camera/image_raw"; then
    echo "摄像头图像话题:"
    ros2 topic info /camera/image_raw
else
    echo "❌ 摄像头图像话题未发布"
fi

if ros2 topic list | grep -q "/camera/camera_info"; then
    echo "摄像头信息话题:"
    ros2 topic info /camera/camera_info
else
    echo "❌ 摄像头信息话题未发布"
fi

if ros2 topic list | grep -q "/apriltag_pose"; then
    echo "AprilTag位姿话题:"
    ros2 topic info /apriltag_pose
else
    echo "❌ AprilTag位姿话题未发布"
fi

echo ""
echo "🎮 实时查看命令:"
echo "ros2 topic echo /camera/image_raw"
echo "ros2 topic echo /camera/camera_info"
echo "ros2 topic echo /apriltag_pose"
echo "ros2 run rqt_image_view rqt_image_view"
