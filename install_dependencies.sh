#!/bin/bash

echo "🔧 安装AprilTag检测系统依赖"
echo "=============================="

echo "1. 安装ROS2依赖包..."
sudo apt update
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-camera-calibration-parsers \
    python3-opencv \
    python3-numpy

echo "2. 安装OpenCV和AprilTag依赖..."
sudo apt install -y \
    python3-opencv-contrib-python \
    python3-cv-bridge \
    libopencv-dev \
    libopencv-contrib-dev

echo "3. 安装Python依赖..."
pip3 install opencv-python opencv-contrib-python numpy

echo "4. 验证安装..."
python3 -c "import cv2; print('OpenCV版本:', cv2.__version__)"
python3 -c "import numpy; print('NumPy版本:', numpy.__version__)"

echo "✅ 依赖安装完成！"
echo ""
echo "现在可以运行AprilTag检测系统："
echo "ros2 launch mecanum_robot apriltag_robot.launch.py"















