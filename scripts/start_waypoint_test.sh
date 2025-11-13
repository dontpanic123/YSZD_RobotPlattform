#!/bin/bash

# Nav2 Waypoint 系统测试启动脚本

echo "🧪 启动Nav2 Waypoint系统测试..."

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2环境未设置，请先source ROS2环境"
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

# 检查是否在正确的目录
if [ ! -f "test_waypoint_system.py" ]; then
    echo "❌ 请在scripts目录下运行此脚本"
    exit 1
fi

echo "📋 测试内容:"
echo "   1. 服务可用性测试"
echo "   2. 里程计数据测试"
echo "   3. 录制工作流测试"
echo "   4. Waypoints文件测试"
echo ""
echo "🔧 测试步骤:"
echo "   1. 启动机器人系统"
echo "   2. 启动waypoint recorder"
echo "   3. 运行测试脚本"
echo "   4. 查看测试结果"
echo ""
echo "📊 测试输出:"
echo "   - 服务状态检查"
echo "   - 数据流验证"
echo "   - 录制功能测试"
echo "   - 文件格式验证"
echo ""

# 启动测试
python3 test_waypoint_system.py
