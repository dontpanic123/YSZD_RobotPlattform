#!/bin/bash

echo "🔍 摄像头话题故障排除指南"
echo "================================"

# 设置环境变量
export AMENT_TRACE_SETUP_FILES=0
export COLCON_TRACE=0
export AMENT_PYTHON_EXECUTABLE=python3

# 进入工作目录
# 获取脚本所在目录，然后获取工作空间根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"

# 设置工作目录
cd "$WORKSPACE_ROOT"

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo ""
echo "📋 1. 检查所有话题:"
ros2 topic list

echo ""
echo "📷 2. 检查摄像头相关话题:"
ros2 topic list | grep -E "(camera|image|apriltag)" || echo "❌ 未找到摄像头话题"

echo ""
echo "🤖 3. 检查节点状态:"
ros2 node list

echo ""
echo "📊 4. 检查摄像头节点详细信息:"
if ros2 node list | grep -q "camera_node"; then
    echo "✅ 摄像头节点正在运行"
    ros2 node info /camera_node
else
    echo "❌ 摄像头节点未运行"
fi

echo ""
echo "🔍 5. 检查进程状态:"
ps aux | grep -E "(camera|apriltag)" | grep -v grep

echo ""
echo "📈 6. 检查话题发布频率:"
if ros2 topic list | grep -q "/camera/image_raw"; then
    echo "摄像头图像话题频率:"
    timeout 5 ros2 topic hz /camera/image_raw || echo "无法获取频率信息"
else
    echo "❌ 摄像头图像话题不存在"
fi

echo ""
echo "🎯 7. 实时查看话题数据 (5秒):"
if ros2 topic list | grep -q "/camera/image_raw"; then
    echo "摄像头图像数据:"
    timeout 5 ros2 topic echo /camera/image_raw --once || echo "无法获取图像数据"
else
    echo "❌ 摄像头图像话题不存在"
fi

echo ""
echo "🛠️  8. 故障排除建议:"
echo "如果摄像头话题不存在，请尝试:"
echo "1. 重启AprilTag系统: ./stop_apriltag_robot.sh && ./start_apriltag_robot.sh"
echo "2. 检查摄像头硬件连接"
echo "3. 检查摄像头权限: ls -l /dev/video*"
echo "4. 查看详细日志: ros2 topic echo /rosout"

echo ""
echo "🎮 9. 可视化工具:"
echo "ros2 run rqt_image_view rqt_image_view"
echo "ros2 run rqt_graph rqt_graph"
