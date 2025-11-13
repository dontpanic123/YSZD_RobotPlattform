#!/bin/bash

echo "🤖 启动机器人控制系统 (简单Tab版本)"
echo "=================================="

# 设置工作目录
# 获取脚本所在目录，然后获取工作空间根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# 设置工作目录
cd "$WORKSPACE_ROOT"

# 检查ROS2环境
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "❌ ROS2 Humble 未安装，请先安装ROS2"
    exit 1
fi

echo "✅ 检查ROS2环境..."

# 使用gnome-terminal的正确语法创建多个tab
echo "🚀 启动所有服务..."

gnome-terminal \
    --title="机器人控制系统" \
    --tab --title="ROS2机器人系统" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '🤖 启动ROS2机器人系统...'
        ros2 launch mecanum_robot apriltag_robot.launch.py
        bash
    " \
    --tab --title="ROS2 WebSocket Bridge" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '🌉 启动ROS2 WebSocket Bridge...'
        python3 scripts/ros2_websocket_bridge.py
        bash
    " \
    --tab --title="Waypoint录制器" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        cd scripts
        echo '🎯 启动Waypoint录制器...'
        python3 waypoint_recorder.py
        bash
    " \
    --tab --title="Web控制台" -- bash -c "
        cd web
        echo '🌐 启动Web控制台...'
        python3 -m http.server 8080
        bash
    " &

# 等待服务启动
echo "⏳ 等待服务启动..."
sleep 3

echo "✅ 系统启动完成！"
echo ""
echo "🔗 访问地址:"
echo "   Web控制台: http://localhost:8080"
echo "   ROS2 Web Bridge: ws://localhost:9090"
echo ""
echo "📊 系统状态:"
echo "   - ROS2机器人系统: 运行中 (Tab 1)"
echo "   - ROS2 WebSocket Bridge: 运行中 (Tab 2)"
echo "   - Waypoint录制器: 运行中 (Tab 3)"
echo "   - Web控制台: 运行中 (Tab 4)"
echo ""
echo "💡 使用说明:"
echo "   1. 打开浏览器访问 http://localhost:8080"
echo "   2. 使用WASD键或摇杆控制机器人"
echo "   3. 切换到'Waypoint导航'Tab进行路径录制"
echo "   4. 观察摄像头画面和AprilTag检测"
echo "   5. 所有服务都在一个terminal的不同tab中运行"
echo ""
echo "🛑 关闭terminal窗口即可停止所有服务"

# 等待用户中断
wait

