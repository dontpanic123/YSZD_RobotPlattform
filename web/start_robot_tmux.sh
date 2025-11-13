#!/bin/bash

echo "🤖 启动机器人控制系统 (Tmux版本)"
echo "================================="

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

# 检查tmux是否安装
if ! command -v tmux &> /dev/null; then
    echo "❌ tmux 未安装，请先安装: sudo apt install tmux"
    exit 1
fi

echo "🚀 启动所有服务..."

# 创建tmux会话
tmux new-session -d -s robot_control -n "ROS2机器人系统"

# 创建其他窗口
tmux new-window -t robot_control -n "ROS2 WebSocket Bridge"
tmux new-window -t robot_control -n "Waypoint录制器"
tmux new-window -t robot_control -n "Web控制台"

# 在第一个窗口启动ROS2机器人系统
tmux send-keys -t robot_control:0 "source /opt/ros/humble/setup.bash" Enter
tmux send-keys -t robot_control:0 "source install/setup.bash" Enter
tmux send-keys -t robot_control:0 "echo '🤖 启动ROS2机器人系统...'" Enter
tmux send-keys -t robot_control:0 "ros2 launch mecanum_robot apriltag_robot.launch.py" Enter

# 在第二个窗口启动WebSocket Bridge
tmux send-keys -t robot_control:1 "source /opt/ros/humble/setup.bash" Enter
tmux send-keys -t robot_control:1 "source install/setup.bash" Enter
tmux send-keys -t robot_control:1 "echo '🌉 启动ROS2 WebSocket Bridge...'" Enter
tmux send-keys -t robot_control:1 "python3 scripts/ros2_websocket_bridge.py" Enter

# 在第三个窗口启动Waypoint录制器
tmux send-keys -t robot_control:2 "source /opt/ros/humble/setup.bash" Enter
tmux send-keys -t robot_control:2 "source install/setup.bash" Enter
tmux send-keys -t robot_control:2 "cd scripts" Enter
tmux send-keys -t robot_control:2 "echo '🎯 启动Waypoint录制器...'" Enter
tmux send-keys -t robot_control:2 "python3 waypoint_recorder.py" Enter

# 在第四个窗口启动Web控制台
tmux send-keys -t robot_control:3 "cd web" Enter
tmux send-keys -t robot_control:3 "echo '🌐 启动Web控制台...'" Enter
tmux send-keys -t robot_control:3 "python3 -m http.server 8080" Enter

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
echo "   - ROS2机器人系统: 运行中 (窗口 0)"
echo "   - ROS2 WebSocket Bridge: 运行中 (窗口 1)"
echo "   - Waypoint录制器: 运行中 (窗口 2)"
echo "   - Web控制台: 运行中 (窗口 3)"
echo ""
echo "💡 使用说明:"
echo "   1. 打开浏览器访问 http://localhost:8080"
echo "   2. 使用WASD键或摇杆控制机器人"
echo "   3. 切换到'Waypoint导航'Tab进行路径录制"
echo "   4. 观察摄像头画面和AprilTag检测"
echo "   5. 所有服务都在tmux会话中运行"
echo ""
echo "🛠️ Tmux操作:"
echo "   - 查看会话: tmux list-sessions"
echo "   - 连接到会话: tmux attach-session -t robot_control"
echo "   - 切换窗口: Ctrl+b 然后按数字键 (0-3)"
echo "   - 分离会话: Ctrl+b 然后按 d"
echo "   - 关闭会话: tmux kill-session -t robot_control"
echo ""
echo "🛑 关闭所有服务:"
echo "   tmux kill-session -t robot_control"

# 连接到tmux会话
echo "🔗 正在连接到tmux会话..."
tmux attach-session -t robot_control
