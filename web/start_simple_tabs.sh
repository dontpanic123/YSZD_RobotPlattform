#!/bin/bash

echo "🤖 启动机器人控制系统 (简化Tab版本)"
echo "================================="

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

# 检查gnome-terminal
if ! command -v gnome-terminal &> /dev/null; then
    echo "❌ gnome-terminal 不可用"
    echo "💡 请安装: sudo apt install gnome-terminal"
    echo "💡 或使用tmux版本: ./start_robot_tmux.sh"
    exit 1
fi

echo "🚀 启动服务..."

# 使用更简单的方法，分别启动每个tab
echo "📱 启动Tab 1: ROS2机器人系统"
gnome-terminal --title="ROS2机器人系统" -- bash -c "
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    echo '🤖 启动ROS2机器人系统...'
    echo '🗺️  启动地图服务器...'
    # 启动地图服务器
    ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$WORKSPACE_ROOT/maps/map/test_map.yaml" &
    sleep 2
    # 启动机器人系统（包含修复版AprilTag检测）
    ros2 launch mecanum_robot apriltag_robot_fixed.launch.py
    bash
" &

sleep 1

echo "📱 启动Tab 2: ROS2 WebSocket Bridge"
gnome-terminal --title="ROS2 WebSocket Bridge" -- bash -c "
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    echo '🌉 启动ROS2 WebSocket Bridge...'
    python3 scripts/ros2_websocket_bridge.py
    bash
" &

sleep 1

echo "📱 启动Tab 3: Waypoint录制器"
gnome-terminal --title="Waypoint录制器" -- bash -c "
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    cd scripts
    echo '🎯 启动Waypoint录制器...'
    python3 waypoint_recorder.py
    bash
" &

sleep 1

echo "📱 启动Tab 4: ROS2服务代理"
gnome-terminal --title="ROS2服务代理" -- bash -c "
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    echo '🔧 启动ROS2服务代理...'
    python3 scripts/ros2_service_proxy.py
    bash
" &

sleep 1

echo "📱 启动Tab 5: Waypoint跟踪器"
gnome-terminal --title="Waypoint跟踪器" -- bash -c "
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    cd scripts
    echo '🎯 启动Waypoint跟踪器...'
    echo '📁 启动时不加载任何waypoint文件，等待用户选择'
    python3 simple_waypoint_follower.py
    bash
" &

sleep 1

echo "📱 启动Tab 6: 机器人状态机"
gnome-terminal --title="机器人状态机" -- bash -c "
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    echo '🤖 启动机器人状态机...'
    python3 scripts/robot_state_machine.py
    bash
" &

sleep 1

echo "📱 启动Tab 7: Web控制台"
gnome-terminal --title="Web控制台" -- bash -c "
    cd web
    echo '🌐 启动Web控制台...'
    python3 -m http.server 8080
    bash
" &

# 等待服务启动
echo "⏳ 等待服务启动..."
sleep 1

echo "✅ 系统启动完成！"
echo ""
echo "🔗 访问地址:"
echo "   Web控制台: http://localhost:8080"
echo "   ROS2 Web Bridge: ws://localhost:9090"
echo "   ROS2服务代理: http://localhost:8081"
echo ""
echo "📊 系统状态:"
echo "   - ROS2机器人系统: 运行中 (独立窗口)"
echo "   - ROS2 WebSocket Bridge: 运行中 (独立窗口)"
echo "   - Waypoint录制器: 运行中 (独立窗口)"
echo "   - ROS2服务代理: 运行中 (独立窗口)"
echo "   - Waypoint跟踪器: 运行中 (独立窗口)"
echo "   - 机器人状态机: 运行中 (独立窗口)"
echo "   - Web控制台: 运行中 (独立窗口)"
echo ""
echo "💡 使用说明:"
echo "   1. 打开浏览器访问 http://localhost:8080"
echo "   2. 使用WASD键或摇杆控制机器人"
echo "   3. 切换到'Waypoint导航'Tab进行路径录制和跟踪"
echo "   4. 观察摄像头画面和AprilTag检测"
echo "   5. 查看机器人状态机显示当前运行状态"
echo "   6. 所有服务都在独立的terminal窗口中运行"
echo ""
echo "🎯 Waypoint功能:"
echo "   - 录制: 手动控制机器人录制路径点"
echo "   - 跟踪: 自动跟踪已录制的路径"
echo "   - 服务: 通过HTTP API调用ROS2服务"
echo ""
echo "🛑 关闭terminal窗口即可停止对应服务"
echo ""
echo "💡 如果需要所有服务在一个terminal中，请使用:"
echo "   ./start_robot_tmux.sh"

# 等待用户中断
wait
