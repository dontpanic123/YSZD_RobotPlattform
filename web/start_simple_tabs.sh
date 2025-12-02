#!/bin/bash

# ============================================================================
# 配置区域 - 所有可配置参数集中管理
# ============================================================================
ROS2_DISTRO="humble"
ROS2_SETUP_PATH="/opt/ros/${ROS2_DISTRO}/setup.bash"
UART_DEVICE="/dev/ttyTHS1"
UART_BAUDRATE="115200"
MAP_YAML_PATH="maps/map/test_map.yaml"
WEB_PORT="8080"
WEBSOCKET_PORT="9090"
SERVICE_PROXY_PORT="8081"
STARTUP_DELAY=1
MAP_SERVER_DELAY=2

# ROS2 Robot Bridge配置
# 注意：包现在位于工作空间内的src目录
ROBOT_BRIDGE_PACKAGE="ros2_robot_bridge"  # 包名
ROBOT_BRIDGE_WEBSOCKET_URL="ws://localhost:8439"  # 默认WebSocket URL，可根据实际情况修改

# ============================================================================
# 辅助函数
# ============================================================================

# 修复UART设备权限
fix_uart_permissions() {
    local device="$1"
    local silent="${2:-false}"
    
    if [ ! -e "$device" ]; then
        return 0
    fi
    
    if [ ! -r "$device" ] || [ ! -w "$device" ]; then
        if [ "$silent" != "true" ]; then
            echo "⚠️  UART设备权限问题检测到"
            echo "💡 运行以下命令设置权限（只需一次）:"
            echo "   ./setup_uart_permissions.sh"
            echo "   然后重新登录或运行: newgrp dialout"
            echo ""
            echo "🔄 尝试自动修复权限（可能需要sudo密码）..."
        fi
        sudo chmod 664 "$device" 2>/dev/null || true
        sudo chgrp dialout "$device" 2>/dev/null || true
        if [ -r "$device" ] && [ -w "$device" ]; then
            if [ "$silent" != "true" ]; then
                echo "✅ 权限已临时修复（建议运行setup_uart_permissions.sh永久设置）"
            fi
            return 0
        else
            if [ "$silent" != "true" ]; then
                echo "⚠️  自动修复失败，请手动运行: ./setup_uart_permissions.sh"
            fi
            return 1
        fi
    fi
    return 0
}

# 检查依赖项
check_dependencies() {
    # 检查ROS2环境
    if [ ! -f "$ROS2_SETUP_PATH" ]; then
        echo "❌ ROS2 ${ROS2_DISTRO} 未安装，请先安装ROS2"
        exit 1
    fi
    
    # 检查gnome-terminal
    if ! command -v gnome-terminal &> /dev/null; then
        echo "❌ gnome-terminal 不可用"
        echo "💡 请安装: sudo apt install gnome-terminal"
        echo "💡 或使用tmux版本: ./start_robot_tmux.sh"
        exit 1
    fi
    
    # 检查工作空间setup文件
    if [ ! -f "install/setup.bash" ]; then
        echo "⚠️  警告: install/setup.bash 不存在，某些服务可能无法启动"
    fi
    
    return 0
}

# 检查脚本文件是否存在
check_script_exists() {
    local script_path="$1"
    local service_name="$2"
    
    if [ ! -f "$script_path" ]; then
        echo "❌ 错误: ${service_name} 脚本不存在: ${script_path}"
        return 1
    fi
    return 0
}

# 检查ROS2 Robot Bridge是否已构建
# 注意：包现在位于工作空间内的src目录，使用工作空间根目录的install
check_robot_bridge() {
    local workspace_root="$1"  # 工作空间根目录（WORKSPACE_ROOT）
    local package_name="$2"
    
    # 检查包源码是否存在
    if [ ! -d "${workspace_root}/src/${package_name}" ]; then
        echo "⚠️  警告: ROS2 Robot Bridge包不存在: ${workspace_root}/src/${package_name}"
        return 1
    fi
    
    # 检查是否已构建（使用工作空间的install目录）
    if [ ! -f "${workspace_root}/install/setup.bash" ]; then
        echo "⚠️  警告: 工作空间未构建"
        echo "💡 构建命令:"
        echo "   cd ${workspace_root}"
        echo "   source /opt/ros/${ROS2_DISTRO}/setup.bash"
        echo "   colcon build --base-paths src --packages-select ${package_name}"
        echo "   source install/setup.bash"
        return 1
    fi
    
    # 检查可执行文件是否存在
    if [ ! -f "${workspace_root}/install/${package_name}/lib/${package_name}/robot_websocket_bridge" ]; then
        echo "⚠️  警告: ROS2 Robot Bridge可执行文件不存在，可能需要重新构建"
        echo "💡 构建命令:"
        echo "   cd ${workspace_root}"
        echo "   source /opt/ros/${ROS2_DISTRO}/setup.bash"
        echo "   colcon build --base-paths src --packages-select ${package_name}"
        return 1
    fi
    
    return 0
}

# 检查端口是否被占用
check_port() {
    local port="$1"
    local service_name="$2"
    
    if command -v netstat &> /dev/null; then
        if netstat -tuln 2>/dev/null | grep -q ":${port} "; then
            echo "⚠️  警告: 端口 ${port} 已被占用，${service_name} 可能无法启动"
            return 1
        fi
    elif command -v ss &> /dev/null; then
        if ss -tuln 2>/dev/null | grep -q ":${port} "; then
            echo "⚠️  警告: 端口 ${port} 已被占用，${service_name} 可能无法启动"
            return 1
        fi
    fi
    return 0
}

# 启动服务（通用函数）
# 参数: title working_dir needs_ros2 user_commands
# user_commands是包含换行符的字符串
launch_service() {
    local title="$1"
    local working_dir="$2"
    local needs_ros2="$3"
    local user_commands="$4"
    
    # 构建完整命令字符串
    local full_cmd=""
    
    # 工作目录
    if [ -n "$working_dir" ]; then
        full_cmd="cd '${working_dir}'"
    fi
    
    # ROS2环境设置
    if [ "$needs_ros2" = "true" ]; then
        if [ -n "$full_cmd" ]; then
            full_cmd="${full_cmd}"$'\n'"source ${ROS2_SETUP_PATH}"
        else
            full_cmd="source ${ROS2_SETUP_PATH}"
        fi
        full_cmd="${full_cmd}"$'\n'"source install/setup.bash"
    fi
    
    # 用户命令
    if [ -n "$user_commands" ]; then
        if [ -n "$full_cmd" ]; then
            full_cmd="${full_cmd}"$'\n'"${user_commands}"
        else
            full_cmd="${user_commands}"
        fi
    fi
    
    # 保持窗口打开
    full_cmd="${full_cmd}"$'\n'"bash"
    
    # 启动gnome-terminal
    gnome-terminal --title="${title}" -- bash -c "${full_cmd}" &
}

# 清理函数（信号处理）
cleanup() {
    echo ""
    echo "🛑 正在关闭服务..."
    # 可以在这里添加清理逻辑，如kill特定进程
    exit 0
}

# 注册信号处理
trap cleanup SIGINT SIGTERM

# ============================================================================
# 主程序
# ============================================================================

echo "🤖 启动机器人控制系统 (简化Tab版本)"
echo "================================="

# 获取脚本所在目录，然后获取工作空间根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# 设置工作目录
cd "$WORKSPACE_ROOT"

# 检查依赖项
check_dependencies

echo "✅ 检查ROS2环境..."

# 检查并修复UART权限（如果设备存在）
fix_uart_permissions "$UART_DEVICE"

echo "🚀 启动服务..."

# ============================================================================
# 启动各个服务
# ============================================================================

# Tab 1: ROS2机器人系统
echo "📱 启动Tab 1: ROS2机器人系统"
if [ -f "${WORKSPACE_ROOT}/${MAP_YAML_PATH}" ]; then
    cmd1="echo '🤖 启动ROS2机器人系统...'
echo '🗺️  启动地图服务器...'
# 启动地图服务器
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=\"${WORKSPACE_ROOT}/${MAP_YAML_PATH}\" &
sleep ${MAP_SERVER_DELAY}
# 启动机器人系统（包含修复版AprilTag检测）
# 注意：超声波传感器节点在Tab 8中单独启动
ros2 launch mecanum_robot apriltag_robot_fixed.launch.py use_ultrasonic:=false"
    launch_service "ROS2机器人系统" "" "true" "$cmd1"
else
    echo "⚠️  警告: 地图文件不存在，但继续启动机器人系统"
    cmd1b="echo '🤖 启动ROS2机器人系统...'
ros2 launch mecanum_robot apriltag_robot_fixed.launch.py use_ultrasonic:=false"
    launch_service "ROS2机器人系统" "" "true" "$cmd1b"
fi

sleep ${STARTUP_DELAY}

# Tab 2: ROS2 WebSocket Bridge
echo "📱 启动Tab 2: ROS2 WebSocket Bridge"
if check_script_exists "scripts/ros2_websocket_bridge.py" "ROS2 WebSocket Bridge"; then
    check_port "${WEBSOCKET_PORT}" "ROS2 WebSocket Bridge"
    cmd2="echo '🌉 启动ROS2 WebSocket Bridge...'
python3 scripts/ros2_websocket_bridge.py"
    launch_service "ROS2 WebSocket Bridge" "" "true" "$cmd2"
fi

sleep ${STARTUP_DELAY}

# Tab 3: Waypoint录制器
echo "📱 启动Tab 3: Waypoint录制器"
if check_script_exists "scripts/waypoint_recorder.py" "Waypoint录制器"; then
    cmd3="echo '🎯 启动Waypoint录制器...'
python3 waypoint_recorder.py"
    launch_service "Waypoint录制器" "scripts" "true" "$cmd3"
fi

sleep ${STARTUP_DELAY}

# Tab 4: ROS2服务代理
echo "📱 启动Tab 4: ROS2服务代理"
if check_script_exists "scripts/ros2_service_proxy.py" "ROS2服务代理"; then
    check_port "${SERVICE_PROXY_PORT}" "ROS2服务代理"
    cmd4="echo '🔧 启动ROS2服务代理...'
python3 scripts/ros2_service_proxy.py"
    launch_service "ROS2服务代理" "" "true" "$cmd4"
fi

sleep ${STARTUP_DELAY}

# Tab 5: Waypoint跟踪器
echo "📱 启动Tab 5: Waypoint跟踪器"
if check_script_exists "scripts/simple_waypoint_follower.py" "Waypoint跟踪器"; then
    cmd5="echo '🎯 启动Waypoint跟踪器...'
echo '📁 启动时不加载任何waypoint文件，等待用户选择'
python3 simple_waypoint_follower.py"
    launch_service "Waypoint跟踪器" "scripts" "true" "$cmd5"
fi

sleep ${STARTUP_DELAY}

# Tab 6: 机器人状态机
echo "📱 启动Tab 6: 机器人状态机"
if check_script_exists "scripts/robot_state_machine.py" "机器人状态机"; then
    cmd6="echo '🤖 启动机器人状态机...'
python3 scripts/robot_state_machine.py"
    launch_service "机器人状态机" "" "true" "$cmd6"
fi

sleep ${STARTUP_DELAY}

# Tab 7: Web控制台
echo "📱 启动Tab 7: Web控制台"
check_port "${WEB_PORT}" "Web控制台"
cmd7="echo '🌐 启动Web控制台...'
python3 -m http.server ${WEB_PORT}"
launch_service "Web控制台" "web" "false" "$cmd7"

sleep ${STARTUP_DELAY}

# Tab 8: 超声波传感器节点
echo "📱 启动Tab 8: 超声波传感器节点"
# 预先修复权限（如果需要）
fix_uart_permissions "$UART_DEVICE" "true"
if [ -e "$UART_DEVICE" ] && ([ -r "$UART_DEVICE" ] || [ -w "$UART_DEVICE" ] || fix_uart_permissions "$UART_DEVICE" "true"); then
    cmd8="echo '📡 启动超声波传感器节点...'
echo '🔌 串口: ${UART_DEVICE} (Jetson UART1)'
echo '⚡ 波特率: ${UART_BAUDRATE}'
# 如果权限仍然有问题，尝试再次修复
if [ ! -r ${UART_DEVICE} ] || [ ! -w ${UART_DEVICE} ]; then
    echo '⚠️  权限问题，尝试修复...'
    sudo chmod 664 ${UART_DEVICE} 2>/dev/null || true
    sudo chgrp dialout ${UART_DEVICE} 2>/dev/null || true
fi
ros2 run mecanum_robot ultrasonic_sensor_node.py --ros-args -p serial_port:=${UART_DEVICE} -p baudrate:=${UART_BAUDRATE}"
    launch_service "超声波传感器" "" "true" "$cmd8"
else
    echo "⚠️  警告: UART设备 ${UART_DEVICE} 权限问题，跳过超声波传感器节点"
fi

sleep ${STARTUP_DELAY}

# Tab 9: ROS2 Robot Bridge
echo "📱 启动Tab 9: ROS2 Robot Bridge"
if check_robot_bridge "$WORKSPACE_ROOT" "$ROBOT_BRIDGE_PACKAGE"; then
    cmd9="echo '🌉 启动ROS2 Robot Bridge...'
echo '🔗 WebSocket URL: ${ROBOT_BRIDGE_WEBSOCKET_URL}'
echo '📡 订阅: /cmd_vel (geometry_msgs/Twist)'
echo '📤 发布: /odom (nav_msgs/Odometry)'
echo '📦 包路径: ${WORKSPACE_ROOT}/src/${ROBOT_BRIDGE_PACKAGE}'
# 注意：工作空间的install/setup.bash已经在launch_service中source了
# 启动ROS2 Robot Bridge
ros2 run ${ROBOT_BRIDGE_PACKAGE} robot_websocket_bridge --ros-args -p websocket_url:=\"${ROBOT_BRIDGE_WEBSOCKET_URL}\""
    launch_service "ROS2 Robot Bridge" "" "true" "$cmd9"
else
    echo "⚠️  警告: ROS2 Robot Bridge未构建或路径不正确，跳过启动"
fi

# 等待服务启动
echo "⏳ 等待服务启动..."
sleep ${STARTUP_DELAY}

echo "✅ 系统启动完成！"
echo ""
echo "🔗 访问地址:"
echo "   Web控制台: http://localhost:${WEB_PORT}"
echo "   ROS2 Web Bridge: ws://localhost:${WEBSOCKET_PORT}"
echo "   ROS2服务代理: http://localhost:${SERVICE_PROXY_PORT}"
echo ""
echo "📊 系统状态:"
echo "   - ROS2机器人系统: 运行中 (独立窗口)"
echo "   - ROS2 WebSocket Bridge: 运行中 (独立窗口)"
echo "   - Waypoint录制器: 运行中 (独立窗口)"
echo "   - ROS2服务代理: 运行中 (独立窗口)"
echo "   - Waypoint跟踪器: 运行中 (独立窗口)"
echo "   - 机器人状态机: 运行中 (独立窗口)"
echo "   - Web控制台: 运行中 (独立窗口)"
echo "   - 超声波传感器节点: 运行中 (独立窗口)"
echo "   - ROS2 Robot Bridge: 运行中 (独立窗口)"
echo ""
echo "💡 使用说明:"
echo "   1. 打开浏览器访问 http://localhost:${WEB_PORT}"
echo "   2. 使用WASD键或摇杆控制机器人"
echo "   3. 切换到'Waypoint导航'Tab进行路径录制和跟踪"
echo "   4. 观察摄像头画面和AprilTag检测"
echo "   5. 查看超声波传感器数据（8个传感器，编号1-8）"
echo "   6. 查看机器人状态机显示当前运行状态"
echo "   7. 所有服务都在独立的terminal窗口中运行"
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
