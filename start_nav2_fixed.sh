#!/bin/bash

echo "🤖 启动Nav2导航系统（修复版本）"
echo "=============================="

# 获取脚本所在目录，然后获取工作空间根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"

# 设置工作目录
cd "$WORKSPACE_ROOT"

# 停止所有现有进程
echo "🛑 停止现有进程..."
pkill -f "nav2"
pkill -f "costmap"
pkill -f "lifecycle_manager"
pkill -f "rviz2"
sleep 3

echo "✅ 检查ROS2环境..."

# 检查ROS2环境
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "❌ ROS2 Humble 未安装，请先安装ROS2"
    exit 1
fi

echo "🚀 启动Nav2导航系统..."

# 启动Nav2系统（使用官方方式）
gnome-terminal --title="Nav2导航系统" -- bash -c "
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    echo '🗺️  启动Nav2导航系统...'
    echo '⚠️  注意：此系统包含障碍物避让功能'
    echo '🚫 机器人将无法进入地图的黑色区域'
    
    # 启动地图服务器
    ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$WORKSPACE_ROOT/maps/map/test_map.yaml" &
    sleep 2
    
    # 启动TF变换
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
    sleep 1
    
    # 启动机器人节点
    ros2 run mecanum_robot robot_node.py &
    sleep 1
    
    # 启动遥控节点
    ros2 run mecanum_robot remote_control.py &
    sleep 1
    
    # 启动AMCL定位
    ros2 run nav2_amcl amcl --ros-args --params-file config/nav2_params.yaml &
    sleep 2
    
    # 启动全局代价地图
    ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args --params-file config/nav2_params.yaml &
    sleep 2
    
    # 启动局部代价地图
    ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args --params-file config/nav2_params.yaml &
    sleep 2
    
    # 启动路径规划器
    ros2 run nav2_planner planner_server --ros-args --params-file config/nav2_params.yaml &
    sleep 2
    
    # 启动控制器
    ros2 run nav2_controller controller_server --ros-args --params-file config/nav2_params.yaml &
    sleep 2
    
    # 启动行为树导航器
    ros2 run nav2_bt_navigator bt_navigator --ros-args --params-file config/nav2_params.yaml &
    sleep 2
    
    # 启动生命周期管理器
    ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args --params-file config/nav2_params.yaml &
    sleep 3
    
    # 启动RViz
    ros2 run rviz2 rviz2 -d rviz/nav2_with_obstacles.rviz
    bash
" &

# 等待服务启动
echo "⏳ 等待服务启动..."
sleep 8

echo "✅ Nav2导航系统启动完成！"
echo ""
echo "🔗 访问地址:"
echo "   RViz可视化: 自动启动"
echo "   Nav2服务: 运行中"
echo ""
echo "📊 系统功能:"
echo "   - 地图服务器: 运行中 (test_map.pgm已加载)"
echo "   - 代价地图: 运行中 (障碍物检测)"
echo "   - 路径规划: 运行中 (避开黑色区域)"
echo "   - 机器人控制: 运行中"
echo ""
echo "💡 使用说明:"
echo "   1. 在RViz中设置机器人初始位置 (2D Pose Estimate)"
echo "   2. 设置目标位置 (2D Nav Goal)"
echo "   3. 机器人将自动规划路径并避开黑色区域"
echo "   4. 黑色区域在代价地图中显示为高代价区域"
echo ""
echo "🎯 关键特性:"
echo "   - 静态障碍物避让: 地图中的黑色区域"
echo "   - 动态路径规划: 实时重新规划"
echo "   - 代价地图可视化: 红色=高代价，绿色=低代价"
echo ""
echo "🛑 关闭terminal窗口即可停止系统"















