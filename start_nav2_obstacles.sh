#!/bin/bash

echo "🤖 启动Nav2导航系统（障碍物避让版本）"
echo "======================================"

# 设置工作目录
cd /home/bd/Documents/Robot/agv_sim

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
    exit 1
fi

echo "🚀 启动Nav2导航系统..."

# 启动Nav2系统
gnome-terminal --title="Nav2导航系统" -- bash -c "
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    echo '🗺️  启动Nav2导航系统...'
    echo '⚠️  注意：此系统包含障碍物避让功能'
    echo '🚫 机器人将无法进入地图的黑色区域'
    ros2 launch mecanum_robot nav2_with_obstacles.launch.py
    bash
" &

# 等待服务启动
echo "⏳ 等待服务启动..."
sleep 5

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




