#!/bin/bash

echo "🧪 完整Waypoint测试"
echo "=================="

# 设置工作目录
cd /home/bd/Documents/Robot/agv_sim

# 检查ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "✅ ROS2环境已设置"

# 检查系统状态
echo "🔍 检查系统状态..."

# 检查机器人节点
echo "🤖 机器人节点:"
ros2 node list | grep -E "(robot|mecanum)" || echo "   没有找到机器人节点"

# 检查话题
echo "📊 话题状态:"
if ros2 topic list | grep -q "/cmd_vel"; then
    echo "   ✅ /cmd_vel 话题存在"
else
    echo "   ❌ /cmd_vel 话题不存在"
fi

if ros2 topic list | grep -q "/odom"; then
    echo "   ✅ /odom 话题存在"
else
    echo "   ❌ /odom 话题不存在"
fi

# 检查waypoint follower
echo "🎯 Waypoint Follower状态:"
if ros2 service list | grep -q "/start_following"; then
    echo "   ✅ /start_following 服务存在"
else
    echo "   ❌ /start_following 服务不存在"
fi

# 检查waypoints文件
echo "📁 Waypoints文件:"
WAYPOINTS_FILE="/home/bd/Documents/Robot/agv_sim/waypoints/waypoints_20251008_152959.json"
if [ -f "$WAYPOINTS_FILE" ]; then
    echo "   ✅ Waypoints文件存在"
    WAYPOINT_COUNT=$(grep -c '"id":' "$WAYPOINTS_FILE")
    echo "   📊 Waypoint数量: $WAYPOINT_COUNT"
else
    echo "   ❌ Waypoints文件不存在"
    exit 1
fi

echo ""
echo "🎮 开始Waypoint跟踪测试..."

# 开始跟踪
echo "🚀 开始跟踪waypoints..."
ros2 service call /start_following std_srvs/srv/Empty

# 监控5秒钟
echo "⏱️ 监控5秒钟..."
for i in {1..5}; do
    echo "   第 $i 秒..."
    
    # 检查速度命令
    echo "   检查速度命令..."
    timeout 1 ros2 topic echo /cmd_vel --once 2>/dev/null || echo "   没有速度命令"
    
    sleep 1
done

# 停止跟踪
echo "🛑 停止跟踪..."
ros2 service call /stop_following std_srvs/srv/Empty

echo ""
echo "✅ 测试完成！"
echo ""
echo "💡 如果看到速度命令输出，说明waypoint跟踪正常工作"
echo "💡 如果机器人实际移动了，说明整个系统正常工作"
echo ""
echo "📊 手动监控命令:"
echo "   # 查看速度命令"
echo "   ros2 topic echo /cmd_vel"
echo ""
echo "   # 查看机器人位置"
echo "   ros2 topic echo /odom"
echo ""
echo "   # 重新开始跟踪"
echo "   ros2 service call /start_following std_srvs/srv/Empty"
