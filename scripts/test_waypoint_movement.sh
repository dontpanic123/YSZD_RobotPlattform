#!/bin/bash

echo "🧪 测试Waypoint移动"
echo "=================="

# 设置工作目录
cd /home/bd/Documents/Robot/agv_sim

echo "🔍 检查系统状态..."

# 检查ROS2话题
echo "📊 ROS2话题状态:"
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

# 检查waypoint follower服务
echo ""
echo "📊 Waypoint服务状态:"
if ros2 service list | grep -q "/start_following"; then
    echo "   ✅ /start_following 服务存在"
else
    echo "   ❌ /start_following 服务不存在"
fi

if ros2 service list | grep -q "/stop_following"; then
    echo "   ✅ /stop_following 服务存在"
else
    echo "   ❌ /stop_following 服务不存在"
fi

# 检查waypoints文件
echo ""
echo "📁 Waypoints文件状态:"
LATEST_FILE=$(ls -t waypoints/*.json 2>/dev/null | head -1)
if [ -n "$LATEST_FILE" ]; then
    echo "   ✅ 找到waypoints文件: $LATEST_FILE"
    WAYPOINT_COUNT=$(grep -c '"id":' "$LATEST_FILE")
    echo "   📊 Waypoint数量: $WAYPOINT_COUNT"
else
    echo "   ❌ 没有找到waypoints文件"
fi

echo ""
echo "🎮 测试步骤:"
echo "   1. 开始跟踪waypoints"
echo "   2. 监控机器人移动"
echo "   3. 停止跟踪"
echo ""

# 开始跟踪
echo "🚀 开始跟踪waypoints..."
ros2 service call /start_following std_srvs/srv/Empty

echo "✅ 跟踪已启动！"
echo ""
echo "📊 监控命令:"
echo "   # 查看机器人速度"
echo "   ros2 topic echo /cmd_vel"
echo ""
echo "   # 查看机器人位置"
echo "   ros2 topic echo /odom"
echo ""
echo "   # 停止跟踪"
echo "   ros2 service call /stop_following std_srvs/srv/Empty"
echo ""

# 监控5秒钟
echo "⏱️ 监控5秒钟..."
for i in {1..5}; do
    echo "   第 $i 秒..."
    sleep 1
done

echo ""
echo "🛑 停止跟踪..."
ros2 service call /stop_following std_srvs/srv/Empty

echo "✅ 测试完成！"
echo ""
echo "💡 如果机器人移动了，说明waypoint重播功能正常工作"
echo "💡 如果机器人没有移动，请检查："
echo "   - 机器人系统是否完全启动"
echo "   - waypoints文件是否正确"
echo "   - 机器人是否已定位"
