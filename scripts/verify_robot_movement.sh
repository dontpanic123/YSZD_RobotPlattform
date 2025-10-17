#!/bin/bash

echo "🔍 验证机器人移动"
echo "================"

# 设置工作目录
cd /home/bd/Documents/Robot/agv_sim

echo "📊 检查机器人状态..."

# 检查机器人节点
echo "🤖 机器人节点状态:"
ros2 node list | grep -E "(robot|mecanum)" || echo "   没有找到机器人节点"

# 检查话题发布频率
echo ""
echo "📊 话题发布频率:"
echo "   /cmd_vel 发布频率:"
timeout 3 ros2 topic hz /cmd_vel 2>/dev/null || echo "   无法获取频率信息"

echo "   /odom 发布频率:"
timeout 3 ros2 topic hz /odom 2>/dev/null || echo "   无法获取频率信息"

# 检查当前速度
echo ""
echo "📊 当前机器人速度:"
timeout 2 ros2 topic echo /cmd_vel --once 2>/dev/null || echo "   无法获取速度信息"

# 检查当前位置
echo ""
echo "📊 当前机器人位置:"
timeout 2 ros2 topic echo /odom --once 2>/dev/null || echo "   无法获取位置信息"

echo ""
echo "🎮 手动测试机器人移动:"
echo "   发送前进命令..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

sleep 1

echo "   发送停止命令..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

echo ""
echo "✅ 测试完成！"
echo ""
echo "💡 如果机器人移动了，说明控制系统正常工作"
echo "💡 如果机器人没有移动，可能的原因："
echo "   - 机器人硬件未连接"
echo "   - 机器人驱动未启动"
echo "   - 安全开关未打开"
echo "   - 机器人处于紧急停止状态"
