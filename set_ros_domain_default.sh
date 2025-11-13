#!/bin/bash

# 设置ROS2域ID为默认值(0)的脚本

echo "🔧 设置ROS2域ID为默认值..."

# 取消设置任何现有的ROS_DOMAIN_ID
unset ROS_DOMAIN_ID

# 明确设置为0 (默认值)
export ROS_DOMAIN_ID=0

echo "✅ ROS_DOMAIN_ID已设置为: $ROS_DOMAIN_ID"

# 验证设置
echo "🔍 验证设置:"
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "   ROS_DISTRO: $ROS_DISTRO"

# 检查话题
echo "📋 当前可用话题:"
ros2 topic list

echo ""
echo "💡 提示: 将此脚本添加到您的 ~/.bashrc 中以永久设置:"
# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "   echo 'source $SCRIPT_DIR/set_ros_domain_default.sh' >> ~/.bashrc"
