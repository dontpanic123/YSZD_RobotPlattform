#!/bin/bash

# Setup script for Jetson UART permissions
# This script sets up udev rules and adds user to dialout group

echo "🔧 设置Jetson UART权限..."

# Check if running as root
if [ "$EUID" -eq 0 ]; then 
    echo "❌ 请不要使用sudo运行此脚本"
    exit 1
fi

# Get current user
CURRENT_USER=$(whoami)

echo "👤 当前用户: $CURRENT_USER"

# Add user to dialout group
echo "📝 添加用户到dialout组..."
sudo usermod -a -G dialout $CURRENT_USER

# Copy udev rules
echo "📋 安装udev规则..."
UDEV_RULES_FILE="99-jetson-uart.rules"
UDEV_TARGET="/etc/udev/rules.d/$UDEV_RULES_FILE"

if [ -f "$UDEV_RULES_FILE" ]; then
    sudo cp "$UDEV_RULES_FILE" "$UDEV_TARGET"
    sudo chmod 644 "$UDEV_TARGET"
    echo "✅ udev规则已安装: $UDEV_TARGET"
else
    echo "⚠️  未找到udev规则文件: $UDEV_RULES_FILE"
    echo "📝 创建udev规则..."
    sudo tee "$UDEV_TARGET" > /dev/null <<EOF
# Jetson UART devices - Allow dialout group access
KERNEL=="ttyTHS1", MODE="0664", GROUP="dialout"
KERNEL=="ttyTHS0", MODE="0664", GROUP="dialout"
KERNEL=="ttyTHS2", MODE="0664", GROUP="dialout"
KERNEL=="ttyTHS3", MODE="0664", GROUP="dialout"
EOF
    sudo chmod 644 "$UDEV_TARGET"
    echo "✅ udev规则已创建: $UDEV_TARGET"
fi

# Reload udev rules
echo "🔄 重新加载udev规则..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# Set current permissions (in case device already exists)
echo "🔧 设置当前设备权限..."
for device in /dev/ttyTHS{0..3}; do
    if [ -e "$device" ]; then
        sudo chmod 664 "$device"
        sudo chgrp dialout "$device"
        echo "✅ 已设置权限: $device"
    fi
done

echo ""
echo "✅ 权限设置完成！"
echo ""
echo "⚠️  重要提示:"
echo "   1. 您需要重新登录或运行以下命令使组权限生效:"
echo "      newgrp dialout"
echo "   2. 或者注销并重新登录系统"
echo ""
echo "💡 验证权限:"
echo "   groups  # 应该包含 'dialout'"
echo "   ls -l /dev/ttyTHS1  # 应该显示 dialout 组"
echo ""




