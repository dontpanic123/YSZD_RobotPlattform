#!/bin/bash

echo "🧪 测试Gnome Terminal Tab功能"
echo "============================="

# 测试gnome-terminal的tab功能
echo "🚀 启动测试terminal..."

gnome-terminal \
    --title="测试Terminal" \
    --tab --title="测试Tab1" -- bash -c "
        echo '🧪 这是测试Tab 1'
        echo '时间: ' \$(date)
        sleep 5
        echo 'Tab 1 完成'
        bash
    " \
    --tab --title="测试Tab2" -- bash -c "
        echo '🧪 这是测试Tab 2'
        echo '时间: ' \$(date)
        sleep 5
        echo 'Tab 2 完成'
        bash
    " \
    --tab --title="测试Tab3" -- bash -c "
        echo '🧪 这是测试Tab 3'
        echo '时间: ' \$(date)
        sleep 5
        echo 'Tab 3 完成'
        bash
    " &

echo "✅ 测试terminal已启动！"
echo "📱 检查是否有新的terminal窗口打开，包含3个tab"
echo "🛑 关闭terminal窗口停止测试"

# 等待用户中断
wait

