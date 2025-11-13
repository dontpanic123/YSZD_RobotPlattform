#!/bin/bash

# 摇杆方向测试启动脚本

echo "🎮 启动摇杆方向测试..."

# 检查是否在正确的目录
if [ ! -f "index.html" ]; then
    echo "❌ 请在web目录下运行此脚本"
    exit 1
fi

echo "🔧 修复内容说明:"
echo "   ✅ 摇杆左右移动方向已修复"
echo "   ✅ 在updateJoystickControl中添加了方向修正"
echo "   ✅ 摇杆向左拖拽 → 机器人左移"
echo "   ✅ 摇杆向右拖拽 → 机器人右移"
echo ""
echo "📍 访问地址:"
echo "   主页面: http://localhost:8080/index.html"
echo "   方向测试: http://localhost:8080/test_joystick_direction.html"
echo "   修复测试: http://localhost:8080/test_joystick_fix.html"
echo ""
echo "🧪 测试步骤:"
echo "   1. 向左拖动摇杆，观察机器人是否左移"
echo "   2. 向右拖动摇杆，观察机器人是否右移"
echo "   3. 向上拖动摇杆，观察机器人是否前进"
echo "   4. 向下拖动摇杆，观察机器人是否后退"
echo ""
echo "🎯 预期效果:"
echo "   - 摇杆向左拖拽 → 机器人左移"
echo "   - 摇杆向右拖拽 → 机器人右移"
echo "   - 摇杆向上拖拽 → 机器人前进"
echo "   - 摇杆向下拖拽 → 机器人后退"
echo ""
echo "按 Ctrl+C 停止服务器"

# 启动Python HTTP服务器
python3 -m http.server 8080
