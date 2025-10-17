#!/bin/bash

# 摇杆修复测试启动脚本

echo "🎮 启动摇杆修复测试..."

# 检查是否在正确的目录
if [ ! -f "index.html" ]; then
    echo "❌ 请在web目录下运行此脚本"
    exit 1
fi

echo "🔧 修复内容说明:"
echo "   ✅ 摇杆左右移动方向已修复"
echo "   ✅ 旋转按键灵敏度提高20%"
echo "   ✅ 控制频率从10Hz提升到20Hz"
echo "   ✅ 按键响应速度已优化"
echo ""
echo "📍 访问地址:"
echo "   主页面: http://localhost:8080/index.html"
echo "   测试页面: http://localhost:8080/test_joystick_fix.html"
echo ""
echo "🧪 测试步骤:"
echo "   1. 测试摇杆左右移动方向是否正确"
echo "   2. 测试旋转按键(Q/E键和箭头按钮)的灵敏度"
echo "   3. 观察按键响应的即时性"
echo "   4. 检查控制频率是否提升"
echo ""
echo "🎯 预期效果:"
echo "   - 摇杆向左拖拽应该使机器人左移"
echo "   - 摇杆向右拖拽应该使机器人右移"
echo "   - 旋转按键响应更加灵敏"
echo "   - 按键按下时立即响应"
echo ""
echo "按 Ctrl+C 停止服务器"

# 启动Python HTTP服务器
python3 -m http.server 8080
