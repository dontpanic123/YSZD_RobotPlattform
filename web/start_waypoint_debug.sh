#!/bin/bash

echo "🔧 启动Waypoint Tab调试"
echo "======================"

# 设置工作目录
cd /home/bd/Documents/Robot/agv_sim/web

# 停止现有的Web服务器
echo "🛑 停止现有Web服务器..."
pkill -f "python3 -m http.server" 2>/dev/null || true
sleep 1

# 启动新的Web服务器
echo "🚀 启动Web服务器..."
python3 -m http.server 8080 &
sleep 2

echo "✅ Web服务器已启动"
echo ""
echo "🌐 测试页面地址:"
echo "   主页面: http://localhost:8080"
echo "   调试页面: http://localhost:8080/debug_waypoint_tab.html"
echo "   简单测试: http://localhost:8080/test_waypoint_simple.html"
echo ""

echo "🔧 测试步骤:"
echo "   1. 打开浏览器访问上述地址"
echo "   2. 按F12打开开发者工具"
echo "   3. 切换到Console标签页"
echo "   4. 点击'Waypoint导航'Tab"
echo "   5. 观察控制台输出"
echo ""

echo "📊 检查项目:"
echo "   - 是否有JavaScript错误"
echo "   - Tab点击事件是否触发"
echo "   - 元素是否正确找到"
echo "   - CSS样式是否正确应用"
echo ""

echo "🛑 停止调试:"
echo "   pkill -f 'python3 -m http.server'"
