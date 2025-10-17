#!/bin/bash

echo "🧪 启动Tab切换测试"
echo "=================="

# 设置工作目录
cd /home/bd/Documents/Robot/agv_sim/web

# 检查是否有Web服务器在运行
if pgrep -f "python3 -m http.server" > /dev/null; then
    echo "✅ Web服务器已在运行"
else
    echo "🚀 启动Web服务器..."
    python3 -m http.server 8081 &
    sleep 2
fi

echo "🌐 测试页面地址:"
echo "   主页面: http://localhost:8081"
echo "   调试页面: http://localhost:8081/debug_tab_issue.html"
echo "   简单测试: http://localhost:8081/test_simple_tab.html"
echo ""

echo "🔧 测试步骤:"
echo "   1. 打开浏览器访问上述地址"
echo "   2. 打开浏览器开发者工具 (F12)"
echo "   3. 查看Console标签页的日志"
echo "   4. 点击'Waypoint导航'Tab"
echo "   5. 检查是否有错误信息"
echo ""

echo "📊 检查项目:"
echo "   - Tab按钮是否正确找到"
echo "   - waypoint-tab元素是否存在"
echo "   - Tab切换事件是否触发"
echo "   - CSS样式是否正确应用"
echo ""

echo "🛑 停止测试:"
echo "   pkill -f 'python3 -m http.server'"
