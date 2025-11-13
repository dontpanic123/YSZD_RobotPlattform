#!/bin/bash

# 路线录制调试启动脚本

echo "🐛 启动路线录制调试模式..."

# 检查是否在正确的目录
if [ ! -f "index.html" ]; then
    echo "❌ 请在web目录下运行此脚本"
    exit 1
fi

echo "🔍 调试功能说明:"
echo "   - 检查路线录制模块是否正确加载"
echo "   - 验证DOM元素是否存在"
echo "   - 检查事件监听器是否正确设置"
echo "   - 实时显示模块状态和调试信息"
echo ""
echo "📍 访问地址:"
echo "   主页面: http://localhost:8080/index.html"
echo "   调试页面: http://localhost:8080/debug_route_recording.html"
echo "   测试页面: http://localhost:8080/test_route_recording.html"
echo ""
echo "🔧 调试步骤:"
echo "   1. 打开浏览器开发者工具 (F12)"
echo "   2. 查看控制台日志输出"
echo "   3. 检查是否有错误信息"
echo "   4. 切换到'调试信息'tab查看详细状态"
echo "   5. 尝试点击路线录制按钮测试功能"
echo ""
echo "按 Ctrl+C 停止服务器"

# 启动Python HTTP服务器
python3 -m http.server 8080
