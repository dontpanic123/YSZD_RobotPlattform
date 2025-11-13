#!/bin/bash

# Waypoint系统简单测试启动脚本

echo "🎯 启动Waypoint系统简单测试..."

# 检查是否在正确的目录
if [ ! -f "test_waypoint_simple.html" ]; then
    echo "❌ 请在web目录下运行此脚本"
    exit 1
fi

echo "📋 测试内容:"
echo "   1. Waypoint系统模块加载测试"
echo "   2. 界面元素验证测试"
echo "   3. 系统集成状态检查"
echo ""
echo "🔧 测试步骤:"
echo "   1. 打开测试页面"
echo "   2. 查看自动测试结果"
echo "   3. 手动运行额外测试"
echo "   4. 查看系统日志"
echo ""
echo "📊 测试输出:"
echo "   - 模块加载状态"
echo "   - 界面元素验证"
echo "   - 系统集成状态"
echo "   - 实时日志显示"
echo ""

# 启动Web服务器
echo "🌐 启动Web服务器..."
echo "   访问地址: http://localhost:8081/test_waypoint_simple.html"
echo ""

python3 -m http.server 8081
