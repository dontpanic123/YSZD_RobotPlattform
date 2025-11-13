#!/bin/bash

# Waypoint系统集成测试启动脚本

echo "🎯 启动Waypoint系统集成测试..."

# 检查是否在正确的目录
if [ ! -f "test_waypoint_integration.html" ]; then
    echo "❌ 请在web目录下运行此脚本"
    exit 1
fi

echo "📋 测试内容:"
echo "   1. Waypoint系统模块加载"
echo "   2. 录制控制功能"
echo "   3. 跟踪控制功能"
echo "   4. 参数设置功能"
echo "   5. 路径管理功能"
echo "   6. 系统日志功能"
echo ""
echo "🔧 测试步骤:"
echo "   1. 打开测试页面"
echo "   2. 切换到功能测试Tab"
echo "   3. 运行各项测试"
echo "   4. 查看测试结果"
echo ""
echo "📊 测试输出:"
echo "   - 模块加载状态"
echo "   - 功能可用性检查"
echo "   - 界面元素验证"
echo "   - 系统集成状态"
echo ""

# 启动Web服务器
echo "🌐 启动Web服务器..."
echo "   访问地址: http://localhost:8081/test_waypoint_integration.html"
echo ""

python3 -m http.server 8081
