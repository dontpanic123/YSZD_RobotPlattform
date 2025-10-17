#!/bin/bash

# 旋转箭头控制测试启动脚本

echo "🔄 启动旋转箭头控制测试..."

# 检查是否在正确的目录
if [ ! -f "index.html" ]; then
    echo "❌ 请在web目录下运行此脚本"
    exit 1
fi

echo "🎮 旋转箭头控制功能说明:"
echo "   - 摇杆左右两侧新增蓝色旋转箭头按钮"
echo "   - 支持鼠标点击控制旋转"
echo "   - 与键盘控制(Q/E)和滑块控制兼容"
echo "   - 实时显示旋转状态和速度"
echo "   - 蓝色风格与摇杆统一设计"
echo ""
echo "📍 访问地址:"
echo "   主页面: http://localhost:8080/index.html"
echo "   测试页面: http://localhost:8080/test_rotation_arrows.html"
echo ""
echo "🧪 测试步骤:"
echo "   1. 点击左右旋转箭头按钮"
echo "   2. 观察按钮激活状态"
echo "   3. 检查速度显示更新"
echo "   4. 测试与键盘控制的兼容性"
echo ""
echo "按 Ctrl+C 停止服务器"

# 启动Python HTTP服务器
python3 -m http.server 8080
