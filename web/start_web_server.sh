#!/bin/bash

echo "🌐 启动机器人Web控制台"
echo "========================"

# 检查Python是否安装
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 未安装，请先安装Python3"
    exit 1
fi

# 检查端口是否被占用
PORT=8080
if lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null ; then
    echo "⚠️  端口 $PORT 已被占用，尝试使用端口 8081"
    PORT=8081
fi

# 设置工作目录
cd "$(dirname "$0")"

echo "📁 工作目录: $(pwd)"
echo "🌐 启动Web服务器在端口 $PORT"
echo "🔗 访问地址: http://localhost:$PORT"

# 启动简单的HTTP服务器
echo "🚀 启动Web服务器..."
python3 -m http.server $PORT

echo "👋 Web服务器已停止"












