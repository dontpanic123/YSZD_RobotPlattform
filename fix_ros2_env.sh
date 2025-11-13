#!/bin/bash

echo "🔧 修复ROS2环境变量"
echo "=================="

# 设置所有必要的环境变量
export AMENT_TRACE_SETUP_FILES=""
export AMENT_PYTHON_EXECUTABLE=""
export COLCON_TRACE=""
export COLCON_PREFIX_PATH=""

# 获取脚本所在目录，然后获取工作空间根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"

# 设置工作目录
cd "$WORKSPACE_ROOT"

# 加载ROS2环境
echo "📦 加载ROS2环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "✅ 环境设置完成！"

# 测试ROS2命令
echo "🧪 测试ROS2命令..."
ros2 --version

echo "🚀 现在可以启动Nav2系统了！"















