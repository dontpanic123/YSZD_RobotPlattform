#!/bin/bash

echo "🧪 测试Terminal Tab功能"
echo "======================"

# 启动测试服务在一个terminal的不同tab中
echo "🚀 启动测试服务..."
gnome-terminal --title="测试服务" -- bash -c "
    # Tab 1: 测试服务1
    gnome-terminal --tab --title='测试服务1' -- bash -c '
        echo \"🧪 测试服务1启动中...\"
        echo \"服务1运行中...\"
        sleep 5
        echo \"服务1完成\"
        bash
    '
    
    # Tab 2: 测试服务2
    gnome-terminal --tab --title='测试服务2' -- bash -c '
        echo \"🧪 测试服务2启动中...\"
        echo \"服务2运行中...\"
        sleep 5
        echo \"服务2完成\"
        bash
    '
    
    # Tab 3: 测试服务3
    gnome-terminal --tab --title='测试服务3' -- bash -c '
        echo \"🧪 测试服务3启动中...\"
        echo \"服务3运行中...\"
        sleep 5
        echo \"服务3完成\"
        bash
    '
    
    echo \"✅ 所有测试服务已启动！\"
    echo \"🔗 查看terminal中的不同tab\"
    echo \"🛑 关闭terminal窗口停止所有服务\"
    
    # 保持terminal打开
    bash
" &

echo "✅ 测试服务启动完成！"
echo "📊 检查新打开的terminal窗口中的tab"
echo "🛑 关闭terminal窗口即可停止所有服务"

# 等待用户中断
wait

