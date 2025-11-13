# 🎯 Waypoint重播指南

## 📖 概述

本指南将教您如何重播已录制的waypoints，让机器人自动按照之前录制的路径移动。

## 🚀 快速开始

### 方法1: 一键重播（最简单）
```bash
cd /home/bd/Documents/Robot/agv_sim/scripts
./quick_play_waypoint.sh
```

### 方法2: 完整重播（推荐）
```bash
cd /home/bd/Documents/Robot/agv_sim/scripts
./play_waypoint.sh
```

### 方法3: 手动重播
```bash
# 1. 启动waypoint follower
cd /home/bd/Documents/Robot/agv_sim/scripts
python3 waypoint_follower.py --ros-args -p waypoints_file:="/home/bd/Documents/Robot/agv_sim/waypoints/waypoints_20251008_152959.json"

# 2. 开始重播
ros2 service call /start_following std_srvs/srv/Empty

# 3. 停止重播
ros2 service call /stop_following std_srvs/srv/Empty
```

## 🌐 Web界面重播

### 1. 启动完整系统
```bash
cd /home/bd/Documents/Robot/agv_sim/web
./start_complete_system.sh
```

### 2. 访问Web界面
- 打开浏览器访问：http://localhost:8080
- 切换到"Waypoint导航"Tab

### 3. 选择并重播路径
- 在"已保存路径"列表中选择要重播的路径
- 点击"开始跟踪"按钮
- 机器人将自动按照录制的路径移动

## 📊 监控重播状态

### 查看重播进度
```bash
# 实时监控重播状态
ros2 topic echo /follow_waypoints/feedback

# 查看重播结果
ros2 topic echo /follow_waypoints/result
```

### 查看ROS2服务状态
```bash
# 查看所有waypoint相关服务
ros2 service list | grep waypoint

# 查看waypoint follower节点
ros2 node list | grep waypoint
```

## 🔧 重播控制命令

### 基本控制
```bash
# 开始重播
ros2 service call /start_following std_srvs/srv/Empty

# 停止重播
ros2 service call /stop_following std_srvs/srv/Empty

# 暂停重播（如果支持）
ros2 service call /pause_following std_srvs/srv/Empty

# 恢复重播（如果支持）
ros2 service call /resume_following std_srvs/srv/Empty
```

### 高级控制
```bash
# 设置重播速度
ros2 param set /waypoint_follower replay_speed 1.0

# 设置循环模式
ros2 param set /waypoint_follower loop_mode false

# 设置waypoint超时
ros2 param set /waypoint_follower waypoint_timeout 30.0
```

## 📁 可用的Waypoints文件

根据您的系统，当前可用的waypoints文件：

1. **waypoints_20251008_152959.json** - 最新的录制文件（16个waypoints）
2. **example_waypoints.json** - 示例文件

### 查看waypoints文件信息
```bash
# 查看文件大小和修改时间
ls -la /home/bd/Documents/Robot/agv_sim/waypoints/

# 查看waypoints数量
grep -c '"id":' /home/bd/Documents/Robot/agv_sim/waypoints/waypoints_20251008_152959.json
```

## 🎮 重播场景示例

### 场景1: 办公室巡逻
```bash
# 使用特定的巡逻路径
./play_waypoint.sh waypoints/office_patrol.json
```

### 场景2: 仓库检查
```bash
# 使用仓库检查路径
./play_waypoint.sh waypoints/warehouse_check.json
```

### 场景3: 清洁路线
```bash
# 使用清洁路线
./play_waypoint.sh waypoints/cleaning_route.json
```

## 🛠️ 故障排除

### 常见问题

#### 1. 重播没有反应
```bash
# 检查waypoint follower是否运行
ps aux | grep waypoint_follower

# 检查ROS2服务
ros2 service list | grep waypoint

# 重启waypoint follower
pkill -f waypoint_follower
./quick_play_waypoint.sh
```

#### 2. 机器人不移动
```bash
# 检查机器人系统是否运行
ros2 node list | grep robot

# 检查导航系统
ros2 topic list | grep nav

# 检查里程计
ros2 topic echo /odom
```

#### 3. 路径偏离
```bash
# 检查机器人定位
ros2 topic echo /amcl_pose

# 检查地图
ros2 topic echo /map
```

### 调试命令
```bash
# 查看所有ROS2节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看所有服务
ros2 service list

# 查看waypoint follower日志
ros2 node info /waypoint_follower
```

## 📈 重播性能优化

### 1. 调整重播参数
```bash
# 设置更快的重播速度
ros2 param set /waypoint_follower replay_speed 1.5

# 设置更短的waypoint超时
ros2 param set /waypoint_follower waypoint_timeout 15.0
```

### 2. 监控系统资源
```bash
# 查看CPU使用率
top -p $(pgrep -f waypoint_follower)

# 查看内存使用
ps aux | grep waypoint_follower
```

## 🔄 自动化重播

### 定时重播
```bash
# 创建定时重播脚本
cat > /home/bd/Documents/Robot/agv_sim/scripts/scheduled_replay.sh << 'EOF'
#!/bin/bash
cd /home/bd/Documents/Robot/agv_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/quick_play_waypoint.sh
EOF

chmod +x /home/bd/Documents/Robot/agv_sim/scripts/scheduled_replay.sh

# 添加到crontab（每小时重播一次）
echo "0 * * * * /home/bd/Documents/Robot/agv_sim/scripts/scheduled_replay.sh" | crontab -
```

### 条件重播
```bash
# 只在特定条件下重播
if [ "$(date +%H)" -ge 9 ] && [ "$(date +%H)" -le 17 ]; then
    ./quick_play_waypoint.sh
fi
```

## 💡 最佳实践

### 1. 重播前检查
- 确保机器人已正确定位
- 检查环境是否与录制时一致
- 确认没有障碍物阻挡路径

### 2. 重播中监控
- 实时监控重播状态
- 准备随时停止重播
- 记录重播日志

### 3. 重播后验证
- 检查是否完成所有waypoints
- 验证机器人是否回到起始位置
- 分析重播性能数据

## 📞 技术支持

如果遇到问题，请检查：
1. ROS2环境设置
2. Waypoint文件格式
3. 机器人系统状态
4. 导航系统配置

更多技术细节请参考：
- `WAYPOINT_USAGE_GUIDE.md`
- `WAYPOINT_WEB_INTEGRATION.md`
- `WAYPOINT_INTEGRATION_SUMMARY.md`
