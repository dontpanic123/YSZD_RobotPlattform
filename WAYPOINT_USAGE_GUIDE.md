# 🎯 Waypoint系统使用指南

## 📖 概述

Waypoint系统是一个完整的机器人路径录制和自动导航解决方案，支持：
- **路径录制**: 手动控制机器人移动，自动记录关键路径点
- **自动导航**: 使用录制的路径点进行自动导航
- **Web界面**: 通过浏览器进行可视化控制
- **Nav2集成**: 与ROS2 Nav2导航系统完全兼容

## 🚀 快速开始

### 1. 启动完整系统
```bash
# 启动所有服务（推荐）
cd /home/bd/Documents/Robot/agv_sim/web
./start_complete_system.sh

# 或者使用tmux版本（更稳定）
./start_robot_tmux.sh
```

### 2. 访问Web界面
打开浏览器访问：http://localhost:8080

### 3. 切换到Waypoint导航Tab
在Web界面中点击"Waypoint导航"标签页

## 🎮 使用方法

### 📹 录制路径

#### 方法1: 通过Web界面
1. 在"Waypoint导航"Tab中
2. 输入路径名称（例如："办公室巡逻"）
3. 点击"开始录制"按钮
4. 使用摇杆或键盘控制机器人移动
5. 点击"停止录制"按钮
6. 点击"保存Waypoints"按钮

#### 方法2: 通过命令行
```bash
# 启动waypoint recorder
cd /home/bd/Documents/Robot/agv_sim/scripts
./start_waypoint_recorder.sh

# 开始录制
ros2 service call /start_recording std_srvs/srv/Empty

# 停止录制
ros2 service call /stop_recording std_srvs/srv/Empty

# 保存waypoints
ros2 service call /save_waypoints std_srvs/srv/Empty
```

### 🎯 自动导航

#### 方法1: 通过Web界面
1. 在"Waypoint导航"Tab中
2. 在"已保存路径"列表中选择要跟踪的路径
3. 点击"开始跟踪"按钮
4. 机器人将自动按照录制的路径移动

#### 方法2: 通过命令行
```bash
# 启动waypoint follower
cd /home/bd/Documents/Robot/agv_sim/scripts
./start_waypoint_follower.sh /home/bd/Documents/Robot/agv_sim/waypoints/your_waypoints.json
```

## 🔧 高级功能

### 参数设置

#### 录制参数
- **距离阈值**: 0.5米（两个waypoint之间的最小距离）
- **角度阈值**: 0.2弧度（两个waypoint之间的最小角度差）
- **里程计话题**: `/odom`

#### 导航参数
- **Waypoint超时**: 30秒
- **循环模式**: 关闭
- **速度限制**: 根据机器人配置

### 文件格式

#### Waypoints文件结构
```json
{
  "waypoints": [
    {
      "x": 1.0,
      "y": 2.0,
      "z": 0.0,
      "yaw": 0.5,
      "timestamp": 1640995200.0
    }
  ],
  "metadata": {
    "name": "办公室巡逻",
    "created_at": "2024-01-01T12:00:00Z",
    "total_distance": 10.5,
    "waypoint_count": 15
  }
}
```

### 服务接口

#### ROS2服务
- `/start_recording` - 开始录制
- `/stop_recording` - 停止录制
- `/save_waypoints` - 保存waypoints
- `/start_following` - 开始跟踪
- `/stop_following` - 停止跟踪

#### 话题接口
- `/odom` - 里程计数据（输入）
- `/recorded_path` - 录制的路径（可视化）
- `/follow_waypoints/feedback` - 跟踪反馈
- `/follow_waypoints/result` - 跟踪结果

## 📊 监控和调试

### 查看录制状态
```bash
# 查看录制状态
ros2 topic echo /recorded_path

# 查看服务状态
ros2 service list | grep waypoint
```

### 查看导航状态
```bash
# 查看跟踪反馈
ros2 topic echo /follow_waypoints/feedback

# 查看跟踪结果
ros2 topic echo /follow_waypoints/result
```

### 可视化工具
```bash
# 启动RViz可视化
rviz2 -d /home/bd/Documents/Robot/agv_sim/rviz/mecanum_robot.rviz
```

## 🛠️ 故障排除

### 常见问题

#### 1. 录制没有响应
- 检查ROS2环境是否正确设置
- 确认机器人系统正在运行
- 检查里程计话题是否发布数据

#### 2. 导航失败
- 确认Nav2系统正在运行
- 检查机器人是否已定位
- 验证waypoints文件格式是否正确

#### 3. Web界面无响应
- 检查WebSocket连接状态
- 确认ROS2 Bridge正在运行
- 查看浏览器控制台错误信息

### 调试命令
```bash
# 检查ROS2节点
ros2 node list

# 检查服务
ros2 service list

# 检查话题
ros2 topic list

# 查看节点信息
ros2 node info /waypoint_recorder
```

## 📁 文件结构

```
/home/bd/Documents/Robot/agv_sim/
├── scripts/
│   ├── waypoint_recorder.py      # 录制节点
│   ├── waypoint_follower.py      # 跟踪节点
│   ├── start_waypoint_recorder.sh
│   ├── start_waypoint_follower.sh
│   └── start_nav2_waypoint_system.sh
├── waypoints/                    # 保存的waypoints文件
│   ├── office_patrol.json
│   └── warehouse_route.json
└── web/
    ├── js/waypoint-system.js     # Web界面集成
    └── start_complete_system.sh  # 完整系统启动
```

## 🎯 最佳实践

### 录制建议
1. **缓慢移动**: 录制时保持缓慢、平稳的移动
2. **关键点**: 在重要的转弯点和目标点停留
3. **测试路径**: 录制后先测试路径的可行性
4. **命名规范**: 使用描述性的路径名称

### 导航建议
1. **环境检查**: 确保导航环境与录制时一致
2. **定位确认**: 确保机器人已正确定位
3. **障碍物**: 注意动态障碍物的影响
4. **监控**: 在导航过程中保持监控

## 🔄 工作流程示例

### 完整的巡逻任务
1. **启动系统**: `./start_complete_system.sh`
2. **录制路径**: 在Web界面中录制巡逻路径
3. **保存路径**: 保存为"夜间巡逻"路径
4. **测试路径**: 先手动测试路径可行性
5. **自动导航**: 使用保存的路径进行自动巡逻
6. **监控执行**: 通过Web界面监控执行状态

### 多路径管理
1. **创建不同路径**: 办公室、仓库、清洁路线
2. **路径分类**: 按功能或区域分类
3. **调度系统**: 根据时间或条件选择路径
4. **路径优化**: 定期更新和优化路径

## 📞 技术支持

如果遇到问题，请检查：
1. ROS2环境设置
2. 机器人系统状态
3. Web界面连接
4. 日志文件内容

更多技术细节请参考：
- `WAYPOINT_WEB_INTEGRATION.md`
- `WAYPOINT_INTEGRATION_SUMMARY.md`
- `TERMINAL_TAB_GUIDE.md`
