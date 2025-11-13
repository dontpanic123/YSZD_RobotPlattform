# Nav2 Waypoint 系统

这是一个完整的Nav2 Waypoint录制和跟踪系统，可以录制机器人的运动轨迹并生成Nav2兼容的waypoints文件。

## 🎯 功能特性

### Waypoint Recorder
- **自动录制**: 订阅机器人里程计数据，自动生成waypoints
- **智能过滤**: 基于距离和角度阈值，避免冗余waypoints
- **实时可视化**: 发布路径用于RViz可视化
- **服务控制**: 通过ROS2服务控制录制开始/停止/保存
- **Nav2兼容**: 生成标准Nav2格式的waypoints文件

### Waypoint Follower
- **自动跟踪**: 使用Nav2的FollowWaypoints Action跟踪waypoints
- **循环模式**: 支持循环跟踪waypoints
- **状态监控**: 实时监控跟踪状态和进度
- **错误处理**: 处理missed waypoints和超时情况

## 📁 文件结构

```
scripts/
├── waypoint_recorder.py          # Waypoint录制节点
├── waypoint_follower.py          # Waypoint跟踪节点
├── start_waypoint_recorder.sh    # 录制器启动脚本
├── start_waypoint_follower.sh    # 跟踪器启动脚本
└── start_nav2_waypoint_system.sh # 完整系统启动脚本

waypoints/                        # Waypoints文件输出目录
├── waypoints_20241201_143022.json
├── waypoints_20241201_143156.json
└── ...
```

## 🚀 快速开始

### 1. 录制Waypoints

```bash
# 启动录制器
cd /home/bd/Documents/Robot/agv_sim/scripts
./start_waypoint_recorder.sh
```

**录制步骤:**
1. 启动机器人系统
2. 手动控制机器人移动
3. 使用服务开始录制
4. 停止录制并保存

**服务命令:**
```bash
# 开始录制
ros2 service call /start_recording std_srvs/srv/Empty

# 停止录制
ros2 service call /stop_recording std_srvs/srv/Empty

# 保存waypoints
ros2 service call /save_waypoints std_srvs/srv/Empty
```

### 2. 跟踪Waypoints

```bash
# 启动跟踪器
cd /home/bd/Documents/Robot/agv_sim/scripts
./start_waypoint_follower.sh /path/to/waypoints.json
```

**跟踪步骤:**
1. 确保Nav2系统正在运行
2. 确保机器人已定位
3. 启动waypoint follower
4. 机器人将自动跟踪waypoints

### 3. 完整系统

```bash
# 启动完整系统
cd /home/bd/Documents/Robot/agv_sim/scripts
./start_nav2_waypoint_system.sh both
```

## ⚙️ 参数配置

### Waypoint Recorder 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `recording` | `false` | 是否开始录制 |
| `waypoint_distance_threshold` | `0.5` | 距离阈值(米) |
| `waypoint_angle_threshold` | `0.2` | 角度阈值(弧度) |
| `output_directory` | `/home/bd/Documents/Robot/agv_sim/waypoints` | 输出目录 |
| `topic_odom` | `/odom` | 里程计话题 |

### Waypoint Follower 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `waypoints_file` | `""` | Waypoints文件路径 |
| `loop_waypoints` | `false` | 是否循环跟踪 |
| `waypoint_timeout` | `30.0` | Waypoint超时时间(秒) |

## 📊 输出格式

### Waypoints JSON格式

```json
{
  "waypoints": [
    {
      "id": 0,
      "position": {
        "x": 1.23,
        "y": 4.56,
        "z": 0.0
      },
      "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
      },
      "timestamp": {
        "sec": 1701234567,
        "nanosec": 890123456
      }
    }
  ],
  "metadata": {
    "total_waypoints": 10,
    "recording_duration": 45.2,
    "distance_threshold": 0.5,
    "angle_threshold": 0.2,
    "created_at": "2024-12-01T14:30:22"
  }
}
```

## 🔧 高级用法

### 自定义参数启动

```bash
# 自定义距离阈值
ros2 run mecanum_robot waypoint_recorder --ros-args -p waypoint_distance_threshold:=1.0

# 自定义角度阈值
ros2 run mecanum_robot waypoint_recorder --ros-args -p waypoint_angle_threshold:=0.5

# 自定义输出目录
ros2 run mecanum_robot waypoint_recorder --ros-args -p output_directory:=/custom/path
```

### 循环跟踪

```bash
# 启动循环跟踪
ros2 run mecanum_robot waypoint_follower --ros-args -p loop_waypoints:=true
```

### 状态监控

```bash
# 查看录制状态
ros2 topic echo /recorded_path

# 查看跟踪状态
ros2 topic echo /follow_waypoints/feedback

# 查看跟踪结果
ros2 topic echo /follow_waypoints/result
```

## 🎮 与Web界面集成

这个Waypoint系统可以与现有的Web控制界面集成：

1. **录制控制**: 在Web界面添加录制按钮
2. **路径可视化**: 在Web界面显示录制的路径
3. **自动导航**: 在Web界面选择waypoints进行自动导航

## 🐛 故障排除

### 常见问题

1. **录制器无法启动**
   - 检查ROS2环境是否正确设置
   - 检查里程计话题是否存在
   - 检查输出目录权限

2. **Follower无法跟踪**
   - 检查Nav2系统是否运行
   - 检查机器人是否已定位
   - 检查waypoints文件格式

3. **Waypoints质量差**
   - 调整距离和角度阈值
   - 确保机器人移动平滑
   - 检查里程计数据质量

### 调试命令

```bash
# 检查服务状态
ros2 service list | grep waypoint

# 检查话题状态
ros2 topic list | grep waypoint

# 查看节点状态
ros2 node list | grep waypoint

# 查看参数
ros2 param list | grep waypoint
```

## 📈 性能优化

### 录制优化
- 根据机器人速度调整阈值
- 使用合适的录制频率
- 避免在静止时录制

### 跟踪优化
- 设置合适的超时时间
- 使用循环模式提高效率
- 监控跟踪状态

## 🔮 未来扩展

1. **路径规划**: 集成路径规划算法
2. **避障**: 添加动态避障功能
3. **多机器人**: 支持多机器人协调
4. **云端同步**: 支持云端waypoints管理
5. **机器学习**: 使用ML优化waypoints选择

## 📞 支持

如有问题，请检查：
1. ROS2环境设置
2. 依赖包安装
3. 文件权限
4. 网络连接

---

**Nav2 Waypoint系统** - 让机器人导航更智能！ 🚀
