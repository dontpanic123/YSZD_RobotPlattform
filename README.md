# 麦克纳姆轮机器人遥控系统

这是一个完整的ROS2麦克纳姆轮机器人遥控系统，支持键盘控制和RViz可视化。

## 🚀 功能特性

- **麦克纳姆轮机器人模型**：完整的URDF模型，支持全向运动
- **键盘遥控**：实时键盘控制，支持前进、后退、左移、右移、旋转
- **运动学处理**：正确的麦克纳姆轮运动学正解和逆解
- **RViz可视化**：实时显示机器人状态和运动轨迹
- **里程计发布**：提供位置和姿态信息
- **TF变换**：支持坐标系变换

## 📦 系统组件

### 节点说明
- **mecanum_robot_node**: 机器人节点，处理运动学和里程计
- **remote_control_node**: 键盘遥控节点
- **robot_state_publisher**: 机器人状态发布器
- **joint_state_publisher**: 关节状态发布器
- **rviz2**: 可视化界面

### 话题接口
- **发布话题**:
  - `/cmd_vel` (geometry_msgs/Twist): 速度命令
  - `/joint_states` (sensor_msgs/JointState): 关节状态
  - `/odom` (nav_msgs/Odometry): 里程计信息
  - `/robot_status` (std_msgs/String): 机器人状态
  - `/robot_description` (std_msgs/String): 机器人URDF描述

- **订阅话题**:
  - `/cmd_vel` (geometry_msgs/Twist): 机器人节点订阅速度命令

## 🛠️ 安装和编译

### 1. 安装依赖
```bash
sudo apt update
sudo apt install ros-humble-geometry-msgs ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-rviz2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
```

### 2. 编译项目
```bash
cd /home/bd/Documents/Robot/agv_sim
colcon build
source install/setup.bash
```

## 🎮 使用方法

### 启动完整系统
```bash
# 启动麦克纳姆轮机器人系统（包含RViz）
ros2 launch mecanum_robot mecanum_robot.launch.py

# 不启动RViz
ros2 launch mecanum_robot mecanum_robot.launch.py use_rviz:=false
```

### 单独启动组件
```bash
# 启动机器人节点
ros2 run mecanum_robot robot_node.py

# 启动遥控节点
ros2 run mecanum_robot remote_control.py

# 启动机器人状态发布器
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat urdf/mecanum_robot.urdf)"

# 启动RViz
ros2 run rviz2 rviz2 -d rviz/mecanum_robot.rviz
```

## 🎯 键盘控制

| 按键 | 功能 | 说明 |
|------|------|------|
| `w` | 前进 | X方向线性运动 |
| `s` | 后退 | X方向线性运动 |
| `a` | 左移 | Y方向线性运动 |
| `d` | 右移 | Y方向线性运动 |
| `q` | 左转 | Z轴旋转 |
| `e` | 右转 | Z轴旋转 |
| `x` | 停止 | 所有速度归零 |
| `+` | 增加速度 | 提高最大速度限制 |
| `-` | 减少速度 | 降低最大速度限制 |
| `Ctrl+C` | 退出 | 安全退出程序 |

## 🔍 麦克纳姆轮运动模式

1. **纯前进/后退**: 按 `w` 或 `s`
2. **纯左移/右移**: 按 `a` 或 `d`
3. **纯旋转**: 按 `q` 或 `e`
4. **组合运动**: 同时按多个方向键实现斜向运动
5. **原地旋转**: 在停止状态下按 `q` 或 `e`

## 📊 监控和调试

### 查看话题
```bash
# 查看所有话题
ros2 topic list

# 查看速度命令
ros2 topic echo /cmd_vel

# 查看里程计
ros2 topic echo /odom

# 查看关节状态
ros2 topic echo /joint_states
```

### 查看节点
```bash
# 查看所有节点
ros2 node list

# 查看节点信息
ros2 node info /mecanum_robot_node
ros2 node info /remote_control_node
```

### 查看TF变换
```bash
# 查看TF树
ros2 run tf2_tools view_frames.py

# 查看TF变换
ros2 run tf2_ros tf2_echo odom base_link
```

## 🎨 自定义配置

### 修改机器人参数
在 `scripts/robot_node.py` 中调整麦克纳姆轮参数：
```python
self.wheel_radius = 0.05  # 轮子半径 (m)
self.wheel_base = 0.3     # 轮距 (m) - 前后轮距离
self.wheel_track = 0.3    # 轴距 (m) - 左右轮距离
```

### 修改URDF模型
编辑 `urdf/mecanum_robot.urdf` 文件来修改机器人外观和尺寸。

### 修改RViz配置
编辑 `rviz/mecanum_robot.rviz` 文件来自定义可视化界面。

## 🛠️ 故障排除

### 1. 编译问题
```bash
# 清理并重新编译
rm -rf build install log
colcon build
```

### 2. 权限问题
```bash
# 确保脚本有执行权限
chmod +x scripts/*.py
chmod +x launch/*.py
```

### 3. 依赖问题
确保所有ROS2包都已正确安装。

### 4. 键盘输入问题
- 确保终端支持原始输入模式
- 检查是否有其他程序占用了键盘输入

## 📚 技术细节

### 麦克纳姆轮运动学
系统实现了完整的麦克纳姆轮运动学：
- **逆解**: 从机器人速度计算轮子角速度
- **正解**: 从轮子角速度计算机器人速度
- **积分**: 从速度积分得到位置和姿态

### 坐标系
- **odom**: 里程计坐标系（固定）
- **base_link**: 机器人本体坐标系
- **轮子坐标系**: 四个麦克纳姆轮的坐标系

## 🎯 应用场景

- 麦克纳姆轮机器人开发
- 全向移动平台研究
- 机器人控制算法验证
- 教学演示和培训
- 原型开发和测试

## 📝 许可证

MIT License















