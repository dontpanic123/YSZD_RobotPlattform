# 🤖 机器人状态机用户手册

## 概述

本手册介绍如何使用和操作机器人状态机系统。状态机负责管理机器人的不同工作模式，确保安全、有序的机器人操作。

## 📋 目录

1. [系统概述](#系统概述)
2. [状态机架构](#状态机架构)
3. [状态详解](#状态详解)
4. [状态转换规则](#状态转换规则)
5. [用户界面操作](#用户界面操作)
6. [命令行操作](#命令行操作)
7. [故障排除](#故障排除)
8. [API参考](#api参考)

---

## 系统概述

### 功能特性

- **7种工作状态**：空闲、手动控制、自动导航、AprilTag跟踪、紧急停止、充电、错误
- **优先级管理**：紧急停止具有最高优先级
- **自动超时**：手动控制2秒无命令自动返回空闲
- **实时监控**：前端界面实时显示当前状态
- **安全保护**：多重安全检查和错误处理

### 系统组件

- **状态机节点**：`robot_state_machine.py`
- **控制器节点**：`robot_state_controller.py`
- **前端界面**：Web控制台
- **ROS2话题**：状态通信和命令传递

---

## 状态机架构

### 状态图

```
    ┌─────────────┐
    │    IDLE     │◄─────────────────┐
    │   (空闲)    │                  │
    └─────┬───────┘                  │
          │                          │
          ▼                          │
    ┌─────────────┐                  │
    │MANUAL_CONTROL│                 │
    │  (手动控制)  │                  │
    └─────┬───────┘                  │
          │                          │
          ▼                          │
    ┌─────────────┐                  │
    │AUTO_NAVIGATION│                │
    │  (自动导航)  │                  │
    └─────┬───────┘                  │
          │                          │
          ▼                          │
    ┌─────────────┐                  │
    │APRILTAG_TRACKING│              │
    │ (AprilTag跟踪) │                │
    └─────┬───────┘                  │
          │                          │
          ▼                          │
    ┌─────────────┐                  │
    │EMERGENCY_STOP│◄─────────────────┘
    │  (紧急停止)  │
    └─────┬───────┘
          │
          ▼
    ┌─────────────┐
    │   CHARGING   │
    │   (充电)     │
    └─────┬───────┘
          │
          ▼
    ┌─────────────┐
    │    ERROR    │
    │   (错误)    │
    └─────────────┘
```

### 优先级顺序

1. **EMERGENCY_STOP** (最高优先级)
2. **ERROR**
3. **CHARGING**
4. **其他状态**

---

## 状态详解

### 1. IDLE (空闲状态)

**描述**：机器人的默认状态，等待指令。

**特征**：
- ✅ 机器人停止所有运动
- ✅ 等待用户指令
- ✅ 可以接收任何类型的命令

**触发条件**：
- 系统启动
- 其他状态完成或取消
- 手动控制超时（2秒无命令）

**日志信息**：`⏸️ 机器人进入空闲状态`

### 2. MANUAL_CONTROL (手动控制)

**描述**：用户通过摇杆或键盘直接控制机器人。

**特征**：
- ✅ 响应手动控制命令
- ✅ 2秒无命令自动返回空闲
- ✅ 实时响应控制输入

**触发条件**：
- 收到 `/manual_cmd_vel` 话题的非零速度命令
- 前端摇杆操作
- 键盘控制输入

**日志信息**：`🎮 机器人进入手动控制模式`

### 3. AUTO_NAVIGATION (自动导航)

**描述**：机器人自动导航到指定目标位置。

**特征**：
- ✅ 执行路径规划
- ✅ 避障功能
- ✅ 目标到达检测

**触发条件**：
- 收到 `/goal_pose` 话题的导航目标
- 前端设置导航目标

**日志信息**：`🧭 机器人进入自动导航模式`

### 4. APRILTAG_TRACKING (AprilTag跟踪)

**描述**：机器人跟踪检测到的AprilTag。

**特征**：
- ✅ 实时跟踪AprilTag
- ✅ 位置和姿态调整
- ✅ 自动保持距离
- ✅ 60秒超时自动返回空闲状态

**触发条件**：
- 检测到AprilTag位姿
- 收到 `/apriltag_pose` 话题数据

**日志信息**：`🏷️ 机器人进入AprilTag跟踪模式`

### 5. EMERGENCY_STOP (紧急停止)

**描述**：立即停止机器人，最高优先级状态。

**特征**：
- ✅ 立即停止所有运动
- ✅ 中断任何其他状态
- ✅ 需要手动解除

**触发条件**：
- 收到 `/emergency_stop` 话题的 `true` 消息
- 按下紧急停止按钮
- 系统检测到危险情况

**日志信息**：`🚨 机器人紧急停止!`

### 6. CHARGING (充电状态)

**描述**：机器人进入充电模式。

**特征**：
- ✅ 停止所有运动
- ✅ 准备充电接口
- ✅ 监控充电状态

**触发条件**：
- 收到 `/charging_status` 话题的 `true` 消息
- 检测到充电桩
- 电池电量低

**日志信息**：`🔋 机器人进入充电状态`

### 7. ERROR (错误状态)

**描述**：系统检测到错误，需要人工干预。

**特征**：
- ✅ 停止所有运动
- ✅ 记录错误信息
- ✅ 需要手动重置

**触发条件**：
- 系统检测到错误
- 传感器故障
- 通信异常

**日志信息**：`❌ 机器人进入错误状态`

---

## 状态转换规则

### 从 IDLE 状态转换

| 目标状态 | 触发条件 | 优先级 |
|---------|----------|--------|
| MANUAL_CONTROL | 收到手动控制命令 | 正常 |
| AUTO_NAVIGATION | 收到导航目标 | 正常 |
| APRILTAG_TRACKING | 检测到AprilTag | 正常 |
| EMERGENCY_STOP | 紧急停止信号 | 最高 |
| CHARGING | 充电信号 | 高 |

### 从其他状态返回 IDLE

| 当前状态 | 返回条件 | 超时时间 |
|---------|----------|----------|
| MANUAL_CONTROL | 无控制命令 | 2秒 |
| AUTO_NAVIGATION | 导航完成 | 无限制 |
| APRILTAG_TRACKING | 失去AprilTag | 60秒超时 |
| EMERGENCY_STOP | 解除紧急停止 | 手动 |
| CHARGING | 充电完成 | 无限制 |

### 强制状态转换

某些状态可以强制转换到其他状态：

- **EMERGENCY_STOP** → 任何状态（解除紧急停止后）
- **ERROR** → IDLE（手动重置后）

---

## 用户界面操作

### 前端控制台

#### 状态显示
- **位置**：右侧"机器人状态"面板
- **显示**：当前状态名称（如"空闲"、"手动控制"等）
- **更新**：实时更新状态变化

#### 手动控制
1. **选择控制模式**：
   - 点击"摇杆控制"按钮
   - 点击"键盘控制"按钮

2. **摇杆控制**：
   - 拖动虚拟摇杆
   - 观察速度显示
   - 释放摇杆自动停止

3. **键盘控制**：
   - 使用方向键控制
   - 空格键停止

#### 导航控制
1. **设置目标**：
   - 在地图上点击目标位置
   - 或输入坐标值

2. **开始导航**：
   - 点击"开始导航"按钮
   - 观察状态变化

#### 紧急停止
1. **触发紧急停止**：
   - 点击红色"紧急停止"按钮
   - 或发送紧急停止命令

2. **解除紧急停止**：
   - 点击"解除紧急停止"按钮
   - 或发送解除命令

---

## 命令行操作

### ROS2 话题命令

#### 手动控制
```bash
# 前进
ros2 topic pub /manual_cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}"

# 后退
ros2 topic pub /manual_cmd_vel geometry_msgs/msg/Twist "linear: {x: -0.5, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}"

# 左转
ros2 topic pub /manual_cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.5}"

# 停止
ros2 topic pub /manual_cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}"
```

#### 设置导航目标
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  position:
    x: 2.0
    y: 1.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

#### 紧急停止
```bash
# 激活紧急停止
ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: true"

# 解除紧急停止
ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: false"
```

#### 充电控制
```bash
# 开始充电
ros2 topic pub /charging_status std_msgs/msg/Bool "data: true"

# 停止充电
ros2 topic pub /charging_status std_msgs/msg/Bool "data: false"
```

### 状态监控

#### 查看当前状态
```bash
ros2 topic echo /robot_state
```

#### 查看详细状态信息
```bash
ros2 topic echo /robot_status
```

#### 查看状态机日志
```bash
ros2 node info /robot_state_machine
```

#### 配置AprilTag超时时间
```bash
# 设置AprilTag跟踪超时为30秒
ros2 param set /robot_state_machine apriltag_timeout 30.0

# 查看当前参数
ros2 param get /robot_state_machine apriltag_timeout
```

---

## 故障排除

### 常见问题

#### 1. 状态不更新
**症状**：前端显示状态不变
**解决方案**：
- 检查WebSocket连接
- 重启状态机节点
- 检查话题通信

#### 2. 手动控制无响应
**症状**：摇杆操作无效
**解决方案**：
- 检查 `/manual_cmd_vel` 话题
- 确认状态机处于正确状态
- 检查命令格式

#### 3. 紧急停止无法解除
**症状**：紧急停止后无法恢复
**解决方案**：
- 发送解除紧急停止命令
- 重启状态机节点
- 检查错误日志

#### 4. 状态机卡在错误状态
**症状**：显示错误状态无法恢复
**解决方案**：
- 使用重置命令
- 检查错误原因
- 重启系统

### 调试命令

#### 检查话题状态
```bash
ros2 topic list
ros2 topic info /robot_state
ros2 topic info /robot_status
```

#### 检查节点状态
```bash
ros2 node list
ros2 node info /robot_state_machine
```

#### 查看日志
```bash
ros2 log level /robot_state_machine DEBUG
```

---

## API参考

### 话题接口

#### 输入话题

| 话题名称 | 消息类型 | 描述 |
|---------|----------|------|
| `/manual_cmd_vel` | `geometry_msgs/Twist` | 手动控制命令 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 导航目标 |
| `/apriltag_pose` | `geometry_msgs/PoseStamped` | AprilTag位姿 |
| `/emergency_stop` | `std_msgs/Bool` | 紧急停止信号 |
| `/charging_status` | `std_msgs/Bool` | 充电状态 |

#### 输出话题

| 话题名称 | 消息类型 | 描述 |
|---------|----------|------|
| `/robot_state` | `std_msgs/String` | 当前状态 |
| `/robot_status` | `std_msgs/String` | 详细状态信息 |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度命令 |

### 服务接口

#### 重置错误状态
```bash
ros2 service call /reset_error_state std_srvs/srv/Empty
```

#### 强制状态转换
```bash
ros2 service call /force_state_change std_srvs/srv/SetString "{data: 'idle'}"
```

### 参数配置

| 参数名称 | 类型 | 默认值 | 描述 |
|---------|------|--------|------|
| `state_timeout` | double | 30.0 | 状态超时时间（秒） |
| `manual_control_timeout` | double | 2.0 | 手动控制超时时间（秒） |
| `apriltag_timeout` | double | 60.0 | AprilTag跟踪超时时间（秒） |
| `enable_auto_navigation` | bool | true | 启用自动导航 |
| `enable_apriltag_tracking` | bool | true | 启用AprilTag跟踪 |

---

## 最佳实践

### 1. 安全操作
- 始终确保紧急停止按钮可访问
- 在未知环境中先使用手动控制
- 定期检查系统状态

### 2. 状态管理
- 避免频繁状态切换
- 等待当前状态完成再切换
- 监控状态转换日志

### 3. 故障处理
- 遇到错误立即停止操作
- 记录错误日志
- 必要时重启系统

### 4. 性能优化
- 合理设置超时时间
- 避免不必要的状态切换
- 监控系统资源使用

---

## 联系支持

如有问题或需要技术支持，请：

1. 查看系统日志
2. 检查话题通信
3. 重启相关节点
4. 联系技术支持团队

---

**版本**：1.0  
**更新日期**：2024年1月  
**作者**：机器人系统开发团队
