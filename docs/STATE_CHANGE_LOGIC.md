# 🤖 机器人状态变更逻辑说明

## 📋 状态定义

机器人状态机定义了7种工作状态：

```python
IDLE = "idle"                    # 空闲状态（默认）
MANUAL_CONTROL = "manual_control"  # 手动控制
AUTO_NAVIGATION = "auto_navigation"  # 自动导航
APRILTAG_TRACKING = "apriltag_tracking"  # AprilTag跟踪
EMERGENCY_STOP = "emergency_stop"  # 紧急停止
CHARGING = "charging"            # 充电中
ERROR = "error"                  # 错误状态
```

## 🔄 状态转换规则

### 状态转换表

| 当前状态 | 可转换到的状态 |
|---------|--------------|
| **IDLE** | MANUAL_CONTROL, AUTO_NAVIGATION, APRILTAG_TRACKING, EMERGENCY_STOP, CHARGING |
| **MANUAL_CONTROL** | IDLE, EMERGENCY_STOP, ERROR |
| **AUTO_NAVIGATION** | IDLE, EMERGENCY_STOP, ERROR |
| **APRILTAG_TRACKING** | IDLE, EMERGENCY_STOP, ERROR |
| **EMERGENCY_STOP** | IDLE, ERROR |
| **CHARGING** | IDLE, EMERGENCY_STOP |
| **ERROR** | IDLE, EMERGENCY_STOP |

### 优先级机制

状态转换遵循严格的优先级顺序（从高到低）：

1. **EMERGENCY_STOP** (紧急停止) - 最高优先级
2. **ERROR** (错误状态)
3. **CHARGING** (充电状态)
4. **其他状态** (MANUAL_CONTROL, AUTO_NAVIGATION, APRILTAG_TRACKING)

## 🎯 状态触发条件

### 1. IDLE (空闲状态)

**进入条件：**
- 系统启动时的默认状态
- 手动控制超时（2秒无命令）
- 手动控制保持时间到达（1.5秒）
- 自动导航完成
- AprilTag跟踪超时（60秒）
- 紧急停止解除
- 充电完成

**退出条件：**
- 收到手动控制命令 → MANUAL_CONTROL
- 收到导航目标 → AUTO_NAVIGATION
- 检测到AprilTag → APRILTAG_TRACKING
- 收到紧急停止信号 → EMERGENCY_STOP
- 收到充电信号 → CHARGING

### 2. MANUAL_CONTROL (手动控制)

**进入条件：**
- 在IDLE状态下收到 `/manual_cmd_vel` 话题的非零速度命令

**退出条件：**
- 手动控制保持时间到达（1.5秒）→ IDLE
- 2秒内没有收到速度命令 → IDLE
- 收到紧急停止信号 → EMERGENCY_STOP
- 发生错误 → ERROR

**特殊机制：**
- 保持时间机制：收到第一个运动指令后，必须保持1.5秒才能返回空闲
- 超时机制：如果2秒内没有收到新的速度命令，自动返回空闲

### 3. AUTO_NAVIGATION (自动导航)

**进入条件：**
- 在IDLE状态下收到 `/goal_pose` 话题的导航目标

**退出条件：**
- Waypoint跟踪完成（收到 `completed` 状态）→ IDLE
- Waypoint跟踪停止（收到 `stopped` 状态）→ IDLE
- 收到紧急停止信号 → EMERGENCY_STOP
- 发生错误 → ERROR

### 4. APRILTAG_TRACKING (AprilTag跟踪)

**进入条件：**
- 在IDLE状态下收到 `/apriltag_pose` 话题的位姿消息

**退出条件：**
- AprilTag跟踪超时（60秒内没有收到新的位姿消息）→ IDLE
- 不再收到AprilTag位姿消息 → IDLE
- 收到紧急停止信号 → EMERGENCY_STOP
- 发生错误 → ERROR

**特殊机制：**
- 超时机制：如果60秒内没有收到新的AprilTag位姿，自动返回空闲

### 5. EMERGENCY_STOP (紧急停止)

**进入条件：**
- 收到 `/emergency_stop` 话题的 `true` 消息（任何状态下都可以）

**退出条件：**
- 收到 `/emergency_stop` 话题的 `false` 消息 → IDLE

**特殊机制：**
- 最高优先级：可以中断任何其他状态
- 解除时重置所有活动状态标志

### 6. CHARGING (充电状态)

**进入条件：**
- 收到 `/charging_status` 话题的 `true` 消息

**退出条件：**
- 收到 `/charging_status` 话题的 `false` 消息 → IDLE
- 收到紧急停止信号 → EMERGENCY_STOP

### 7. ERROR (错误状态)

**进入条件：**
- 系统检测到错误条件（`error_condition = True`）

**退出条件：**
- 需要手动调用 `reset_error_state()` 方法 → IDLE
- 收到紧急停止信号 → EMERGENCY_STOP

## 📍 定位状态更新逻辑

### 定位状态定义

定位状态独立于机器人工作状态，用于标识机器人当前所在的位置：

- **"未定位"** - 默认状态，未检测到AprilTag
- **"充电桩"** - 检测到AprilTag ID 0
- **"装载点"** - 检测到AprilTag ID 1
- **"送达点"** - 检测到AprilTag ID 2
- **"未知位置(ID:X)"** - 检测到其他ID的AprilTag

### 定位状态更新机制

1. **从AprilTag位姿消息更新**：
   - 订阅 `/apriltag_pose` 话题
   - 从 `frame_id` 中提取 tag_id（格式：`apriltag_{id}`）
   - 如果无法从 `frame_id` 提取，则从TF变换中获取
   - 根据 tag_id 更新定位状态

2. **从TF变换更新**（备选方案）：
   - 每0.5秒检查一次TF变换
   - 查找 `camera_link` 到 `apriltag_{id}` 的变换
   - 如果找到，更新定位状态

3. **定位超时机制**：
   - 如果10秒内没有检测到AprilTag，自动重置为"未定位"
   - 防止定位状态过时

### 定位状态更新流程

```
收到 /apriltag_pose 消息
    ↓
提取 frame_id (例如: "apriltag_0")
    ↓
从 frame_id 提取 tag_id
    ↓ (如果失败)
从 TF 变换获取 tag_id
    ↓
根据 tag_id 更新定位状态:
    - tag_id = 0 → "充电桩"
    - tag_id = 1 → "装载点"
    - tag_id = 2 → "送达点"
    - 其他 → "未知位置(ID:X)"
    ↓
更新 last_location_time
    ↓
发布到 /robot_location 话题
```

## ⚙️ 状态机更新流程

状态机每0.1秒更新一次（`state_timer`），更新流程如下：

```
state_machine_update() (每0.1秒)
    ↓
determine_next_state()
    ↓
检查优先级:
    1. EMERGENCY_STOP?
    2. ERROR?
    3. CHARGING?
    4. 根据当前状态和输入确定
    ↓
如果新状态 != 当前状态
    ↓
transition_to_state()
    ↓
检查状态转换是否合法
    ↓
执行状态转换
    ↓
execute_state_entry_actions()
    ↓
执行状态进入动作（如停止机器人）
```

## 🔍 状态转换详细逻辑

### determine_next_state() 函数逻辑

```python
def determine_next_state(self):
    # 1. 最高优先级：紧急停止
    if self.emergency_stop_active:
        return RobotState.EMERGENCY_STOP
    
    # 2. 错误状态
    if self.error_condition:
        return RobotState.ERROR
    
    # 3. 充电状态
    if self.charging_active:
        return RobotState.CHARGING
    
    # 4. 根据当前状态处理
    if self.current_state == RobotState.IDLE:
        # 空闲状态：检查是否有活动请求
        if self.manual_control_active:
            return RobotState.MANUAL_CONTROL
        elif self.auto_navigation_active:
            return RobotState.AUTO_NAVIGATION
        elif self.apriltag_tracking_active:
            return RobotState.APRILTAG_TRACKING
    
    elif self.current_state == RobotState.MANUAL_CONTROL:
        # 手动控制：检查保持时间和超时
        if 保持时间到达 or 2秒无命令:
            return RobotState.IDLE
        if not self.manual_control_active:
            return RobotState.IDLE
    
    elif self.current_state == RobotState.AUTO_NAVIGATION:
        # 自动导航：检查是否完成
        if not self.auto_navigation_active:
            return RobotState.IDLE
    
    elif self.current_state == RobotState.APRILTAG_TRACKING:
        # AprilTag跟踪：检查超时
        if 60秒超时 or not self.apriltag_tracking_active:
            return RobotState.IDLE
    
    # 其他状态类似...
    
    return self.current_state  # 保持当前状态
```

## 📊 状态发布机制

状态机每1秒发布一次状态信息：

1. **`/robot_state`** - 发布当前状态名称（字符串）
2. **`/robot_status`** - 发布详细状态信息（字典，包含所有状态标志）
3. **`/robot_location`** - 发布当前定位状态（字符串）

## 🎯 关键参数

- **状态更新频率**：0.1秒（10Hz）
- **状态发布频率**：1秒（1Hz）
- **定位检查频率**：0.5秒（2Hz）
- **AprilTag跟踪超时**：60秒
- **定位超时**：10秒
- **手动控制保持时间**：1.5秒
- **手动控制超时**：2秒

## 🔄 状态转换示例

### 示例1：从空闲到手动控制

```
1. 系统处于 IDLE 状态
2. 用户通过摇杆发送速度命令到 /manual_cmd_vel
3. manual_control_callback() 被调用
4. manual_control_active = True
5. 下次状态更新时，determine_next_state() 返回 MANUAL_CONTROL
6. 执行状态转换：IDLE → MANUAL_CONTROL
7. 记录日志："🎮 机器人进入手动控制模式"
```

### 示例2：从空闲到AprilTag跟踪

```
1. 系统处于 IDLE 状态
2. AprilTag检测器检测到标签，发布 /apriltag_pose
3. apriltag_pose_callback() 被调用
4. apriltag_tracking_active = True
5. 提取 tag_id，更新定位状态（如：未定位 → 充电桩）
6. 下次状态更新时，determine_next_state() 返回 APRILTAG_TRACKING
7. 执行状态转换：IDLE → APRILTAG_TRACKING
8. 记录日志："🏷️ 机器人进入AprilTag跟踪模式"
```

### 示例3：紧急停止中断

```
1. 系统处于任何状态（如 MANUAL_CONTROL）
2. 收到 /emergency_stop 的 true 消息
3. emergency_stop_callback() 被调用
4. emergency_stop_active = True
5. 重置所有活动状态标志
6. 下次状态更新时，determine_next_state() 立即返回 EMERGENCY_STOP
7. 执行状态转换：当前状态 → EMERGENCY_STOP
8. 停止机器人运动
9. 记录日志："🚨 机器人紧急停止!"
```

## 📝 注意事项

1. **状态转换必须合法**：只能按照状态转换表进行转换
2. **优先级不可违反**：紧急停止总是最高优先级
3. **超时机制**：多个状态都有超时保护，防止卡在某个状态
4. **定位状态独立**：定位状态的更新不影响机器人工作状态
5. **状态标志管理**：每个状态都有对应的活动标志，用于判断是否应该进入该状态


