# 麦克纳姆轮控制界面更新

## 🎯 更新目标
将原有的差速轮控制界面升级为麦克纳姆轮全向移动控制界面，支持：
- 前进/后退 (linear.x)
- 左移/右移 (linear.y) 
- 旋转 (angular.z)

## 🔧 主要修改

### 1. HTML界面更新 (`web/index.html`)
- **控制按钮重新布局**：
  - 前进/后退：W/X键
  - 左移/右移：Q/E键  
  - 左转/右转：A/D键
  - 停止：S键

- **速度显示更新**：
  - 前进/后退速度显示
  - 左移/右移速度显示
  - 旋转速度显示

- **速度设置滑块**：
  - 最大前进/后退速度
  - 最大侧向移动速度
  - 最大旋转速度

- **旋转控制滑块**：
  - 独立的旋转控制滑块
  - 实时显示旋转值

### 2. JavaScript控制逻辑更新 (`web/js/robot-control.js`)

#### 新增变量：
```javascript
this.lateralSpeed = 0.0;     // 左移/右移速度 (linear.y)
this.maxLateralSpeed = 1.0; // 最大侧向移动速度
```

#### 控制逻辑更新（与remote_control.py保持一致）：
```javascript
// 麦克纳姆轮控制逻辑 - 与remote_control.py保持一致
// 前进/后退控制 (w/s)
if (this.currentKeys.has('w') || this.currentKeys.has('forward')) {
    linear = this.maxLinearSpeed;
}
if (this.currentKeys.has('s') || this.currentKeys.has('backward')) {
    linear = -this.maxLinearSpeed;
}

// 侧向移动控制 (a/d)
if (this.currentKeys.has('a') || this.currentKeys.has('left_shift')) {
    lateral = this.maxLateralSpeed;
}
if (this.currentKeys.has('d') || this.currentKeys.has('right_shift')) {
    lateral = -this.maxLateralSpeed;
}

// 旋转控制 (q/e)
if (this.currentKeys.has('q') || this.currentKeys.has('rotate_left')) {
    angular = this.maxAngularSpeed;
}
if (this.currentKeys.has('e') || this.currentKeys.has('rotate_right')) {
    angular = -this.maxAngularSpeed;
}
```

#### Twist消息格式：
```javascript
const twistMessage = {
    linear: {
        x: this.linearSpeed,    // 前进/后退
        y: this.lateralSpeed,   // 左移/右移
        z: 0.0
    },
    angular: {
        x: 0.0,
        y: 0.0,
        z: this.angularSpeed    // 旋转
    }
};
```

### 3. 摇杆控制更新 (`web/js/joystick.js`)

#### 摇杆映射：
- **Y轴**：前进/后退 (linear.x)
- **X轴**：左移/右移 (linear.y)
- **旋转**：通过独立滑块控制

```javascript
// 麦克纳姆轮控制：
// Y轴：前进/后退 (linear.x)
// X轴：左移/右移 (linear.y)
const linear = deltaY / this.radius;      // 前进/后退
const lateral = deltaX / this.radius;    // 左移/右移
const angular = 0.0;                     // 旋转通过滑块控制
```

### 4. CSS样式更新 (`web/css/style.css`)

#### 新增旋转控制样式：
```css
.rotation-control {
    margin: 0.5rem 0;
    padding: 0.5rem;
    background: rgba(231, 76, 60, 0.1);
    border-radius: 8px;
    border-left: 3px solid #e74c3c;
}

.rotation-control input[type="range"] {
    width: 100%;
    height: 6px;
    border-radius: 3px;
    background: #ddd;
    outline: none;
    -webkit-appearance: none;
}
```

## 🧪 测试页面

创建了专门的测试页面 `web/test_mecanum.html`：
- 实时显示速度命令
- 可视化Twist消息格式
- 完整的控制说明
- 独立测试环境

## 🎮 控制方式

### 键盘控制（与remote_control.py保持一致）：
- **W**: 前进
- **S**: 后退  
- **A**: 左移
- **D**: 右移
- **Q**: 左转
- **E**: 右转
- **X**: 停止

### 摇杆控制：
- **Y轴**: 前进/后退
- **X轴**: 左移/右移
- **旋转滑块**: 旋转控制

### 速度设置：
- 最大前进/后退速度：0.1-2.0 m/s
- 最大侧向移动速度：0.1-2.0 m/s  
- 最大旋转速度：0.1-3.0 rad/s

## 🔗 与后端集成

### ROS2 Twist消息格式：
```yaml
linear:
  x: 0.0    # 前进/后退速度 (m/s)
  y: 0.0    # 左移/右移速度 (m/s)  
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0    # 旋转速度 (rad/s)
```

### 麦克纳姆轮运动学：
后端机器人节点 (`scripts/robot_node.py`) 已经实现了麦克纳姆轮运动学逆解：
```python
# 麦克纳姆轮运动学逆解
w1 = (vx - vy - (L + W) * wz) / R  # front_left
w2 = (vx + vy + (L + W) * wz) / R  # front_right  
w3 = (vx + vy - (L + W) * wz) / R  # rear_left
w4 = (vx - vy + (L + W) * wz) / R  # rear_right
```

## ✅ 验证方法

1. **启动测试页面**：
   ```bash
   cd /home/bd/Documents/Robot/agv_sim/web
   python3 -m http.server 8080
   # 访问 http://localhost:8080/test_mecanum.html
   ```

2. **测试控制功能**：
   - 使用键盘WASD+QE测试各方向移动
   - 使用摇杆测试全向移动
   - 使用旋转滑块测试旋转控制
   - 观察实时速度显示

3. **验证ROS2集成**：
   - 启动机器人系统
   - 启动WebSocket桥接器
   - 在Web界面控制机器人移动

## 🎉 完成状态

✅ HTML界面更新完成  
✅ JavaScript控制逻辑更新完成  
✅ 摇杆控制更新完成  
✅ CSS样式更新完成  
✅ 测试页面创建完成  
✅ 控制说明更新完成  

现在前端界面完全支持麦克纳姆轮的全向移动控制！🎮🤖
