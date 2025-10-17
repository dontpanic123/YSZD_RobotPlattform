# 机器小车Web控制台

基于ros2-web-bridge开发的机器小车前端可视化界面，提供完整的机器人遥控和监控功能。

## 🚀 功能特性

### 🎮 遥控控制
- **虚拟摇杆**: 支持鼠标和触摸控制的虚拟摇杆
- **键盘控制**: WASD键控制机器人移动
- **按钮控制**: 直观的按钮界面控制
- **速度调节**: 可调节最大线速度和角速度

### 📹 视频监控
- **实时摄像头**: 显示机器人摄像头画面
- **AprilTag检测**: 实时显示AprilTag检测结果
- **图像处理**: 支持图像增强和检测可视化

### 📊 状态监控
- **机器人状态**: 位置、方向、速度信息
- **AprilTag信息**: 检测到的标签位置和距离
- **连接状态**: ROS2连接状态和质量监控
- **系统信息**: 实时系统状态显示

### 🧭 导航控制
- **目标设置**: 设置机器人导航目标点
- **路径规划**: 支持自动路径规划
- **导航取消**: 一键取消当前导航任务

## 📁 文件结构

```
web/
├── index.html              # 主页面
├── css/
│   └── style.css          # 样式文件
├── js/
│   ├── ros2-bridge.js     # ROS2 Web Bridge连接
│   ├── robot-control.js    # 机器人控制逻辑
│   ├── joystick.js         # 虚拟摇杆控制
│   └── main.js            # 主应用程序
├── images/                # 图片资源
├── start_web_server.sh    # 启动Web服务器
├── start_ros2_bridge.sh   # 启动ROS2 Web Bridge
├── start_complete_system.sh # 启动完整系统
└── README.md              # 说明文档
```

## 🛠️ 安装和配置

### 1. 安装依赖

#### 安装Python WebSocket依赖
```bash
# 安装websockets库
pip3 install websockets
```

#### 安装Python HTTP服务器
```bash
# Python3通常已预装，如果没有：
sudo apt-get install python3
```

### 2. 配置ROS2环境
确保ROS2环境已正确设置：
```bash
source /opt/ros/humble/setup.bash
source /home/bd/Documents/Robot/agv_sim/install/setup.bash
```

## 🚀 使用方法

### 方法1: 启动完整系统（推荐）
```bash
cd /home/bd/Documents/Robot/agv_sim/web
./start_complete_system.sh
```

这将启动：
- ROS2机器人系统
- ROS2 Web Bridge
- Web控制台

### 方法2: 分步启动

#### 1. 启动ROS2机器人系统
```bash
cd /home/bd/Documents/Robot/agv_sim
./start_apriltag_robot.sh
```

#### 2. 启动ROS2 Web Bridge
```bash
cd /home/bd/Documents/Robot/agv_sim/web
./start_ros2_bridge.sh
```

#### 3. 启动Web服务器
```bash
cd /home/bd/Documents/Robot/agv_sim/web
./start_web_server.sh
```

### 访问Web控制台
打开浏览器访问：`http://localhost:8080`

## 🎮 控制说明

### 键盘控制
- **W**: 前进
- **A**: 左转
- **S**: 停止
- **D**: 右转
- **X**: 后退

### 摇杆控制
- 鼠标拖拽或触摸摇杆控制机器人移动
- 摇杆中心为停止位置
- 摇杆边缘为最大速度

### 速度设置
- 使用滑块调节最大线速度和角速度
- 线速度范围: 0.1 - 2.0 m/s
- 角速度范围: 0.1 - 3.0 rad/s

### 导航控制
1. 在目标位置输入框中输入坐标
2. 点击"设置目标点"按钮
3. 机器人将自动导航到目标位置
4. 点击"取消导航"停止导航

## 📡 支持的话题

### 输入话题
- `/cmd_vel` - 机器人速度控制
- `/goal_pose` - 导航目标点
- `/initialpose` - 初始位置设置

### 输出话题
- `/camera/image_raw` - 摄像头图像
- `/apriltag_detection` - AprilTag检测图像
- `/apriltag_pose` - AprilTag位姿信息
- `/apriltag_status` - AprilTag检测状态
- `/odom` - 机器人里程计数据

## 🔧 故障排除

### 连接问题
1. **Web控制台无法连接ROS2**
   - 检查ROS2 Web Bridge是否运行
   - 确认端口9090未被占用
   - 检查防火墙设置

2. **摄像头画面不显示**
   - 确认摄像头节点正在运行
   - 检查话题`/camera/image_raw`是否有数据
   - 查看浏览器控制台错误信息

3. **AprilTag检测不工作**
   - 确认AprilTag检测节点正在运行
   - 检查摄像头视野中是否有AprilTag标记
   - 查看检测参数设置

### 性能优化
1. **降低图像质量**：修改摄像头分辨率
2. **减少更新频率**：调整控制循环频率
3. **关闭不需要的功能**：禁用不需要的话题订阅

## 🌐 网络配置

### 本地访问
- 默认地址: `http://localhost:8080`
- 本地网络访问: `http://[你的IP]:8080`

### 远程访问
1. 修改ROS2 Web Bridge配置
2. 设置防火墙规则
3. 使用HTTPS确保安全连接

## 📱 移动设备支持

Web控制台支持移动设备访问：
- 响应式设计，适配各种屏幕尺寸
- 触摸控制支持
- 虚拟摇杆优化移动设备操作

## 🔒 安全注意事项

1. **网络安全**: 在生产环境中使用HTTPS
2. **访问控制**: 设置适当的访问权限
3. **数据加密**: 敏感数据传输加密
4. **定期更新**: 保持依赖包最新版本

## 📈 扩展功能

### 自定义功能
- 添加新的控制模式
- 集成更多传感器数据
- 实现自定义UI组件

### 性能监控
- 添加性能指标显示
- 实现数据记录功能
- 集成日志系统

## 🆘 技术支持

如果遇到问题，请检查：
1. 浏览器控制台错误信息
2. ROS2系统日志
3. 网络连接状态
4. 依赖包版本兼容性

## 📄 许可证

本项目基于MIT许可证开源。
