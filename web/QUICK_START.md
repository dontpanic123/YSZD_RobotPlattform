# 🚀 快速启动指南

## 一键启动（推荐）

```bash
cd /home/bd/Documents/Robot/agv_sim/web
./start_complete_system.sh
```

然后打开浏览器访问：`http://localhost:8080`

## 分步启动

### 1. 启动ROS2机器人系统
```bash
cd /home/bd/Documents/Robot/agv_sim
./start_apriltag_robot.sh
```

### 2. 启动ROS2 Web Bridge
```bash
cd /home/bd/Documents/Robot/agv_sim/web
./start_ros2_bridge.sh
```

### 3. 启动Web服务器
```bash
cd /home/bd/Documents/Robot/agv_sim/web
./start_web_server.sh
```

## 测试系统

访问测试页面：`http://localhost:8080/test.html`

## 功能说明

### 🎮 控制方式
- **键盘**: WASD键控制移动
- **摇杆**: 鼠标拖拽虚拟摇杆
- **按钮**: 点击控制按钮

### 📹 监控功能
- **摄像头**: 实时视频流
- **AprilTag**: 检测结果显示
- **状态**: 机器人位置和速度

### 🧭 导航功能
- **目标设置**: 输入坐标设置目标
- **自动导航**: 机器人自动到达目标
- **取消导航**: 一键停止导航

## 故障排除

### 连接问题
1. 检查ROS2系统是否运行
2. 确认Web Bridge端口9090可用
3. 检查防火墙设置

### 控制问题
1. 确认`/cmd_vel`话题正常
2. 检查机器人节点状态
3. 查看浏览器控制台错误

### 视频问题
1. 确认摄像头节点运行
2. 检查`/camera/image_raw`话题
3. 验证图像格式支持

## 系统要求

- **操作系统**: Ubuntu 20.04+
- **ROS2**: Humble
- **Node.js**: 14.0+
- **浏览器**: Chrome/Firefox/Safari
- **Python**: 3.6+

## 端口说明

- **Web服务器**: 8080
- **ROS2 Web Bridge**: 9090
- **ROS2系统**: 默认端口

## 安全提示

- 仅在受信任网络中使用
- 生产环境请使用HTTPS
- 设置适当的访问控制












