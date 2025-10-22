# 🤖 AprilTag检测机器人系统使用指南

## 📋 系统概述

这是一个完整的AprilTag检测系统，包含：
- **摄像头节点**：调用电脑摄像头获取图像
- **AprilTag检测节点**：识别AprilTag并计算相对位置
- **位置计算节点**：计算机器人到AprilTag的相对位置
- **麦克纳姆轮机器人**：支持全向移动的机器人平台

## 🚀 快速开始

### 0. 设置ROS2域ID (重要!)
```bash
# 确保使用默认域ID (0)
cd /home/bd/Documents/Robot/agv_sim
./set_ros_domain_default.sh
```

### 1. 安装依赖
```bash
cd /home/bd/Documents/Robot/agv_sim
./install_dependencies.sh
```

### 2. 编译项目
```bash
colcon build
source install/setup.bash
```

### 3. 启动系统
```bash
./start_apriltag_robot.sh
```

## 🎯 功能特性

### 摄像头功能
- 自动检测并打开电脑摄像头
- 发布图像话题 `/camera/image_raw`
- 发布摄像头信息 `/camera/camera_info`
- 支持640x480分辨率，30FPS

### AprilTag检测功能
- 支持AprilTag 36h11格式
- 实时检测和识别AprilTag
- 计算AprilTag的3D位置和姿态
- 发布检测结果图像 `/apriltag_detection`
- 发布AprilTag位姿 `/apriltag_pose`

### 位置计算功能
- 计算机器人到AprilTag的相对位置
- 发布相对位姿 `/robot_relative_pose`
- 支持导航控制 `/navigation_cmd`
- 实时状态监控

## 🎮 控制方式

### 键盘控制（原有功能）
- `w/s`: 前进/后退
- `a/d`: 左移/右移
- `q/e`: 左转/右转
- `x`: 停止
- `+/-`: 调节速度

### 自动导航（新增功能）
- 系统会自动检测AprilTag
- 计算机器人到AprilTag的相对位置
- 自动控制机器人朝向AprilTag移动

## 📊 话题接口

### 发布的话题
- `/camera/image_raw`: 摄像头原始图像
- `/camera/camera_info`: 摄像头参数信息
- `/apriltag_detection`: AprilTag检测结果图像
- `/apriltag_pose`: AprilTag位姿信息
- `/robot_relative_pose`: 机器人相对位姿
- `/navigation_cmd`: 导航控制命令
- `/apriltag_status`: AprilTag检测状态
- `/position_status`: 位置计算状态

### 订阅的话题
- `/camera/image_raw`: AprilTag检测节点订阅
- `/apriltag_pose`: 位置计算节点订阅
- `/odom`: 机器人里程计信息

## 🔧 配置参数

### 摄像头参数
- 摄像头ID: 0（默认）
- 分辨率: 640x480
- 帧率: 30 FPS
- 内参矩阵: 自动计算

### AprilTag参数
- 标签格式: AprilTag 36h11
- 标签尺寸: 0.1m x 0.1m
- 检测阈值: 自动调整

### 导航参数
- 最大线速度: 0.5 m/s
- 最大角速度: 1.0 rad/s
- 位置容差: 0.1 m
- 角度容差: 0.1 rad

## 🛠️ 故障排除

### 1. 摄像头问题
```bash
# 检查摄像头是否可用
ls /dev/video*

# 测试摄像头
ros2 run mecanum_robot camera_node.py
```

### 2. AprilTag检测问题
```bash
# 检查话题
ros2 topic list | grep apriltag

# 查看检测状态
ros2 topic echo /apriltag_status
```

### 3. 依赖问题
```bash
# 重新安装依赖
./install_dependencies.sh

# 检查OpenCV
python3 -c "import cv2; print(cv2.__version__)"
```

## 📝 使用步骤

### 步骤1：准备AprilTag
1. 打印AprilTag标签（推荐尺寸：10cm x 10cm）
2. 将标签放置在机器人前方
3. 确保标签在摄像头视野内

### 步骤2：启动系统
```bash
./start_apriltag_robot.sh
```

### 步骤3：观察检测结果
1. 在RViz中查看机器人模型
2. 观察摄像头图像中的AprilTag检测
3. 查看相对位置信息

### 步骤4：控制机器人
- 使用键盘手动控制
- 或让系统自动导航到AprilTag

## 🔍 调试工具

### 查看话题数据
```bash
# 查看AprilTag位姿
ros2 topic echo /apriltag_pose

# 查看相对位置
ros2 topic echo /robot_relative_pose

# 查看导航命令
ros2 topic echo /navigation_cmd
```

### 查看节点状态
```bash
# 查看所有节点
ros2 node list

# 查看节点信息
ros2 node info /apriltag_detector_node
ros2 node info /position_calculator_node
```

## 🎨 自定义配置

### 修改摄像头参数
编辑 `scripts/camera_node.py` 中的摄像头设置：
```python
self.camera_id = 0  # 摄像头ID
self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # 宽度
self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 高度
```

### 修改AprilTag参数
编辑 `scripts/apriltag_detector.py` 中的标签设置：
```python
self.tag_size = 0.1  # 标签尺寸（米）
```

### 修改导航参数
编辑 `scripts/position_calculator.py` 中的控制参数：
```python
self.max_linear_speed = 0.5  # 最大线速度
self.max_angular_speed = 1.0  # 最大角速度
```

## 📚 技术细节

### AprilTag检测流程
1. 摄像头获取图像
2. 转换为灰度图像
3. 使用OpenCV检测AprilTag
4. 计算3D位置和姿态
5. 发布检测结果

### 位置计算流程
1. 获取AprilTag位姿
2. 获取机器人位姿
3. 计算相对位置
4. 生成导航命令
5. 控制机器人移动

## 🎯 应用场景

- 机器人视觉导航
- 目标跟踪和定位
- 自动充电站对接
- 仓库货物识别
- 教学演示和培训

现在你可以使用这个完整的AprilTag检测系统了！

















