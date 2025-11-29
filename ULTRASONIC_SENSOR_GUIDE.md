# 超声波传感器节点使用指南

## 概述

`ultrasonic_sensor_node.py` 是一个ROS2节点，用于读取超声波传感器模块的数据。该节点通过串口（GPIO/UART）与传感器模块通信，解析协议消息并发布传感器数据。

## 协议说明

### 命令格式
- 消息以 `$` 开头
- 以 `\r\n` 结尾（回车换行）
- 校验和格式：`*XX`（调试时固定为XX）
- 字段之间用分号 `;` 分隔

### 支持的命令

#### 1. 设置查询频率
```
$UDM1;[频率]*XX
```
- 方向：主机 -> 模块
- 功能：设置查询频率
- 参数：
  - `0`: 查询反馈模式（不自动反馈，需主动查询）
  - `1-5`: 自动反馈频率（1Hz-5Hz）

示例：
- `$UDM1;0*XX` - 查询反馈模式
- `$UDM1;5*XX` - 自动反馈，每秒5次

#### 2. 查询传感器信息
```
$UDM2;CHK*XX
```
- 方向：主机 -> 模块
- 功能：查询传感器信息（仅在查询反馈模式下有效）

#### 3. 传感器反馈消息
```
$udm1;[n];[传感器信息1];[传感器信息2];[传感器信息n]*XX
```
- 方向：模块 -> 主机
- 功能：反馈传感器信息
- 参数：
  - `[n]`: 传感器数量（1个字符）
  - `[传感器信息]`: 5个字符
    - 第1位：传感器索引（0-7）
    - 后4位：距离值（毫米）

示例：
- `$udm1;3;11900;32000;40670*XX`
  - 3个传感器
  - 传感器1：1900mm (1.9m)
  - 传感器3：2000mm (2.0m)
  - 传感器4：670mm (0.67m)

## 节点参数

节点支持以下ROS2参数：

- `serial_port` (string, 默认: `/dev/ttyTHS1`)
  - 串口设备路径
  - Jetson Orin Nano默认: `/dev/ttyTHS1` (UART1)
  - 可设置为 `auto` 自动检测串口（优先Jetson UART设备）

- `baudrate` (int, 默认: `115200`)
  - 串口波特率
  - Jetson Orin Nano默认: `115200`

- `query_frequency` (int, 默认: `5`)
  - 查询频率（0-5）
  - 0: 查询反馈模式
  - 1-5: 自动反馈频率（Hz）

- `auto_feedback` (bool, 默认: `true`)
  - 是否启用自动反馈模式
  - `true`: 自动反馈（使用query_frequency设置频率）
  - `false`: 查询反馈模式（需要主动查询）

## 发布的话题

### 单个传感器话题
- `/ultrasonic/sensor_0` 到 `/ultrasonic/sensor_7` (sensor_msgs/Range)
  - 每个传感器独立的话题
  - 消息类型：`sensor_msgs/Range`
  - 距离单位：米
  - 范围：0.02m - 4.0m

### 所有传感器数据
- `/ultrasonic/all_sensors` (std_msgs/Float32MultiArray)
  - 包含所有8个传感器的数据
  - 数组长度为8
  - 无效传感器值为 -1.0

## Jetson Orin Nano 配置

### GPIO引脚映射

Jetson Orin Nano UART1 GPIO映射：
- **GPIO 6**: GND (地)
- **GPIO 8**: UART1_TXD (发送)
- **GPIO 10**: UART1_RXD (接收)

### UART设备路径

Jetson Orin Nano的UART1设备路径为：`/dev/ttyTHS1`

### 权限设置

Jetson UART设备需要适当的权限。运行以下命令：

```bash
sudo usermod -a -G dialout $USER
```

然后**重新登录**或运行：

```bash
newgrp dialout
```

验证权限：

```bash
ls -l /dev/ttyTHS1
# 应该显示: crw-rw---- 1 root dialout ... /dev/ttyTHS1
```

### 启用UART1（如果需要）

确保UART1已启用。检查设备树配置或运行：

```bash
# 检查UART设备
ls -l /dev/ttyTHS*

# 检查UART是否可用
dmesg | grep ttyTHS
```

## 使用方法

### 1. 基本启动（使用默认参数 - Jetson配置）

```bash
ros2 run mecanum_robot ultrasonic_sensor_node.py
```

默认配置：
- 串口: `/dev/ttyTHS1` (Jetson UART1)
- 波特率: `115200`

### 2. 指定串口

```bash
# 使用其他UART
ros2 run mecanum_robot ultrasonic_sensor_node.py --ros-args -p serial_port:=/dev/ttyTHS0

# 使用USB串口
ros2 run mecanum_robot ultrasonic_sensor_node.py --ros-args -p serial_port:=/dev/ttyUSB0
```

### 3. 自动检测串口

```bash
ros2 run mecanum_robot ultrasonic_sensor_node.py --ros-args -p serial_port:=auto
```

自动检测会优先查找Jetson UART设备 (`/dev/ttyTHS*`)，然后查找USB串口。

### 4. 配置查询频率和模式

```bash
# 自动反馈模式，5Hz
ros2 run mecanum_robot ultrasonic_sensor_node.py --ros-args \
  -p auto_feedback:=true \
  -p query_frequency:=5

# 查询反馈模式
ros2 run mecanum_robot ultrasonic_sensor_node.py --ros-args \
  -p auto_feedback:=false \
  -p query_frequency:=0
```

### 5. 在Launch文件中使用

```python
# Jetson Orin Nano配置
Node(
    package='mecanum_robot',
    executable='ultrasonic_sensor_node.py',
    name='ultrasonic_sensor_node',
    parameters=[{
        'serial_port': '/dev/ttyTHS1',  # Jetson UART1
        'baudrate': 115200,              # Jetson UART1 baudrate
        'query_frequency': 5,
        'auto_feedback': True
    }]
)
```

## 查看传感器数据

### 查看单个传感器

```bash
# 查看传感器0的数据
ros2 topic echo /ultrasonic/sensor_0

# 查看传感器1的数据
ros2 topic echo /ultrasonic/sensor_1
```

### 查看所有传感器数据

```bash
ros2 topic echo /ultrasonic/all_sensors
```

### 查看话题列表

```bash
ros2 topic list | grep ultrasonic
```

### 查看话题信息

```bash
ros2 topic info /ultrasonic/all_sensors
ros2 topic hz /ultrasonic/sensor_0
```

## 消息格式

### sensor_msgs/Range

```yaml
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: "ultrasonic_sensor_0"
radiation_type: 0  # ULTRASOUND
field_of_view: 0.1  # 约5.7度
min_range: 0.02     # 2cm
max_range: 4.0      # 4m
range: 1.9          # 距离（米）
```

### std_msgs/Float32MultiArray

```yaml
layout:
  dim:
    - label: "sensors"
      size: 8
      stride: 8
data: [1.9, -1.0, 2.0, 0.67, -1.0, -1.0, -1.0, -1.0]
# 数组索引对应传感器索引
# -1.0 表示该传感器无数据
```

## 故障排除

### 1. 串口权限问题

如果遇到权限错误，将用户添加到dialout组：

```bash
sudo usermod -a -G dialout $USER
# 需要重新登录生效，或运行: newgrp dialout
```

**Jetson特定**: 对于Jetson UART设备 (`/dev/ttyTHS*`)，确保用户有dialout组权限。

验证权限：

```bash
groups  # 应该包含 dialout
ls -l /dev/ttyTHS1  # 检查设备权限
```

或者使用sudo运行（不推荐，仅用于测试）：

```bash
sudo ros2 run mecanum_robot ultrasonic_sensor_node.py
```

### 2. 串口未找到

**Jetson设备**:

检查Jetson UART设备：

```bash
ls -l /dev/ttyTHS*
```

检查UART是否启用：

```bash
dmesg | grep -i uart
cat /proc/device-tree/serial* 2>/dev/null
```

**USB串口设备**:

检查可用串口：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

**自动检测**:

使用自动检测（优先Jetson UART）：

```bash
ros2 run mecanum_robot ultrasonic_sensor_node.py --ros-args -p serial_port:=auto
```

### 3. 无数据接收

- 检查串口连接
- **Jetson**: 确认GPIO连接正确（GPIO 6=GND, GPIO 8=TXD, GPIO 10=RXD）
- 检查波特率设置是否正确（Jetson默认115200）
- 检查传感器模块是否正常工作
- 查看节点日志：`ros2 topic echo /rosout | grep ultrasonic`
- **Jetson**: 检查UART1是否被其他进程占用：`lsof /dev/ttyTHS1`

### 4. Jetson UART特定问题

**UART设备不存在**:

如果 `/dev/ttyTHS1` 不存在，可能需要启用UART1。检查设备树配置或Jetson配置工具。

**UART被占用**:

检查是否有其他进程使用UART：

```bash
sudo lsof /dev/ttyTHS1
sudo fuser /dev/ttyTHS1
```

**波特率不匹配**:

确保传感器模块和节点使用相同的波特率（默认115200）。

### 5. 数据解析错误

查看节点日志输出，检查消息格式是否正确。

### 6. Jetson GPIO配置验证

验证GPIO连接：

```bash
# 检查GPIO状态（如果安装了gpio工具）
gpio readall  # 或使用Jetson GPIO工具

# 检查UART是否工作
dmesg | grep ttyTHS1
```

确保硬件连接：
- GPIO 6 → GND
- GPIO 8 → UART1_TXD (连接到传感器模块的RX)
- GPIO 10 → UART1_RXD (连接到传感器模块的TX)

## 依赖

- `pyserial`: Python串口通信库
  ```bash
  pip3 install pyserial
  ```

- ROS2包：
  - `rclpy`
  - `sensor_msgs`
  - `std_msgs`

## 传感器索引说明

传感器索引（0-7）与模块的传感器端口一一对应：
- 传感器0 → 端口0
- 传感器1 → 端口1
- ...
- 传感器7 → 端口7

## 注意事项

1. 确保串口设备有正确的权限
2. 波特率必须与传感器模块匹配（通常为9600）
3. 在查询反馈模式下，节点会每秒查询一次
4. 在自动反馈模式下，传感器模块会自动发送数据
5. 无效的传感器读数会显示为 -1.0（在all_sensors话题中）

