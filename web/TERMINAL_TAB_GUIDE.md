# Terminal Tab 启动指南

## 📋 概述

现在机器人控制系统支持在单个terminal窗口的不同tab中运行所有服务，而不是打开多个独立的terminal窗口。

## 🚀 启动方式

### 方式1: Gnome Terminal Tabs (推荐)
```bash
cd /home/bd/Documents/Robot/agv_sim/web
./start_complete_system.sh
```

**特点:**
- 所有服务在一个terminal窗口的不同tab中运行
- 4个tab: ROS2机器人系统、ROS2 WebSocket Bridge、Waypoint录制器、Web控制台
- 易于管理和监控
- 关闭terminal窗口即可停止所有服务

### 方式2: Tmux Sessions (高级用户)
```bash
cd /home/bd/Documents/Robot/agv_sim/web
./start_complete_system_tmux.sh
```

**特点:**
- 使用tmux会话管理多个服务
- 支持会话分离和重连
- 更稳定的服务管理
- 适合服务器环境

## 📱 Tab/窗口说明

### Gnome Terminal版本:
- **Tab 1: ROS2机器人系统** - 运行机器人核心系统
- **Tab 2: ROS2 WebSocket Bridge** - Web与ROS2的通信桥梁
- **Tab 3: Waypoint录制器** - 录制机器人路径
- **Tab 4: Web控制台** - HTTP服务器

### Tmux版本:
- **窗口1: ROS2机器人系统** - 运行机器人核心系统
- **窗口2: ROS2 WebSocket Bridge** - Web与ROS2的通信桥梁
- **窗口3: Waypoint录制器** - 录制机器人路径
- **窗口4: Web控制台** - HTTP服务器

## 🔧 管理命令

### Gnome Terminal版本:
- 点击不同tab切换服务
- 关闭terminal窗口停止所有服务
- 每个tab可以独立操作

### Tmux版本:
```bash
# 连接到会话
tmux attach -t robot_control

# 分离会话 (服务继续运行)
Ctrl+B, D

# 重新连接会话
tmux attach -t robot_control

# 停止所有服务
tmux kill-session -t robot_control

# 查看会话列表
tmux list-sessions
```

## 🌐 访问地址

- **Web控制台**: http://localhost:8080
- **ROS2 Web Bridge**: ws://localhost:9090

## 💡 使用建议

1. **开发调试**: 使用Gnome Terminal版本，便于查看日志
2. **生产环境**: 使用Tmux版本，更稳定可靠
3. **远程服务器**: 使用Tmux版本，支持SSH断开重连

## 🛑 停止服务

### Gnome Terminal版本:
- 关闭terminal窗口
- 或在每个tab中按Ctrl+C

### Tmux版本:
```bash
tmux kill-session -t robot_control
```

## 🔍 故障排除

### 如果tab没有正确创建:
1. 检查gnome-terminal是否安装
2. 检查脚本权限: `chmod +x start_complete_system.sh`

### 如果tmux会话无法创建:
1. 安装tmux: `sudo apt install tmux`
2. 检查脚本权限: `chmod +x start_complete_system_tmux.sh`

### 如果服务启动失败:
1. 检查ROS2环境: `source /opt/ros/humble/setup.bash`
2. 检查工作目录: `cd /home/bd/Documents/Robot/agv_sim`
3. 查看具体tab/窗口的错误信息

