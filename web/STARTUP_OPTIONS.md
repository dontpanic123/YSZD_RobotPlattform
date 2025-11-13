# 机器人控制系统启动选项

## 🚀 启动脚本说明

现在您有多个启动选项，可以根据您的需求选择：

### 1. 完整多Tab版本 (推荐)
```bash
./start_complete_system.sh
```
- **功能**: 在一个gnome-terminal窗口中创建4个tab
- **优点**: 所有服务在一个窗口中，便于管理
- **要求**: 需要gnome-terminal支持

### 2. 简化独立窗口版本
```bash
./start_simple_tabs.sh
```
- **功能**: 创建4个独立的gnome-terminal窗口
- **优点**: 更可靠，每个服务独立运行
- **适用**: 当多tab版本不工作时使用

### 3. Tmux版本 (最稳定)
```bash
./start_robot_tmux.sh
```
- **功能**: 使用tmux创建4个窗口
- **优点**: 最稳定，支持远程连接
- **适用**: 服务器环境或gnome-terminal不可用时

## 📊 服务说明

所有启动脚本都会启动以下4个服务：

| 服务 | 功能 | 端口/地址 |
|------|------|-----------|
| ROS2机器人系统 | 机器人控制和AprilTag检测 | ROS2 topics |
| ROS2 WebSocket Bridge | Web与ROS2通信桥梁 | ws://localhost:9090 |
| Waypoint录制器 | 路径录制和回放 | ROS2 services |
| Web控制台 | 用户界面 | http://localhost:8080 |

## 🔧 故障排除

### 如果gnome-terminal多tab不工作：
1. 尝试简化版本: `./start_simple_tabs.sh`
2. 使用tmux版本: `./start_robot_tmux.sh`

### 如果看不到terminal窗口：
1. 检查DISPLAY环境变量: `echo $DISPLAY`
2. 确保在图形界面环境中运行
3. 尝试手动启动: `gnome-terminal --version`

### 如果ROS2服务启动失败：
1. 检查ROS2环境: `source /opt/ros/humble/setup.bash`
2. 检查工作目录: `cd /home/bd/Documents/Robot/agv_sim`
3. 检查依赖: `source install/setup.bash`

## 💡 使用建议

1. **首次使用**: 建议使用 `./start_complete_system.sh`
2. **服务器环境**: 使用 `./start_robot_tmux.sh`
3. **调试时**: 使用 `./start_simple_tabs.sh` 便于查看每个服务的日志

## 🛑 停止服务

- **Gnome-terminal版本**: 关闭terminal窗口
- **Tmux版本**: 运行 `tmux kill-session -t robot_control`
- **手动停止**: 使用 `Ctrl+C` 或 `pkill` 命令

## 📱 访问界面

启动成功后，打开浏览器访问：
- **主界面**: http://localhost:8080
- **WebSocket**: ws://localhost:9090

## 🔄 重启服务

如果需要重启某个服务：
1. 找到对应的terminal窗口
2. 按 `Ctrl+C` 停止服务
3. 重新运行对应的命令
