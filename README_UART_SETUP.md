# Jetson UART权限设置指南

## 快速设置（推荐）

运行自动设置脚本：

```bash
./setup_uart_permissions.sh
```

然后**重新登录**或运行：
```bash
newgrp dialout
```

## 手动设置

### 1. 添加用户到dialout组

```bash
sudo usermod -a -G dialout $USER
```

### 2. 安装udev规则（永久解决方案）

```bash
# 复制udev规则
sudo cp 99-jetson-uart.rules /etc/udev/rules.d/

# 重新加载udev规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 设置当前设备权限
sudo chmod 664 /dev/ttyTHS1
sudo chgrp dialout /dev/ttyTHS1
```

### 3. 激活组权限

**重要**: 添加用户到组后，需要重新登录或运行：

```bash
newgrp dialout
```

或者注销并重新登录系统。

### 4. 验证权限

```bash
# 检查用户组
groups  # 应该包含 'dialout'

# 检查设备权限
ls -l /dev/ttyTHS1
# 应该显示: crw-rw---- 1 root dialout ... /dev/ttyTHS1
```

## 临时解决方案（每次启动时）

如果不想设置永久权限，启动脚本会自动尝试修复权限，但可能需要输入sudo密码。

## 故障排除

### 权限仍然被拒绝

1. 确认用户在dialout组中：
   ```bash
   groups | grep dialout
   ```

2. 如果不在组中，运行：
   ```bash
   sudo usermod -a -G dialout $USER
   newgrp dialout  # 或重新登录
   ```

3. 检查udev规则是否生效：
   ```bash
   sudo udevadm info /dev/ttyTHS1 | grep MODE
   ```

4. 手动设置权限（临时）：
   ```bash
   sudo chmod 664 /dev/ttyTHS1
   sudo chgrp dialout /dev/ttyTHS1
   ```

### 设备不存在

检查UART设备是否存在：
```bash
ls -l /dev/ttyTHS*
```

如果不存在，可能需要启用UART1在Jetson配置中。




