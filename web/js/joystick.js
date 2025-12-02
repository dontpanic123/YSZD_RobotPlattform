/**
 * 虚拟摇杆控制
 * 提供触摸和鼠标控制的虚拟摇杆
 */

class VirtualJoystick {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.knob = this.container.querySelector('.joystick-knob');
        this.isActive = false;
        this.centerX = 0;
        this.centerY = 0;
        this.radius = 0;
        this.currentX = 0;
        this.currentY = 0;
        
        // 防漂移设置
        this.deadzone = 0.05;  // 死区阈值（5%）
        this.lastValues = { linear: 0, lateral: 0, angular: 0 };
        this.zeroCount = 0;   // 连续零值计数
        this.zeroThreshold = 3; // 连续零值阈值
        this.isInDeadzone = false;
        this.wasInDeadzone = false; // 记录上一帧的死区状态
        
        this.init();
    }
    
    init() {
        this.setupDimensions();
        this.setupEventListeners();
        this.setupStyles();
    }
    
    setupDimensions() {
        const rect = this.container.getBoundingClientRect();
        this.centerX = rect.width / 2;
        this.centerY = rect.height / 2;
        this.radius = Math.min(rect.width, rect.height) / 2 - 20; // 留出边距
        
        this.currentX = this.centerX;
        this.currentY = this.centerY;
    }
    
    setupEventListeners() {
        // 鼠标事件
        this.container.addEventListener('mousedown', (e) => this.startControl(e));
        this.container.addEventListener('mousemove', (e) => this.updateControl(e));
        this.container.addEventListener('mouseup', (e) => this.endControl(e));
        this.container.addEventListener('mouseleave', (e) => this.endControl(e));
        
        // 触摸事件
        this.container.addEventListener('touchstart', (e) => {
            e.preventDefault();
            this.startControl(e.touches[0]);
        });
        this.container.addEventListener('touchmove', (e) => {
            e.preventDefault();
            this.updateControl(e.touches[0]);
        });
        this.container.addEventListener('touchend', (e) => {
            e.preventDefault();
            this.endControl(e);
        });
        this.container.addEventListener('touchcancel', (e) => {
            e.preventDefault();
            this.endControl(e);
        });
        
        // 窗口大小变化时重新计算尺寸
        window.addEventListener('resize', () => {
            setTimeout(() => this.setupDimensions(), 100);
        });
    }
    
    setupStyles() {
        // 设置摇杆样式
        this.container.style.position = 'relative';
        this.container.style.cursor = 'pointer';
        this.container.style.userSelect = 'none';
        
        // 设置旋钮样式
        this.knob.style.position = 'absolute';
        this.knob.style.transition = this.isActive ? 'none' : 'all 0.2s ease';
        this.knob.style.transform = 'translate(-50%, -50%)';
        this.knob.style.pointerEvents = 'none';
    }
    
    startControl(event) {
        this.isActive = true;
        this.knob.style.transition = 'none';
        this.container.style.cursor = 'grabbing';
        
        this.updatePosition(event);
        this.updateKnobPosition();
    }
    
    updateControl(event) {
        if (!this.isActive) return;
        
        this.updatePosition(event);
        this.updateKnobPosition();
        this.updateRobotControl();
    }
    
    endControl(event) {
        this.isActive = false;
        this.knob.style.transition = 'all 0.2s ease';
        this.container.style.cursor = 'pointer';
        
        // 重置到中心位置
        this.currentX = this.centerX;
        this.currentY = this.centerY;
        this.updateKnobPosition();
        
        // 立即更新机器人控制为归零状态
        if (window.robotController) {
            // 立即调用updateJoystickControl确保状态更新
            window.robotController.updateJoystickControl(0.0, 0.0, 0.0);
        }
        
        // 强制发送归零命令，防止漂移
        this.forceZeroCommand();
    }
    
    updatePosition(event) {
        const rect = this.container.getBoundingClientRect();
        const clientX = event.clientX || event.pageX;
        const clientY = event.clientY || event.pageY;
        
        this.currentX = clientX - rect.left;
        this.currentY = clientY - rect.top;
    }
    
    updateKnobPosition() {
        // 计算相对于中心的位置
        const deltaX = this.currentX - this.centerX;
        const deltaY = this.currentY - this.centerY;
        const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        // 限制在圆形范围内
        if (distance > this.radius) {
            const angle = Math.atan2(deltaY, deltaX);
            this.currentX = this.centerX + Math.cos(angle) * this.radius;
            this.currentY = this.centerY + Math.sin(angle) * this.radius;
        }
        
        // 更新旋钮位置
        this.knob.style.left = this.currentX + 'px';
        this.knob.style.top = this.currentY + 'px';
    }
    
    updateRobotControl() {
        if (!window.robotController) return;
        
        // 计算摇杆位置（-1 到 1）
        const deltaX = this.currentX - this.centerX;
        const deltaY = this.centerY - this.currentY; // 反转Y轴，向上为正
        
        // 麦克纳姆轮控制：
        // Y轴：前进/后退 (linear.x)
        // X轴：左移/右移 (linear.y) - 修复方向
        let linear = deltaY / this.radius;      // 前进/后退
        let lateral = deltaX / this.radius;     // 左移/右移 (恢复原始方向)
        const angular = 0.0;                     // 旋转暂时设为0，可通过其他方式控制
        
        // 应用死区处理
        const magnitude = Math.sqrt(linear * linear + lateral * lateral);
        if (magnitude < this.deadzone) {
            linear = 0.0;
            lateral = 0.0;
            this.isInDeadzone = true;
        } else {
            this.isInDeadzone = false;
        }
        
        // 限制在 -1 到 1 范围内
        const clampedLinear = Math.max(-1, Math.min(1, linear));
        const clampedLateral = Math.max(-1, Math.min(1, lateral));
        
        // 检查是否有有效输入
        const hasInput = Math.abs(clampedLinear) > 0.001 || Math.abs(clampedLateral) > 0.001;
        
        // 防漂移检测
        const currentValues = { linear: clampedLinear, lateral: clampedLateral, angular: angular };
        const isZero = Math.abs(clampedLinear) < 0.001 && Math.abs(clampedLateral) < 0.001;
        
        if (isZero) {
            this.zeroCount++;
        } else {
            this.zeroCount = 0;
        }
        
        // 如果摇杆不在激活状态（已松开），立即发送归零命令
        if (!this.isActive && isZero) {
            // 摇杆已松开，立即发送归零命令
            window.robotController.updateJoystickControl(0.0, 0.0, 0.0);
            this.lastValues = { linear: 0, lateral: 0, angular: 0 };
            this.zeroCount = 0;
            return;
        }
        
        // 只有在非零值或连续零值达到阈值时才发送命令
        if (!isZero || this.zeroCount >= this.zeroThreshold) {
            // 检查值是否真正改变
            const hasChanged = Math.abs(currentValues.linear - this.lastValues.linear) > 0.001 ||
                             Math.abs(currentValues.lateral - this.lastValues.lateral) > 0.001 ||
                             Math.abs(currentValues.angular - this.lastValues.angular) > 0.001;
            
            if (hasChanged || this.zeroCount === this.zeroThreshold) {
                // 更新机器人控制
                window.robotController.updateJoystickControl(clampedLinear, clampedLateral, angular);
                this.lastValues = { ...currentValues };
                
                if (isZero && this.zeroCount === this.zeroThreshold) {
                    console.log('🛑 摇杆归零，发送防漂移命令');
                }
            }
        }
    }
    
    // 获取当前摇杆值
    getValues() {
        const deltaX = this.currentX - this.centerX;
        const deltaY = this.centerY - this.currentY;
        
        return {
            linear: Math.max(-1, Math.min(1, deltaY / this.radius)),    // 前进/后退
            lateral: Math.max(1, Math.min(1, deltaX / this.radius)),    // 左移/右移 (修复方向)
            angular: 0.0,  // 旋转
            x: deltaX / this.radius,
            y: deltaY / this.radius
        };
    }
    
    // 重置摇杆
    reset() {
        this.currentX = this.centerX;
        this.currentY = this.centerY;
        this.updateKnobPosition();
        this.updateRobotControl();
    }
    
    // 强制发送归零命令
    forceZeroCommand() {
        if (!window.robotController) {
            console.warn('⚠️ 机器人控制器未找到，无法发送归零命令');
            return;
        }
        
        console.log('🛑 摇杆释放，强制发送归零命令');
        
        // 重置状态
        this.zeroCount = 0;
        this.lastValues = { linear: 0, lateral: 0, angular: 0 };
        this.isInDeadzone = true;
        
        // 确保机器人控制器停止所有运动
        window.robotController.targetLinearSpeed = 0.0;
        window.robotController.targetLateralSpeed = 0.0;
        window.robotController.targetAngularSpeed = 0.0;
        window.robotController.linearSpeed = 0.0;
        window.robotController.lateralSpeed = 0.0;
        window.robotController.angularSpeed = 0.0;
        window.robotController.isMoving = false;
        
        // 立即发送归零命令 - 使用更可靠的单次归零命令
        if (typeof window.robotController.sendSingleZeroCommand === 'function') {
            window.robotController.sendSingleZeroCommand();
        } else if (typeof window.robotController.sendZeroCommand === 'function') {
            window.robotController.sendZeroCommand();
        } else {
            // 备用方案：直接发送零值
            window.robotController.updateJoystickControl(0.0, 0.0, 0.0);
        }
        
        // 更新速度显示
        if (typeof window.robotController.updateSpeedDisplay === 'function') {
            window.robotController.updateSpeedDisplay();
        }
        
        // 延迟再次发送归零命令，确保停止（发送多次以确保可靠性）
        setTimeout(() => {
            if (typeof window.robotController.sendSingleZeroCommand === 'function') {
                window.robotController.sendSingleZeroCommand();
                console.log('🛑 延迟发送归零命令，确保停止');
            } else if (typeof window.robotController.sendZeroCommand === 'function') {
                window.robotController.sendZeroCommand();
                console.log('🛑 延迟发送归零命令，确保停止');
            }
        }, 50);
        
        // 再次延迟发送，确保完全停止
        setTimeout(() => {
            if (typeof window.robotController.sendSingleZeroCommand === 'function') {
                window.robotController.sendSingleZeroCommand();
            } else if (typeof window.robotController.sendZeroCommand === 'function') {
                window.robotController.sendZeroCommand();
            }
        }, 150);
    }
    
    
    // 设置摇杆敏感度
    setSensitivity(sensitivity) {
        this.sensitivity = Math.max(0.1, Math.min(2.0, sensitivity));
    }
    
    // 设置死区大小
    setDeadzone(deadzone) {
        this.deadzone = Math.max(0.01, Math.min(0.2, deadzone));
        console.log(`🎯 摇杆死区设置为: ${(this.deadzone * 100).toFixed(1)}%`);
    }
    
    // 获取死区状态
    isInDeadzone() {
        return this.isInDeadzone;
    }
    
    // 获取当前状态信息
    getStatus() {
        return {
            isActive: this.isActive,
            isInDeadzone: this.isInDeadzone,
            zeroCount: this.zeroCount,
            deadzone: this.deadzone,
            values: this.getValues()
        };
    }
}

// 创建虚拟摇杆实例
document.addEventListener('DOMContentLoaded', () => {
    window.virtualJoystick = new VirtualJoystick('joystick');
});
