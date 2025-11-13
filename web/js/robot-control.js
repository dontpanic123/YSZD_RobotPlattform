/**
 * 机器人控制模块
 * 处理机器人的移动控制和导航
 */

class RobotController {
    constructor() {
        this.linearSpeed = 0.0;      // 前进/后退速度 (linear.x)
        this.lateralSpeed = 0.0;     // 左移/右移速度 (linear.y)
        this.angularSpeed = 0.0;      // 旋转速度 (angular.z)
        this.maxLinearSpeed = 1.0;   // 最大前进/后退速度
        this.maxLateralSpeed = 1.0;  // 最大侧向移动速度
        this.maxAngularSpeed = 1.5;   // 最大旋转速度
        this.isMoving = false;
        this.currentKeys = new Set();
        this.waypointFollowing = false;  // 是否正在执行waypoint跟踪
        this.manualControlEnabled = true;  // 手动控制是否启用
        this.controlLocked = false;  // 控制锁定状态
        this.emergencyStopActive = false;  // 紧急停止状态
        
        this.init();
    }
    
    init() {
        this.setupEventListeners();
        this.setupSpeedControls();
        this.startControlLoop();
    }
    
    setupEventListeners() {
        // 控制模式切换
        this.setupControlModeSwitching();
        
        // 键盘事件
        document.addEventListener('keydown', (e) => {
            try {
                this.handleKeyDown(e);
            } catch (error) {
                console.error('🚨 键盘按下事件处理错误:', error, e);
            }
        });
        document.addEventListener('keyup', (e) => {
            try {
                this.handleKeyUp(e);
            } catch (error) {
                console.error('🚨 键盘释放事件处理错误:', error, e);
            }
        });
        
        // 紧急停止按钮
        this.setupEmergencyStop();
        
        // 按钮事件
        document.querySelectorAll('.control-btn').forEach(btn => {
            btn.addEventListener('mousedown', (e) => this.handleButtonDown(e));
            btn.addEventListener('mouseup', (e) => this.handleButtonUp(e));
            btn.addEventListener('mouseleave', (e) => this.handleButtonUp(e));
        });
        
        // 旋转箭头按钮事件
        document.querySelectorAll('.rotation-btn').forEach(btn => {
            btn.addEventListener('mousedown', (e) => this.handleRotationButtonDown(e));
            btn.addEventListener('mouseup', (e) => this.handleRotationButtonUp(e));
            btn.addEventListener('mouseleave', (e) => this.handleRotationButtonUp(e));
        });
        
        // 导航控制
        document.getElementById('setGoalBtn')?.addEventListener('click', () => this.setGoal());
        document.getElementById('cancelGoalBtn')?.addEventListener('click', () => this.cancelGoal());
        
        // 旋转控制滑块
        const rotationSlider = document.getElementById('rotationSlider');
        const rotationValue = document.getElementById('rotationValue');
        if (rotationSlider && rotationValue) {
            rotationSlider.addEventListener('input', (e) => {
                const rotation = parseFloat(e.target.value);
                rotationValue.textContent = rotation.toFixed(1);
                this.angularSpeed = rotation * this.maxAngularSpeed;
                this.updateSpeedDisplay();
                this.sendControlCommand();
            });
        }
    }
    
    setupControlModeSwitching() {
        // 控制模式切换按钮
        const modeButtons = document.querySelectorAll('.mode-btn');
        modeButtons.forEach(btn => {
            btn.addEventListener('click', (e) => {
                const mode = e.target.dataset.mode;
                this.switchControlMode(mode);
            });
        });
    }
    
    switchControlMode(mode) {
        // 更新按钮状态
        document.querySelectorAll('.mode-btn').forEach(btn => {
            btn.classList.remove('active');
        });
        document.querySelector(`[data-mode="${mode}"]`).classList.add('active');
        
        // 切换控制区域显示
        document.querySelectorAll('.control-area').forEach(area => {
            area.classList.remove('active');
        });
        document.getElementById(`${mode}-control`).classList.add('active');
        
        // 停止当前运动
        this.stop();
        
        console.log(`🎮 切换到${mode === 'joystick' ? '摇杆' : '键盘'}控制模式`);
    }
    
    setupEmergencyStop() {
        const emergencyBtn = document.getElementById('emergencyStopBtn');
        if (emergencyBtn) {
            emergencyBtn.addEventListener('click', () => {
                this.emergencyStop();
            });
        }
        
        // 设置复位按钮
        const resetBtn = document.getElementById('resetBtn');
        if (resetBtn) {
            resetBtn.addEventListener('click', () => {
                this.resetSystem();
            });
        }
    }
    
    emergencyStop() {
        // 防止重复发送紧急停止消息
        if (this.emergencyStopActive) {
            console.log('⚠️ 紧急停止已经激活，跳过重复发送');
            return;
        }
        
        console.log('🚨 紧急停止！');
        
        // 激活紧急停止状态（防止重复发送）
        this.emergencyStopActive = true;
        
        // 锁定所有控制输入
        this.controlLocked = true;
        
        // 立即停止所有运动
        this.forceStop();
        
        // 停止waypoint跟踪（如果正在进行）
        this.stopWaypointFollowing();
        
        // 显示紧急停止通知（提示需要复位）
        this.showEmergencyStopNotification();
        
        // 发布紧急停止消息到状态机（仅发送一次）
        if (window.ros2Bridge && window.ros2Bridge.isConnected()) {
            const emergencyMsg = {
                active: true  // 使用active字段，而不是data
            };
            console.log('🔍 准备发送紧急停止消息（一次性）:', emergencyMsg);
            const success = window.ros2Bridge.publish('/emergency_stop', emergencyMsg);
            console.log('📤 已发送紧急停止信号到状态机（一次性）, 结果:', success);
            
            // 注意：不再自动解除紧急停止，需要用户手动按复位按钮
            console.log('ℹ️ 请按复位按钮恢复系统');
        }
    }
    
    clearEmergencyStop() {
        console.log('✅ 解除紧急停止');
        
        // 发布解除紧急停止消息到状态机
        if (window.ros2Bridge && window.ros2Bridge.isConnected()) {
            const emergencyMsg = {
                active: false  // 使用active字段，而不是data
            };
            console.log('🔍 准备发送解除紧急停止消息:', emergencyMsg);
            const success = window.ros2Bridge.publish('/emergency_stop', emergencyMsg);
            console.log('📤 已发送解除紧急停止信号到状态机, 结果:', success);
            return success;
        } else {
            console.error('❌ ROS2桥接未连接，无法发送解除紧急停止消息');
            return false;
        }
    }
    
    /**
     * 复位系统 - 取消紧急停止状态，恢复空闲
     */
    resetSystem() {
        console.log('🔄 系统复位中...');
        
        // 解除紧急停止状态
        this.emergencyStopActive = false;
        
        // 解锁控制输入
        this.controlLocked = false;
        
        // 移除紧急停止通知
        this.removeEmergencyStopNotification();
        
        // 发布解除紧急停止消息到状态机
        const clearSuccess = this.clearEmergencyStop();
        if (!clearSuccess) {
            console.error('❌ 发布解除紧急停止消息失败');
        }
        
        // 确保机器人停止
        this.forceStop();
        
        // 清除所有按键状态
        this.currentKeys.clear();
        this.isMoving = false;
        
        // 更新速度显示
        this.updateSpeedDisplay();
        
        // 显示复位通知
        this.showResetNotification();
        
        // 更新复位按钮状态（如果需要）
        this.updateResetButtonState();
        
        // 等待一小段时间后检查状态是否已更新
        setTimeout(() => {
            console.log('📊 复位后状态检查:');
            console.log('  - emergencyStopActive:', this.emergencyStopActive);
            console.log('  - controlLocked:', this.controlLocked);
            console.log('  - 等待状态机更新状态...');
        }, 500);
        
        console.log('✅ 系统复位完成，控制已解锁，可以开始导航');
    }
    
    /**
     * 显示复位通知
     */
    showResetNotification() {
        const notification = document.createElement('div');
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            background: #27ae60;
            color: white;
            padding: 1rem 2rem;
            border-radius: 8px;
            font-weight: 600;
            z-index: 10000;
            animation: slideIn 0.3s ease;
        `;
        notification.innerHTML = '✅ 系统已复位，恢复空闲状态';
        document.body.appendChild(notification);
        
        // 3秒后移除通知
        setTimeout(() => {
            notification.remove();
        }, 3000);
    }
    
    /**
     * 更新复位按钮状态
     */
    updateResetButtonState() {
        const resetBtn = document.getElementById('resetBtn');
        if (resetBtn) {
            // 可以根据需要禁用/启用复位按钮
            // 例如：只有在紧急停止状态下才允许复位
        }
    }
    
    showEmergencyStopNotification() {
        // 创建紧急停止通知（持续显示，直到复位）
        // 先移除之前的通知（如果有）
        const existingNotification = document.getElementById('emergencyStopNotification');
        if (existingNotification) {
            existingNotification.remove();
        }
        
        const notification = document.createElement('div');
        notification.id = 'emergencyStopNotification';
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            background: #e74c3c;
            color: white;
            padding: 1rem 2rem;
            border-radius: 8px;
            font-weight: 600;
            z-index: 10000;
            animation: slideIn 0.3s ease;
            box-shadow: 0 4px 12px rgba(231, 76, 60, 0.4);
        `;
        notification.innerHTML = '🚨 紧急停止已激活<br><small style="opacity: 0.9;">请按复位按钮恢复系统</small>';
        document.body.appendChild(notification);
    }
    
    removeEmergencyStopNotification() {
        // 移除紧急停止通知
        const notification = document.getElementById('emergencyStopNotification');
        if (notification) {
            notification.remove();
        }
    }
    
    highlightKeyboardButton(key) {
        const keyBtn = document.querySelector(`[data-key="${key}"]`);
        if (keyBtn) {
            keyBtn.classList.add('active');
        }
    }
    
    unhighlightKeyboardButton(key) {
        const keyBtn = document.querySelector(`[data-key="${key}"]`);
        if (keyBtn) {
            keyBtn.classList.remove('active');
        }
    }
    
    setupSpeedControls() {
        const maxLinearSlider = document.getElementById('maxLinearSpeed');
        const maxLateralSlider = document.getElementById('maxLateralSpeed');
        const maxAngularSlider = document.getElementById('maxAngularSpeed');
        const maxLinearValue = document.getElementById('maxLinearSpeedValue');
        const maxLateralValue = document.getElementById('maxLateralSpeedValue');
        const maxAngularValue = document.getElementById('maxAngularSpeedValue');
        
        if (maxLinearSlider) {
            maxLinearSlider.addEventListener('input', (e) => {
                this.maxLinearSpeed = parseFloat(e.target.value);
                maxLinearValue.textContent = e.target.value;
            });
        }
        
        if (maxLateralSlider) {
            maxLateralSlider.addEventListener('input', (e) => {
                this.maxLateralSpeed = parseFloat(e.target.value);
                maxLateralValue.textContent = e.target.value;
            });
        }
        
        if (maxAngularSlider) {
            maxAngularSlider.addEventListener('input', (e) => {
                this.maxAngularSpeed = parseFloat(e.target.value);
                maxAngularValue.textContent = e.target.value;
            });
        }
    }
    
    handleKeyDown(event) {
        if (event.repeat) return;
        
        // 排除输入框、文本域等表单元素
        const target = event.target;
        if (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA' || target.tagName === 'SELECT') {
            return;
        }
        
        // 安全检查：确保event.key存在
        if (!event.key) {
            console.warn('⚠️ 键盘事件缺少key属性:', {
                event: event,
                type: event.type,
                key: event.key,
                code: event.code,
                keyCode: event.keyCode,
                target: target.tagName
            });
            return;
        }
        
        const key = event.key.toLowerCase();
        this.currentKeys.add(key);
        this.updateMovement();
        this.updateButtonStates();
        
        // 高亮键盘按钮
        this.highlightKeyboardButton(key);
    }
    
    handleKeyUp(event) {
        // 排除输入框、文本域等表单元素
        const target = event.target;
        if (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA' || target.tagName === 'SELECT') {
            return;
        }
        
        // 安全检查：确保event.key存在
        if (!event.key) {
            console.warn('⚠️ 键盘事件缺少key属性:', {
                event: event,
                type: event.type,
                key: event.key,
                code: event.code,
                keyCode: event.keyCode,
                target: target.tagName
            });
            return;
        }
        
        const key = event.key.toLowerCase();
        this.currentKeys.delete(key);
        this.updateMovement();
        this.updateButtonStates();
        
        // 取消高亮键盘按钮
        this.unhighlightKeyboardButton(key);
    }
    
    handleButtonDown(event) {
        const action = event.target.dataset.action;
        if (action) {
            this.currentKeys.add(action);
            this.updateMovement();
            this.updateButtonStates();
        }
    }
    
    handleButtonUp(event) {
        const action = event.target.dataset.action;
        if (action) {
            this.currentKeys.delete(action);
            this.updateMovement();
            this.updateButtonStates();
        }
    }
    
    handleRotationButtonDown(event) {
        const action = event.target.dataset.action;
        if (action) {
            this.currentKeys.add(action);
            this.updateMovement();
            this.updateRotationButtonStates();
        }
    }
    
    handleRotationButtonUp(event) {
        const action = event.target.dataset.action;
        if (action) {
            this.currentKeys.delete(action);
            this.updateMovement();
            this.updateRotationButtonStates();
        }
    }
    
    updateMovement() {
        let linear = 0.0;      // 前进/后退 (linear.x)
        let lateral = 0.0;      // 左移/右移 (linear.y)
        let angular = 0.0;      // 旋转 (angular.z)
        
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
        
        // 旋转控制 (q/e) - 提高灵敏度
        if (this.currentKeys.has('q') || this.currentKeys.has('rotate_left')) {
            angular = this.maxAngularSpeed * 1.2; // 提高20%的灵敏度
        }
        if (this.currentKeys.has('e') || this.currentKeys.has('rotate_right')) {
            angular = -this.maxAngularSpeed * 1.2; // 提高20%的灵敏度
        }
        
        // 停止 (x)
        if (this.currentKeys.has('x') || this.currentKeys.has('stop')) {
            linear = 0.0;
            lateral = 0.0;
            angular = 0.0;
        }
        
        // 检查是否有任何输入
        const hasInput = linear !== 0 || lateral !== 0 || angular !== 0;
        const wasMoving = this.isMoving;
        this.isMoving = hasInput;
        
        // 更新速度值
        this.linearSpeed = linear;
        this.lateralSpeed = lateral;
        this.angularSpeed = angular;
        
        // 更新显示
        this.updateSpeedDisplay();
        
        // 处理控制命令
        if (hasInput) {
            // 有输入时发送控制命令
            this.sendControlCommand();
        } else if (wasMoving) {
            // 从运动状态变为停止状态，发送归零命令防止漂移
            this.sendZeroCommand();
        }
        // 注意：如果一直处于停止状态，不发送任何消息，避免与follower冲突
    }
    
    updateButtonStates() {
        document.querySelectorAll('.control-btn').forEach(btn => {
            const action = btn.dataset.action;
            if (this.currentKeys.has(action)) {
                btn.classList.add('active');
            } else {
                btn.classList.remove('active');
            }
        });
    }
    
    updateRotationButtonStates() {
        document.querySelectorAll('.rotation-btn').forEach(btn => {
            const action = btn.dataset.action;
            if (this.currentKeys.has(action)) {
                btn.classList.add('active');
            } else {
                btn.classList.remove('active');
            }
        });
    }
    
    updateSpeedDisplay() {
        const linearElement = document.getElementById('linearSpeed');
        const lateralElement = document.getElementById('lateralSpeed');
        const angularElement = document.getElementById('angularSpeed');
        
        if (linearElement) {
            linearElement.textContent = this.linearSpeed.toFixed(2);
        }
        if (lateralElement) {
            lateralElement.textContent = this.lateralSpeed.toFixed(2);
        }
        if (angularElement) {
            angularElement.textContent = this.angularSpeed.toFixed(2);
        }
    }
    
    sendControlCommand() {
        if (!window.ros2Bridge || !window.ros2Bridge.isConnected()) {
            return;
        }
        
        // 检查紧急停止状态 - 紧急停止时禁止所有控制输入
        if (this.emergencyStopActive) {
            console.log('🚨 紧急停止状态，禁止控制输入。请按复位按钮恢复系统');
            return;
        }
        
        // 检查控制锁定状态
        if (this.controlLocked) {
            console.log('🔒 控制已锁定，跳过命令发送');
            return;
        }
        
        // 严格的互斥控制：如果正在执行waypoint跟踪，且没有手动控制输入，则不发送命令
        if (this.waypointFollowing && !this.isMoving) {
            console.log('🚫 Waypoint跟踪中，跳过手动控制命令');
            return;
        }
        
        // 如果waypoint跟踪正在进行，但用户有手动输入，则立即停止waypoint跟踪
        if (this.waypointFollowing && this.isMoving) {
            console.log('🔄 检测到手动控制输入，停止waypoint跟踪');
            this.controlLocked = true;  // 锁定控制
            this.stopWaypointFollowing();
            // 等待一小段时间确保waypoint跟踪完全停止
            setTimeout(() => {
                this.controlLocked = false;  // 解锁控制
                this.sendControlCommand();
            }, 200);
            return;
        }
        
        // 只有在没有waypoint跟踪或waypoint跟踪已停止时才发送命令
        if (!this.waypointFollowing) {
            const twistMessage = {
                linear: {
                    x: this.linearSpeed,    // 前进/后退
                    y: -this.lateralSpeed,  // 左移/右移 (反转方向)
                    z: 0.0
                },
                angular: {
                    x: 0.0,
                    y: 0.0,
                    z: this.angularSpeed    // 旋转
                }
            };
            
            window.ros2Bridge.publish('/cmd_vel', twistMessage);
        }
    }
    
    // 发送停止命令 - 只在明确需要停止时调用
    sendStopCommand() {
        if (!window.ros2Bridge || !window.ros2Bridge.isConnected()) {
            return;
        }
        
        // 检查紧急停止状态 - 紧急停止时仍可发送停止命令（确保停止）
        // 但不检查锁定状态，因为紧急停止需要强制停止
        
        // 如果正在执行waypoint跟踪，不发送停止命令
        if (this.waypointFollowing && !this.emergencyStopActive) {
            return;
        }
        
        const stopMessage = {
            linear: {
                x: 0.0,
                y: 0.0,
                z: 0.0
            },
            angular: {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        };
        
        window.ros2Bridge.publish('/cmd_vel', stopMessage);
    }
    
    
    // 发送归零命令 - 防止小车漂移
    sendZeroCommand() {
        if (!window.ros2Bridge || !window.ros2Bridge.isConnected()) {
            console.warn('⚠️ ROS2桥接未连接，无法发送归零命令');
            return;
        }
        
        // 紧急停止时不需要发送归零命令（已经强制停止了）
        if (this.emergencyStopActive) {
            return;
        }
        
        // 检查控制锁定状态
        if (this.controlLocked) {
            console.log('🚫 控制被锁定，跳过归零命令');
            return;
        }
        
        // 如果正在执行waypoint跟踪，不发送归零命令
        if (this.waypointFollowing) {
            console.log('🚫 正在执行waypoint跟踪，跳过归零命令');
            return;
        }
        
        const zeroMessage = {
            linear: {
                x: 0.0,
                y: 0.0,
                z: 0.0
            },
            angular: {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        };
        
        try {
            window.ros2Bridge.publish('/cmd_vel', zeroMessage);
            console.log('🛑 发送归零命令，防止小车漂移');
            
            // 更新速度显示为0
            this.updateSpeedDisplay();
        } catch (error) {
            console.error('❌ 发送归零命令失败:', error);
        }
    }
    
    // 强制停止 - 用于紧急停止或明确停止指令
    forceStop() {
        this.linearSpeed = 0.0;
        this.lateralSpeed = 0.0;
        this.angularSpeed = 0.0;
        this.currentKeys.clear();
        this.isMoving = false;
        
        this.updateSpeedDisplay();
        this.sendStopCommand(); // 明确发送停止命令
        this.updateButtonStates();
    }
    
    // 发送单次归零命令 - 用于摇杆释放时的可靠停止
    sendSingleZeroCommand() {
        if (!window.ros2Bridge || !window.ros2Bridge.isConnected()) {
            console.warn('⚠️ ROS2桥接未连接，无法发送归零命令');
            return;
        }
        
        const zeroMessage = {
            linear: {
                x: 0.0,
                y: 0.0,
                z: 0.0
            },
            angular: {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        };
        
        try {
            window.ros2Bridge.publish('/cmd_vel', zeroMessage);
            console.log('🛑 发送单次归零命令');
            
            // 更新速度显示为0
            this.updateSpeedDisplay();
        } catch (error) {
            console.error('❌ 发送单次归零命令失败:', error);
        }
    }
    
    // 摇杆控制 - 支持麦克纳姆轮全向移动
    updateJoystickControl(linear, lateral, angular) {
        // 应用摇杆死区处理
        const magnitude = Math.sqrt(linear * linear + lateral * lateral);
        const deadzone = 0.05; // 5%死区
        
        if (magnitude < deadzone) {
            linear = 0.0;
            lateral = 0.0;
        }
        
        this.linearSpeed = linear * this.maxLinearSpeed;
        this.lateralSpeed = lateral * this.maxLateralSpeed;
        this.angularSpeed = angular * this.maxAngularSpeed;
        
        // 检查是否有任何输入
        const hasInput = this.linearSpeed !== 0 || this.lateralSpeed !== 0 || this.angularSpeed !== 0;
        this.isMoving = hasInput;
        
        this.updateSpeedDisplay();
        
        // 处理控制命令
        if (hasInput) {
            // 有输入时发送控制命令
            this.sendControlCommand();
        } else if (wasMoving) {
            // 从运动状态变为停止状态，发送归零命令防止漂移
            this.sendZeroCommand();
        }
        // 注意：如果一直处于停止状态，不发送任何消息，避免与follower冲突
    }
    
    // 停止机器人
    stop() {
        this.forceStop();
    }
    
    // 停止waypoint跟踪
    stopWaypointFollowing() {
        console.log('🛑 请求停止waypoint跟踪');
        this.waypointFollowing = false;
        
        // 调用waypoint系统的停止方法
        if (window.waypointSystem) {
            try {
                // 如果waypoint系统正在跟踪，调用停止方法
                if (window.waypointSystem.isFollowing) {
                    console.log('🛑 紧急停止：正在停止waypoint跟踪...');
                    window.waypointSystem.stopFollowing();
                    console.log('✅ 已通知waypoint系统停止跟踪');
                } else {
                    console.log('ℹ️ Waypoint系统未在跟踪状态，跳过停止操作');
                }
            } catch (error) {
                console.error('❌ 停止waypoint跟踪时出错:', error);
            }
        } else {
            console.warn('⚠️ WaypointSystem未正确初始化');
        }
    }
    
    // 设置waypoint跟踪状态
    setWaypointFollowing(following) {
        this.waypointFollowing = following;
        if (following) {
            this.controlLocked = true;  // 开始waypoint跟踪时锁定手动控制
            console.log('🔒 Waypoint跟踪开始，锁定手动控制');
        } else {
            this.controlLocked = false;  // 停止waypoint跟踪时解锁手动控制
            console.log('🔓 Waypoint跟踪停止，解锁手动控制');
        }
        console.log(`Waypoint跟踪状态: ${following ? '开始' : '停止'}`);
    }
    
    // 导航控制
    setGoal() {
        const goalX = parseFloat(document.getElementById('goalX').value) || 0.0;
        const goalY = parseFloat(document.getElementById('goalY').value) || 0.0;
        const goalZ = parseFloat(document.getElementById('goalZ').value) || 0.0;
        
        if (!window.ros2Bridge || !window.ros2Bridge.isConnected()) {
            alert('ROS2连接未建立，无法设置目标点');
            return;
        }
        
        const goalMessage = {
            header: {
                stamp: {
                    sec: Math.floor(Date.now() / 1000),
                    nanosec: (Date.now() % 1000) * 1000000
                },
                frame_id: 'map'
            },
            pose: {
                position: {
                    x: goalX,
                    y: goalY,
                    z: goalZ
                },
                orientation: {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0
                }
            }
        };
        
        const success = window.ros2Bridge.publish('/goal_pose', goalMessage);
        if (success) {
            console.log(`🎯 目标点已设置: (${goalX}, ${goalY}, ${goalZ})`);
            this.showNotification('目标点已设置', 'success');
        } else {
            console.error('🚨 设置目标点失败');
            this.showNotification('设置目标点失败', 'error');
        }
    }
    
    cancelGoal() {
        if (!window.ros2Bridge || !window.ros2Bridge.isConnected()) {
            alert('ROS2连接未建立，无法取消导航');
            return;
        }
        
        // 发送停止命令
        this.stop();
        console.log('🛑 导航已取消');
        this.showNotification('导航已取消', 'info');
    }
    
    // 显示通知
    showNotification(message, type = 'info') {
        // 创建通知元素
        const notification = document.createElement('div');
        notification.className = `notification ${type}`;
        notification.textContent = message;
        
        // 添加样式
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 1rem 1.5rem;
            border-radius: 8px;
            color: white;
            font-weight: 500;
            z-index: 1000;
            animation: slideIn 0.3s ease-out;
        `;
        
        // 设置背景颜色
        const colors = {
            success: '#27ae60',
            error: '#e74c3c',
            info: '#3498db',
            warning: '#f39c12'
        };
        notification.style.backgroundColor = colors[type] || colors.info;
        
        // 添加到页面
        document.body.appendChild(notification);
        
        // 自动移除
        setTimeout(() => {
            notification.style.animation = 'slideOut 0.3s ease-in';
            setTimeout(() => {
                if (notification.parentNode) {
                    notification.parentNode.removeChild(notification);
                }
            }, 300);
        }, 3000);
    }
    
    // 控制循环 - 只在有输入时发送命令
    startControlLoop() {
        setInterval(() => {
            // 只有在有输入且没有waypoint跟踪时才发送命令
            if (this.isMoving && !this.waypointFollowing && !this.controlLocked) {
                this.sendControlCommand();
            }
            // 注意：没有输入时不发送任何消息，避免与follower冲突
        }, 50); // 20Hz控制频率，提高旋转响应速度
    }
    
    // 获取当前速度
    getCurrentSpeed() {
        return {
            linear: this.linearSpeed,
            lateral: this.lateralSpeed,
            angular: this.angularSpeed
        };
    }
    
    // 设置最大速度
    setMaxSpeed(linear, lateral, angular) {
        this.maxLinearSpeed = linear;
        this.maxLateralSpeed = lateral;
        this.maxAngularSpeed = angular;
    }
}

// 创建全局机器人控制器实例
window.robotController = new RobotController();
