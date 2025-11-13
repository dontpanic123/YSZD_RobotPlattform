/**
 * 主应用程序
 * 协调各个模块的工作
 */

class RobotWebApp {
    constructor() {
        this.ros2Bridge = null;
        this.robotController = null;
        this.virtualJoystick = null;
        this.cameraDisplay = null;
        this.apriltagDisplay = null;
        this.robotStatus = null;
        this.waypointSystem = null;
        
        // 全局cmd_vel发布控制
        this.cmdVelPublisher = null;
        this.cmdVelLock = false;
        
        this.init();
    }
    
    init() {
        console.log('🚀 机器人Web应用启动中...');
        
        // 等待DOM加载完成
        if (document.readyState === 'loading') {
            document.addEventListener('DOMContentLoaded', () => this.start());
        } else {
            this.start();
        }
    }
    
    start() {
        this.setupModules();
        this.setupEventListeners();
        this.startUpdateLoop();
        this.updateTime();
        
        console.log('✅ 机器人Web应用启动完成');
    }
    
    setupModules() {
        // 初始化各个模块
        this.ros2Bridge = window.ros2Bridge;
        this.robotController = window.robotController;
        this.virtualJoystick = window.virtualJoystick;
        this.waypointSystem = window.waypointSystem;
        
        // 设置摄像头显示
        this.setupCameraDisplay();
        
        // 延迟设置订阅，确保连接建立
        setTimeout(() => {
            this.setupAprilTagDisplay();
            this.setupRobotStatus();
        }, 2000);
        
        // 设置Waypoint系统模块
        this.setupWaypointSystem();
    }
    
    setupEventListeners() {
        // 连接状态变化
        if (this.ros2Bridge) {
            // 监听连接状态变化
            setInterval(() => {
                this.updateConnectionQuality();
            }, 1000);
        }
        
        // 页面可见性变化
        document.addEventListener('visibilitychange', () => {
            if (document.hidden) {
                this.pauseUpdates();
            } else {
                this.resumeUpdates();
            }
        });
        
        // 窗口大小变化
        window.addEventListener('resize', () => {
            this.handleResize();
        });
        
        // Tab切换
        this.setupTabSwitching();
    }
    
    setupCameraDisplay() {
        if (!this.ros2Bridge) return;
        
        // 订阅摄像头图像
        this.ros2Bridge.subscribe('/camera/image_raw', (message) => {
            this.updateCameraImage(message);
        });
        
        // 订阅AprilTag检测图像
        this.ros2Bridge.subscribe('/apriltag_detection', (message) => {
            this.updateAprilTagImage(message);
        });
    }
    
    setupAprilTagDisplay() {
        if (!this.ros2Bridge) {
            console.warn('⚠️ ROS2桥接未初始化');
            return;
        }
        
        // 等待连接建立后再订阅
        if (!this.ros2Bridge.connected) {
            setTimeout(() => this.setupAprilTagDisplay(), 1000);
            return;
        }
        
        // 订阅AprilTag状态
        this.ros2Bridge.subscribe('/apriltag_status', (message) => {
            this.updateAprilTagStatus(message);
        });
        
        // 订阅AprilTag位姿
        this.ros2Bridge.subscribe('/apriltag_pose', (message) => {
            this.updateAprilTagPose(message);
        });
        
        // 订阅机器人状态
        this.ros2Bridge.subscribe('/robot_state', (message) => {
            this.updateRobotState(message);
        });
        
        // 订阅机器人定位
        this.ros2Bridge.subscribe('/robot_location', (message) => {
            this.updateRobotLocation(message);
        });
    }
    
    setupRobotStatus() {
        if (!this.ros2Bridge) {
            console.warn('⚠️ ROS2桥接未初始化');
            return;
        }
        
        // 等待连接建立后再订阅
        if (!this.ros2Bridge.connected) {
            setTimeout(() => this.setupRobotStatus(), 1000);
            return;
        }
        
        // 订阅里程计数据
        this.ros2Bridge.subscribe('/odom', (message) => {
            this.updateRobotOdometry(message);
        });
    }
    
    
    setupWaypointSystem() {
        if (!this.waypointSystem) {
            console.warn('⚠️ Waypoint系统模块未找到');
            return;
        }
        
        console.log('🎯 Waypoint系统模块已连接');
    }
    
    updateCameraImage(message) {
        try {
            // 将ROS图像消息转换为可显示的图像
            const imageData = this.convertImageMessage(message);
            const cameraImage = document.getElementById('cameraImage');
            
            if (cameraImage && imageData) {
                cameraImage.src = imageData;
                this.updateVideoInfo(message);
            }
        } catch (error) {
            console.error('🚨 更新摄像头图像时出错:', error);
        }
    }
    
    updateAprilTagImage(message) {
        try {
            // 更新AprilTag检测图像
            const imageData = this.convertImageMessage(message);
            const cameraImage = document.getElementById('cameraImage');
            
            if (cameraImage && imageData) {
                cameraImage.src = imageData;
            }
        } catch (error) {
            console.error('🚨 更新AprilTag图像时出错:', error);
        }
    }
    
    convertImageMessage(message) {
        try {
            // 如果是后端通过WebSocket发送的JPEG Base64数据，直接生成Data URL
            if (message && message.encoding === 'jpeg' && typeof message.data === 'string') {
                return `data:image/jpeg;base64,${message.data}`;
            }
            
            // 创建ImageData对象
            const width = message.width;
            const height = message.height;
            const data = new Uint8ClampedArray(message.data);
            
            // 创建ImageData
            const imageData = new ImageData(data, width, height);
            
            // 创建canvas来转换图像
            const canvas = document.createElement('canvas');
            canvas.width = width;
            canvas.height = height;
            const ctx = canvas.getContext('2d');
            
            // 将图像数据绘制到canvas
            ctx.putImageData(imageData, 0, 0);
            
            // 返回data URL
            return canvas.toDataURL('image/png');
        } catch (error) {
            console.error('🚨 转换图像消息时出错:', error);
            return null;
        }
    }
    
    updateVideoInfo(message) {
        const fpsElement = document.getElementById('videoFPS');
        const sizeElement = document.getElementById('videoSize');
        
        if (fpsElement) {
            fpsElement.textContent = `FPS: ${message.header.stamp.sec}`;
        }
        
        if (sizeElement) {
            sizeElement.textContent = `${message.width}x${message.height}`;
        }
    }
    
    updateAprilTagStatus(message) {
        const statusElement = document.getElementById('apriltagStatus');
        const detectedTagsElement = document.getElementById('detectedTags');
        
        if (statusElement) {
            statusElement.textContent = message.data;
            
            if (message.data.includes('检测到')) {
                statusElement.className = 'detection-status detected';
            } else {
                statusElement.className = 'detection-status not-detected';
            }
        }
    }
    
    updateAprilTagPose(message) {
        // 更新检测到的标签信息
        const detectedTagsElement = document.getElementById('detectedTags');
        if (!detectedTagsElement) return;
        
        const position = message.pose.position;
        const orientation = message.pose.orientation;
        let tagId = '未知';
        if (message.header && message.header.frame_id) {
            const fid = String(message.header.frame_id);
            if (fid.startsWith('apriltag_')) {
                tagId = fid.substring('apriltag_'.length);
            } else {
                tagId = fid;
            }
        }
        
        // 计算距离
        const distance = Math.sqrt(
            position.x * position.x + 
            position.y * position.y + 
            position.z * position.z
        );
        
        // 创建标签信息元素
        const tagElement = document.createElement('div');
        tagElement.className = 'tag-item';
        tagElement.innerHTML = `
            <div class="tag-id">AprilTag 检测到 (ID: ${tagId})</div>
            <div class="tag-info">
                <div>距离: ${distance.toFixed(2)}m</div>
                <div>位置: (${position.x.toFixed(2)}, ${position.y.toFixed(2)}, ${position.z.toFixed(2)})</div>
            </div>
        `;
        
        // 更新显示
        detectedTagsElement.innerHTML = '';
        detectedTagsElement.appendChild(tagElement);
    }
    
    updateRobotState(message) {
        // 更新机器人当前状态
        console.log('🤖 更新机器人状态:', message);
        const stateElement = document.getElementById('robotCurrentState');
        if (stateElement) {
            // 从消息中获取状态，支持不同的消息格式
            const state = message.data || message.state || 'unknown';
            console.log('🤖 设置状态为:', state);
            stateElement.textContent = this.getStateDisplayName(state);
            stateElement.className = `state-indicator ${state}`;
        } else {
            console.warn('⚠️ 找不到robotCurrentState元素');
        }
    }
    
    updateRobotLocation(message) {
        // 更新机器人当前定位
        console.log('📍 更新机器人定位:', message);
        const locationElement = document.getElementById('robotCurrentLocation');
        if (locationElement) {
            // 从消息中获取定位，支持不同的消息格式
            const location = message.data || message.location || '未定位';
            console.log('📍 设置定位为:', location);
            locationElement.textContent = location;
            
            // 根据定位类型设置不同的样式
            let locationClass = 'location-indicator';
            if (location === '充电桩') {
                locationClass += ' location-charging';
            } else if (location === '装载点') {
                locationClass += ' location-loading';
            } else if (location === '送达点') {
                locationClass += ' location-delivery';
            } else if (location === '未定位') {
                locationClass += ' location-unknown';
            } else {
                locationClass += ' location-other';
            }
            locationElement.className = locationClass;
        } else {
            console.warn('⚠️ 找不到robotCurrentLocation元素');
        }
    }
    
    getStateDisplayName(state) {
        const stateNames = {
            'idle': '空闲',
            'manual_control': '手动控制',
            'auto_navigation': '自动导航',
            'apriltag_tracking': 'AprilTag跟踪',
            'emergency_stop': '紧急停止',
            'charging': '充电中',
            'error': '错误'
        };
        return stateNames[state] || state;
    }
    
    updateRobotOdometry(message) {
        const position = message.pose.pose.position;
        const orientation = message.pose.pose.orientation;
        const velocity = message.twist.twist;
        
        // 更新位置显示
        const positionElement = document.getElementById('robotPosition');
        if (positionElement) {
            positionElement.textContent = `${position.x.toFixed(2)}, ${position.y.toFixed(2)}, ${position.z.toFixed(2)}`;
        }
        
        // 更新方向显示
        const orientationElement = document.getElementById('robotOrientation');
        if (orientationElement) {
            const euler = this.quaternionToEuler(orientation);
            orientationElement.textContent = `${euler.roll.toFixed(2)}, ${euler.pitch.toFixed(2)}, ${euler.yaw.toFixed(2)}`;
        }
        
        // 更新速度显示
        const velocityElement = document.getElementById('robotVelocity');
        if (velocityElement) {
            // 只显示X,Y线速度和Z角速度
            const linearX = velocity.linear.x.toFixed(2);
            const linearY = velocity.linear.y.toFixed(2);
            const angularZ = velocity.angular.z;
            
            const angularZRad = angularZ.toFixed(2);
            const angularZDeg = (angularZ * 180 / Math.PI).toFixed(1);
            
            velocityElement.textContent = `线性(${linearX}, ${linearY}), 角速度Z(${angularZRad}rad/${angularZDeg}°)`;
        }
    }
    
    quaternionToEuler(q) {
        const x = q.x;
        const y = q.y;
        const z = q.z;
        const w = q.w;
        
        const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        const pitch = Math.asin(2 * (w * y - z * x));
        const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        
        return { roll, pitch, yaw };
    }
    
    updateConnectionQuality() {
        const qualityElement = document.getElementById('connectionQuality');
        if (qualityElement && this.ros2Bridge) {
            qualityElement.textContent = this.ros2Bridge.getConnectionQuality();
        }
    }
    
    updateTime() {
        const timeElement = document.getElementById('currentTime');
        if (timeElement) {
            const now = new Date();
            timeElement.textContent = now.toLocaleTimeString();
        }
        
        // 每秒更新一次
        setTimeout(() => this.updateTime(), 1000);
    }
    
    startUpdateLoop() {
        // 启动更新循环
        setInterval(() => {
            this.updateConnectionQuality();
        }, 1000);
    }
    
    pauseUpdates() {
        console.log('⏸️ 暂停更新');
    }
    
    resumeUpdates() {
        console.log('▶️ 恢复更新');
    }
    
    handleResize() {
        // 处理窗口大小变化
        if (this.virtualJoystick) {
            this.virtualJoystick.setupDimensions();
        }
    }
    
    setupTabSwitching() {
        console.log('🔄 设置Tab切换功能...');
        
        const tabButtons = document.querySelectorAll('.tab-btn');
        const tabPanels = document.querySelectorAll('.tab-panel');
        
        console.log('🔍 找到的Tab按钮:', tabButtons.length);
        console.log('🔍 找到的Tab面板:', tabPanels.length);
        
        // 检查waypoint-tab元素
        const waypointTab = document.getElementById('waypoint-tab');
        console.log('🔍 waypoint-tab元素:', waypointTab);
        
        tabButtons.forEach(button => {
            button.addEventListener('click', () => {
                const targetTab = button.getAttribute('data-tab');
                console.log('🔄 切换到Tab:', targetTab);
                
                // 移除所有活动状态
                tabButtons.forEach(btn => btn.classList.remove('active'));
                tabPanels.forEach(panel => panel.classList.remove('active'));
                
                // 激活当前按钮和面板
                button.classList.add('active');
                const targetPanel = document.getElementById(targetTab + '-tab');
                
                if (targetPanel) {
                    targetPanel.classList.add('active');
                    console.log('✅ Tab切换成功:', targetTab);
                    console.log('📊 目标面板类名:', targetPanel.className);
                    console.log('📊 目标面板样式:', window.getComputedStyle(targetPanel).display);
                    
                    // 如果是waypoint tab，检查内容
                    if (targetTab === 'waypoint') {
                        console.log('🎯 切换到Waypoint Tab，检查内容...');
                        const waypointContent = targetPanel.querySelector('.control-panel');
                        if (waypointContent) {
                            console.log('✅ Waypoint内容存在');
                        } else {
                            console.warn('⚠️ Waypoint内容不存在');
                        }
                    }
                } else {
                    console.error('❌ 找不到目标面板:', targetTab + '-tab');
                }
            });
        });
        
        console.log('✅ Tab切换功能设置完成');
    }
    
    // 显示系统信息
    showSystemInfo() {
        const info = {
            'ROS2连接': this.ros2Bridge?.isConnected() ? '已连接' : '未连接',
            '话题数量': this.ros2Bridge?.getTopics().length || 0,
            '连接质量': this.ros2Bridge?.getConnectionQuality() || '未知',
            '当前时间': new Date().toLocaleString()
        };
        
        console.log('📊 系统信息:', info);
        return info;
    }
}

// 全局诊断函数
function openDiagnosticTab() {
    console.log('🔍 打开诊断Tab');
    // 这个函数现在由Tab切换系统自动处理
}

// 启动应用程序
document.addEventListener('DOMContentLoaded', () => {
    window.robotWebApp = new RobotWebApp();
    
    // 添加全局错误处理
    window.addEventListener('error', (event) => {
        console.error('🚨 全局错误:', event.error);
    });
    
    // 添加未处理的Promise拒绝处理
    window.addEventListener('unhandledrejection', (event) => {
        console.error('🚨 未处理的Promise拒绝:', event.reason);
    });
});

// 添加CSS动画
const style = document.createElement('style');
style.textContent = `
    @keyframes slideIn {
        from {
            transform: translateX(100%);
            opacity: 0;
        }
        to {
            transform: translateX(0);
            opacity: 1;
        }
    }
    
    @keyframes slideOut {
        from {
            transform: translateX(0);
            opacity: 1;
        }
        to {
            transform: translateX(100%);
            opacity: 0;
        }
    }
`;
document.head.appendChild(style);
