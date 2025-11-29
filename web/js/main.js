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
        
        // 定位状态管理
        this.lastApriltagDetectionTime = 0;
        this.locationUpdateTimer = null;
        
        // 摄像头视图模式: 'rgb', 'depth', 'overlay'
        this.currentViewMode = 'rgb';
        // 当前选择的摄像头: 'front' or 'back'
        this.currentCamera = 'front';
        // 分别存储前置和后置摄像头的图像数据
        this.rgbImageData = { front: null, back: null };
        this.depthImageData = { front: null, back: null };
        
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
        this.startLocationStatusCheck();
        
        console.log('✅ 机器人Web应用启动完成');
    }
    
    startLocationStatusCheck() {
        // 每0.5秒检查一次定位状态，更新背景色
        this.locationUpdateTimer = setInterval(() => {
            this.updateLocationStatusColor();
        }, 500);
    }
    
    updateLocationStatusColor() {
        const locationElement = document.getElementById('robotCurrentLocation');
        if (!locationElement) return;
        
        const location = locationElement.textContent;
        
        // 如果是"未定位"，不改变颜色
        if (location === '未定位') {
            // 移除所有状态颜色类
            locationElement.classList.remove('location-fresh', 'location-stale');
            return;
        }
        
        // 如果没有检测时间戳，不改变颜色
        if (this.lastApriltagDetectionTime <= 0) {
            return;
        }
        
        // 计算距离最后检测的时间
        // 注意：服务器发送的时间戳是Unix时间戳（秒），需要与当前时间（秒）比较
        const currentTime = Date.now() / 1000; // 转换为秒
        const timeSinceDetection = currentTime - this.lastApriltagDetectionTime;
        
        // 移除之前的颜色类
        locationElement.classList.remove('location-fresh', 'location-stale');
        
        // 根据时间设置颜色
        if (timeSinceDetection <= 5.0) {
            // 5秒内：绿色
            locationElement.classList.add('location-fresh');
        } else {
            // 超过5秒：黄色
            locationElement.classList.add('location-stale');
        }
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
            this.setupUltrasonicDisplay();
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
        
        // 订阅前置摄像头图像
        this.ros2Bridge.subscribe('/camera_front/image_raw', (message) => {
            this.updateCameraImage(message, 'front');
        });
        
        // 订阅后置摄像头图像
        this.ros2Bridge.subscribe('/camera_back/image_raw', (message) => {
            this.updateCameraImage(message, 'back');
        });
        
        // 订阅前置摄像头深度图像
        this.ros2Bridge.subscribe('/camera_front/depth/image_raw', (message) => {
            this.updateDepthImage(message, 'front');
        });
        
        // 订阅后置摄像头深度图像
        this.ros2Bridge.subscribe('/camera_back/depth/image_raw', (message) => {
            this.updateDepthImage(message, 'back');
        });
        
        // 订阅AprilTag检测图像
        this.ros2Bridge.subscribe('/apriltag_detection', (message) => {
            this.updateAprilTagImage(message);
        });
        
        // 设置摄像头切换按钮
        this.setupCameraToggle();
        // 设置摄像头选择按钮
        this.setupCameraSelection();
        // 初始化按钮显示
        this.updateCameraSelectButton();
    }
    
    setupCameraToggle() {
        const toggleBtn = document.getElementById('cameraToggleBtn');
        if (!toggleBtn) return;
        
        toggleBtn.addEventListener('click', () => {
            this.toggleCameraView();
        });
    }
    
    setupCameraSelection() {
        const selectBtn = document.getElementById('cameraSelectBtn');
        if (!selectBtn) return;
        
        selectBtn.addEventListener('click', () => {
            this.switchCamera();
        });
    }
    
    switchCamera() {
        // 切换前置和后置摄像头
        this.currentCamera = this.currentCamera === 'front' ? 'back' : 'front';
        this.updateCameraDisplay();
        this.updateCameraSelectButton();
    }
    
    updateCameraSelectButton() {
        const selectBtn = document.getElementById('cameraSelectBtn');
        const selectLabel = document.getElementById('cameraSelectLabel');
        
        if (!selectBtn || !selectLabel) return;
        
        const labels = {
            'front': '前置',
            'back': '后置'
        };
        
        selectLabel.textContent = labels[this.currentCamera] || '前置';
        
        // 更新按钮样式
        selectBtn.classList.remove('camera-front', 'camera-back');
        selectBtn.classList.add(`camera-${this.currentCamera}`);
    }
    
    toggleCameraView() {
        // 循环切换: RGB -> Depth -> Overlay -> RGB
        if (this.currentViewMode === 'rgb') {
            this.currentViewMode = 'depth';
        } else if (this.currentViewMode === 'depth') {
            this.currentViewMode = 'overlay';
        } else {
            this.currentViewMode = 'rgb';
        }
        
        this.updateCameraDisplay();
        this.updateToggleButton();
    }
    
    updateToggleButton() {
        const toggleBtn = document.getElementById('cameraToggleBtn');
        const toggleLabel = document.getElementById('cameraToggleLabel');
        
        if (!toggleBtn || !toggleLabel) return;
        
        const labels = {
            'rgb': 'RGB',
            'depth': '深度',
            'overlay': '叠加'
        };
        
        toggleLabel.textContent = labels[this.currentViewMode] || 'RGB';
        
        // 更新按钮样式
        toggleBtn.classList.remove('mode-rgb', 'mode-depth', 'mode-overlay');
        toggleBtn.classList.add(`mode-${this.currentViewMode}`);
    }
    
    updateCameraDisplay() {
        const cameraImage = document.getElementById('cameraImage');
        const depthImage = document.getElementById('depthImage');
        
        if (!cameraImage || !depthImage) return;
        
        // 重置显示状态
        cameraImage.style.display = 'none';
        depthImage.style.display = 'none';
        depthImage.style.opacity = '1';
        depthImage.style.position = '';
        depthImage.classList.remove('overlay-active', 'depth-only');
        
        // 获取当前选择摄像头的图像数据
        const currentRgb = this.rgbImageData[this.currentCamera];
        const currentDepth = this.depthImageData[this.currentCamera];
        
        switch (this.currentViewMode) {
            case 'rgb':
                if (currentRgb) {
                    cameraImage.src = currentRgb;
                    cameraImage.style.display = 'block';
                }
                break;
            case 'depth':
                if (currentDepth) {
                    depthImage.src = currentDepth;
                    depthImage.style.display = 'block';
                    depthImage.style.position = 'static'; // Use static positioning for depth-only mode
                    depthImage.style.opacity = '1';
                    depthImage.classList.add('depth-only');
                }
                break;
            case 'overlay':
                if (currentRgb) {
                    cameraImage.src = currentRgb;
                    cameraImage.style.display = 'block';
                }
                if (currentDepth) {
                    depthImage.src = currentDepth;
                    depthImage.style.display = 'block';
                    depthImage.style.position = 'absolute'; // Use absolute positioning for overlay
                    depthImage.style.opacity = '0.5';
                    depthImage.classList.add('overlay-active');
                }
                break;
        }
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
    
    setupUltrasonicDisplay() {
        if (!this.ros2Bridge) {
            console.warn('⚠️ ROS2桥接未初始化');
            return;
        }
        
        // 等待连接建立后再订阅
        if (!this.ros2Bridge.connected) {
            setTimeout(() => this.setupUltrasonicDisplay(), 1000);
            return;
        }
        
        // 初始化传感器显示网格
        this.initUltrasonicGrid();
        
        // 订阅超声波传感器数据
        this.ros2Bridge.subscribe('/ultrasonic/all_sensors', (message) => {
            this.updateUltrasonicSensors(message);
        });
    }
    
    initUltrasonicGrid() {
        const grid = document.getElementById('ultrasonicSensorsGrid');
        if (!grid) return;
        
        // 清空现有内容
        grid.innerHTML = '';
        
        // 创建8个传感器显示项（编号从1开始）
        for (let i = 0; i < 8; i++) {
            const displayIndex = i + 1; // 显示编号从1开始
            const sensorItem = document.createElement('div');
            sensorItem.className = 'ultrasonic-sensor-item';
            sensorItem.id = `ultrasonicSensor${i}`;
            sensorItem.innerHTML = `
                <div class="sensor-label">传感器 ${displayIndex}</div>
                <div class="sensor-value" id="sensorValue${i}">--</div>
                <div class="sensor-unit">m</div>
            `;
            grid.appendChild(sensorItem);
        }
    }
    
    updateUltrasonicSensors(message) {
        try {
            const sensors = message.data || [];
            
            // 更新每个传感器的显示
            for (let i = 0; i < 8; i++) {
                const valueElement = document.getElementById(`sensorValue${i}`);
                const sensorItem = document.getElementById(`ultrasonicSensor${i}`);
                
                if (!valueElement || !sensorItem) continue;
                
                if (i < sensors.length && sensors[i] >= 0) {
                    // 有效数据：显示距离值（米）
                    const distance = sensors[i].toFixed(2);
                    valueElement.textContent = distance;
                    
                    // 根据距离设置颜色和状态
                    sensorItem.classList.remove('sensor-invalid', 'sensor-near', 'sensor-mid', 'sensor-far');
                    
                    if (sensors[i] < 0.3) {
                        sensorItem.classList.add('sensor-near'); // 红色：< 0.3m
                    } else if (sensors[i] < 1.0) {
                        sensorItem.classList.add('sensor-mid'); // 黄色：0.3-1m
                    } else {
                        sensorItem.classList.add('sensor-far'); // 蓝色：> 1m
                    }
                } else {
                    // 无效数据
                    valueElement.textContent = '--';
                    sensorItem.classList.remove('sensor-near', 'sensor-mid', 'sensor-far');
                    sensorItem.classList.add('sensor-invalid');
                }
            }
        } catch (error) {
            console.error('🚨 更新超声波传感器显示时出错:', error);
        }
    }
    
    
    setupWaypointSystem() {
        if (!this.waypointSystem) {
            console.warn('⚠️ Waypoint系统模块未找到');
            return;
        }
        
        console.log('🎯 Waypoint系统模块已连接');
    }
    
    updateCameraImage(message, camera) {
        try {
            // 将ROS图像消息转换为可显示的图像
            const imageData = this.convertImageMessage(message);
            
            if (imageData) {
                // 存储对应摄像头的图像数据
                this.rgbImageData[camera] = imageData;
                
                // 如果这是当前选择的摄像头，更新显示
                if (camera === this.currentCamera) {
                    if (this.currentViewMode === 'rgb' || this.currentViewMode === 'overlay') {
                        const cameraImage = document.getElementById('cameraImage');
                        if (cameraImage) {
                            cameraImage.src = imageData;
                            if (this.currentViewMode === 'rgb') {
                                cameraImage.style.display = 'block';
                            }
                        }
                    }
                }
                
                this.updateVideoInfo(message);
            }
        } catch (error) {
            console.error('🚨 更新摄像头图像时出错:', error);
        }
    }
    
    updateDepthImage(message, camera) {
        try {
            // 将ROS图像消息转换为可显示的图像
            const imageData = this.convertImageMessage(message);
            
            if (imageData) {
                // 存储对应摄像头的深度图像数据
                this.depthImageData[camera] = imageData;
                
                // 如果这是当前选择的摄像头，更新显示
                if (camera === this.currentCamera) {
                    if (this.currentViewMode === 'depth' || this.currentViewMode === 'overlay') {
                        const depthImage = document.getElementById('depthImage');
                        const cameraImage = document.getElementById('cameraImage');
                        
                        if (depthImage) {
                            depthImage.src = imageData;
                            
                            if (this.currentViewMode === 'depth') {
                                // Depth-only mode: hide RGB, show depth with static positioning
                                if (cameraImage) {
                                    cameraImage.style.display = 'none';
                                }
                                depthImage.style.display = 'block';
                                depthImage.style.position = 'static';
                                depthImage.style.opacity = '1';
                                depthImage.classList.remove('overlay-active');
                                depthImage.classList.add('depth-only');
                            } else if (this.currentViewMode === 'overlay') {
                                // Overlay mode: show both with absolute positioning
                                depthImage.style.display = 'block';
                                depthImage.style.position = 'absolute';
                                depthImage.style.opacity = '0.5';
                                depthImage.classList.add('overlay-active');
                                depthImage.classList.remove('depth-only');
                            }
                        }
                    }
                }
            }
        } catch (error) {
            console.error('🚨 更新深度图像时出错:', error);
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
            let location = '未定位';
            let lastDetectionTime = 0;
            
            // 尝试解析JSON格式的消息
            try {
                // 优先使用直接从WebSocket接收的字段
                if (message.last_detection_time !== undefined) {
                    location = message.location || message.data || '未定位';
                    lastDetectionTime = message.last_detection_time;
                    // 更新最后检测时间
                    if (lastDetectionTime > 0) {
                        this.lastApriltagDetectionTime = lastDetectionTime;
                    }
                } else {
                    // 尝试从data字段解析JSON
                    const messageData = message.data || message.location || '未定位';
                    if (typeof messageData === 'string') {
                        // 尝试解析JSON
                        try {
                            const locationData = JSON.parse(messageData);
                            location = locationData.location || '未定位';
                            lastDetectionTime = locationData.last_detection_time || 0;
                            // 更新最后检测时间
                            if (lastDetectionTime > 0) {
                                this.lastApriltagDetectionTime = lastDetectionTime;
                            }
                        } catch (e) {
                            // 如果不是JSON，直接使用字符串
                            location = messageData;
                        }
                    } else {
                        location = messageData;
                    }
                }
            } catch (e) {
                console.warn('⚠️ 解析定位消息失败:', e);
                location = message.data || message.location || '未定位';
            }
            
            console.log('📍 设置定位为:', location, '最后检测时间:', lastDetectionTime);
            locationElement.textContent = location;
            
            // 根据定位类型设置基础样式
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
            
            // 设置基础类，然后通过定时器更新颜色状态
            locationElement.className = locationClass;
            
            // 立即更新一次颜色状态
            this.updateLocationStatusColor();
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
                    
                    // 如果是路线规划标签页，确保RoutePlanner已初始化
                    if (targetTab === 'route-planner' && window.routePlanner) {
                        if (!window.routePlanner.initialized) {
                            window.routePlanner.initialize();
                        } else {
                            // 重新设置画布尺寸（可能在切换标签时窗口大小改变了）
                            window.routePlanner.setupCanvas();
                        }
                    }
                    
                    // 如果是机器人控制页面（包含路径导航），刷新路线规划节点配置
                    if (targetTab === 'control' && window.waypointSystem) {
                        console.log('🔄 切换到机器人控制页面，刷新路线规划节点配置...');
                        // 刷新终点节点列表
                        if (window.waypointSystem.updateDestinationNodeList) {
                            window.waypointSystem.updateDestinationNodeList();
                        }
                        // 同步当前位置信息
                        if (window.waypointSystem.syncLocationFromRobotStatus) {
                            window.waypointSystem.syncLocationFromRobotStatus();
                        }
                        // 更新当前位置显示
                        if (window.waypointSystem.updateCurrentLocationDisplay) {
                            window.waypointSystem.updateCurrentLocationDisplay();
                        }
                        console.log('✅ 路线规划节点配置已刷新');
                    }
                    
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
