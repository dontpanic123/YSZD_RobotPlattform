/**
 * ROS2 Web Bridge 连接管理
 * 负责与ROS2系统的WebSocket连接
 */

class ROS2Bridge {
    constructor() {
        this.ros = null;
        this.connected = false;
        this.topics = {};
        this.callbacks = {};
        
        // 连接配置
        this.config = {
            url: 'ws://127.0.0.1:9090',
            reconnectInterval: 5000,
            maxReconnectAttempts: 10,
            reconnectAttempts: 0
        };
        
        this.websocket = null;
        
        this.init();
    }
    
    init() {
        this.setupEventListeners();
        this.connect();
    }
    
    setupEventListeners() {
        // 页面可见性变化时重连
        document.addEventListener('visibilitychange', () => {
            if (!document.hidden && !this.connected) {
                this.connect();
            }
        });
        
        // 窗口关闭时断开连接
        window.addEventListener('beforeunload', () => {
            this.disconnect();
        });
    }
    
    connect() {
        try {
            console.log('🔗 正在连接到ROS2 WebSocket Bridge...');
            
            this.websocket = new WebSocket(this.config.url);
            
            // 连接成功
            this.websocket.onopen = () => {
                console.log('✅ ROS2 WebSocket Bridge 连接成功');
                this.connected = true;
                this.reconnectAttempts = 0;
                this.updateConnectionStatus(true);
                this.setupTopics();
            };
            
            // 连接关闭
            this.websocket.onclose = () => {
                console.log('❌ ROS2 WebSocket Bridge 连接关闭');
                this.connected = false;
                this.updateConnectionStatus(false);
                this.scheduleReconnect();
            };
            
            // 连接错误
            this.websocket.onerror = (error) => {
                console.error('🚨 ROS2 WebSocket Bridge 连接错误:', error);
                this.connected = false;
                this.updateConnectionStatus(false);
                this.scheduleReconnect();
            };
            
            // 接收消息
            this.websocket.onmessage = (event) => {
                this.handleMessage(event.data);
            };
            
        } catch (error) {
            console.error('🚨 创建WebSocket连接时出错:', error);
            this.scheduleReconnect();
        }
    }
    
    scheduleReconnect() {
        if (this.reconnectAttempts < this.config.maxReconnectAttempts) {
            this.reconnectAttempts++;
            console.log(`🔄 尝试重连 (${this.reconnectAttempts}/${this.config.maxReconnectAttempts})...`);
            
            setTimeout(() => {
                this.connect();
            }, this.config.reconnectInterval);
        } else {
            console.error('🚨 达到最大重连次数，停止重连');
            this.updateConnectionStatus(false, '连接失败');
        }
    }
    
    disconnect() {
        if (this.websocket) {
            this.websocket.close();
            this.websocket = null;
        }
        this.connected = false;
        this.updateConnectionStatus(false);
    }
    
    handleMessage(data) {
        try {
            // 检查数据是否为空或无效
            if (!data || typeof data !== 'string') {
                console.warn('⚠️ 收到无效的WebSocket数据:', data);
                return;
            }
            
            // 若不是JSON（首字符不是{或[），尝试作为base64图像处理
            const firstChar = data.trim().charAt(0);
            if (firstChar !== '{' && firstChar !== '[') {
                // 简单判断是否可能是base64（只包含base64字符）
                const maybeBase64 = /^[A-Za-z0-9+/=\r\n]+$/.test(data);
                if (maybeBase64) {
                    // 作为AprilTag检测图像处理（前端已支持base64 jpeg显示）
                    this.handleAprilTagDetection({
                        height: 0,
                        width: 0,
                        encoding: 'jpeg',
                        data: data.replace(/\r?\n/g, '')
                    });
                    return;
                }
            }

            // 检查数据长度，如果太长可能是二进制数据
            if (data.length > 1000000) { // 1MB
                console.warn('⚠️ 收到过大的WebSocket数据，可能是二进制数据，跳过JSON解析');
                return;
            }
            
            // 尝试解析JSON
            const message = JSON.parse(data);
            const messageType = message.type;
            
            // 处理不同类型的消息
            if (messageType === 'robot_state') {
                console.log('🤖 收到机器人状态消息:', message);
                this.handleRobotState(message);
            } else {
                switch (messageType) {
                    case 'camera_image':
                        this.handleCameraImage(message);
                        break;
                    case 'apriltag_detection':
                        this.handleAprilTagDetection(message);
                        break;
                    case 'apriltag_pose':
                        this.handleAprilTagPose(message);
                        break;
                    case 'apriltag_status':
                        this.handleAprilTagStatus(message);
                        break;
                    case 'robot_state':
                        this.handleRobotState(message);
                        break;
                    case 'odom':
                        this.handleOdometry(message);
                        break;
                    case 'waypoint_status':
                        this.handleWaypointStatus(message);
                        break;
                    case 'pong':
                        console.log('🏓 收到pong响应');
                        break;
                    default:
                        console.log('📨 收到未知消息类型:', messageType);
                }
            }
        } catch (error) {
            console.error('🚨 处理WebSocket消息时出错:', error);
            console.error('📊 错误数据:', {
                dataLength: data ? data.length : 'undefined',
                dataType: typeof data,
                dataPreview: data ? data.substring(0, 200) + (data.length > 200 ? '...' : '') : 'undefined',
                errorMessage: error.message
            });
        }
    }
    
    handleCameraImage(message) {
        // 处理摄像头图像
        if (this.callbacks['/camera/image_raw']) {
            // 创建模拟的ROS图像消息
            const rosMessage = {
                header: {
                    stamp: { sec: 0, nanosec: 0 },
                    frame_id: 'camera_link'
                },
                height: message.height,
                width: message.width,
                encoding: message.encoding,
                data: message.data
            };
            this.callbacks['/camera/image_raw'](rosMessage);
        }
    }
    
    handleAprilTagDetection(message) {
        // 处理AprilTag检测图像
        if (this.callbacks['/apriltag_detection']) {
            const rosMessage = {
                header: {
                    stamp: { sec: 0, nanosec: 0 },
                    frame_id: 'camera_link'
                },
                height: message.height,
                width: message.width,
                encoding: message.encoding,
                data: message.data
            };
            this.callbacks['/apriltag_detection'](rosMessage);
        }
    }
    
    handleAprilTagPose(message) {
        // 处理AprilTag位姿
        if (this.callbacks['/apriltag_pose']) {
            const rosMessage = {
                header: {
                    stamp: { sec: 0, nanosec: 0 },
                    frame_id: typeof message.tag_id === 'number' ? `apriltag_${message.tag_id}` : (message.tag_id || 'camera_link')
                },
                pose: {
                    position: message.position,
                    orientation: message.orientation
                }
            };
            this.callbacks['/apriltag_pose'](rosMessage);
        }
    }
    
    handleAprilTagStatus(message) {
        // 处理AprilTag状态
        if (this.callbacks['/apriltag_status']) {
            const rosMessage = {
                data: message.status
            };
            this.callbacks['/apriltag_status'](rosMessage);
        }
    }
    
    handleOdometry(message) {
        // 处理里程计消息
        if (this.callbacks['/odom']) {
            // 创建ROS里程计消息格式
            const rosMessage = {
                header: {
                    stamp: message.header?.stamp || { sec: 0, nanosec: 0 },
                    frame_id: message.header?.frame_id || 'odom',
                    seq: message.header?.seq || 0
                },
                child_frame_id: message.child_frame_id || 'base_link',
                pose: {
                    pose: {
                        position: message.pose?.pose?.position || { x: 0, y: 0, z: 0 },
                        orientation: message.pose?.pose?.orientation || { x: 0, y: 0, z: 0, w: 1 }
                    },
                    covariance: message.pose?.covariance || new Array(36).fill(0)
                },
                twist: {
                    twist: {
                        linear: message.twist?.twist?.linear || { x: 0, y: 0, z: 0 },
                        angular: message.twist?.twist?.angular || { x: 0, y: 0, z: 0 }
                    },
                    covariance: message.twist?.covariance || new Array(36).fill(0)
                }
            };
            this.callbacks['/odom'](rosMessage);
        }
    }
    
    handleRobotState(message) {
        // 处理机器人状态消息
        if (this.callbacks['/robot_state']) {
            const rosMessage = {
                data: message.state || message.data || 'unknown'
            };
            this.callbacks['/robot_state'](rosMessage);
        }
    }
    
    handleWaypointStatus(message) {
        // 处理Waypoint状态消息
        console.log('🎯 收到Waypoint状态消息:', message.status);
        // 触发自定义事件，让waypoint-system.js监听
        const event = new CustomEvent('waypoint_status', {
            detail: {
                status: message.status
            }
        });
        document.dispatchEvent(event);
        
        // 如果存在回调，也调用它
        if (this.callbacks['/waypoint_following_status']) {
            const rosMessage = {
                data: message.status || 'unknown'
            };
            this.callbacks['/waypoint_following_status'](rosMessage);
        }
    }
    
    updateConnectionStatus(connected, message = '') {
        const statusElement = document.getElementById('connectionStatus');
        if (statusElement) {
            if (connected) {
                statusElement.className = 'status-indicator connected';
                statusElement.innerHTML = '<i class="fas fa-circle"></i> 已连接';
            } else {
                statusElement.className = 'status-indicator disconnected';
                statusElement.innerHTML = `<i class="fas fa-circle"></i> ${message || '未连接'}`;
            }
        }
    }
    
    setupTopics() {
        if (!this.connected) return;
        
        console.log('📡 设置ROS2话题...');
        
        // 机器人控制话题
        this.setupTopic('/cmd_vel', 'geometry_msgs/msg/Twist');
        
        // 摄像头话题
        this.setupTopic('/camera/image_raw', 'sensor_msgs/msg/Image');
        this.setupTopic('/apriltag_detection', 'sensor_msgs/msg/Image');
        
        // AprilTag检测话题
        this.setupTopic('/apriltag_pose', 'geometry_msgs/msg/PoseStamped');
        this.setupTopic('/apriltag_status', 'std_msgs/msg/String');
        
        // 机器人状态话题
        this.setupTopic('/odom', 'nav_msgs/msg/Odometry');
        this.setupTopic('/robot_description', 'std_msgs/msg/String');
        this.setupTopic('/robot_state', 'std_msgs/msg/String');
        
        // 导航话题
        this.setupTopic('/goal_pose', 'geometry_msgs/msg/PoseStamped');
        this.setupTopic('/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped');
        
        console.log('✅ ROS2话题设置完成');
    }
    
    setupTopic(topicName, messageType) {
        if (!this.connected) return;
        
        try {
            const topic = new ROSLIB.Topic({
                ros: this.ros,
                name: topicName,
                messageType: messageType
            });
            
            this.topics[topicName] = topic;
            console.log(`📡 话题 ${topicName} 已设置`);
            
        } catch (error) {
            console.error(`🚨 设置话题 ${topicName} 时出错:`, error);
        }
    }
    
    publish(topicName, message) {
        if (!this.connected || !this.websocket) {
            console.warn(`⚠️ WebSocket未连接`);
            return false;
        }
        
        try {
            let wsMessage;
            
            if (topicName === '/cmd_vel') {
                wsMessage = {
                    type: 'cmd_vel',
                    linear_x: message.linear.x,
                    linear_y: message.linear.y,
                    linear_z: message.linear.z,
                    angular_x: message.angular.x,
                    angular_y: message.angular.y,
                    angular_z: message.angular.z
                };
            } else if (topicName === '/goal_pose') {
                wsMessage = {
                    type: 'goal_pose',
                    x: message.pose.position.x,
                    y: message.pose.position.y,
                    z: message.pose.position.z
                };
            } else if (topicName === '/emergency_stop') {
                // 正确处理active字段，当active为false时不能使用||运算，因为它会被当作falsy值
                const active = message.active !== undefined ? message.active : 
                               (message.data !== undefined ? message.data : true);
                wsMessage = {
                    type: 'emergency_stop',
                    active: active
                };
            } else {
                console.warn(`⚠️ 不支持的话题: ${topicName}`);
                return false;
            }
            
            this.websocket.send(JSON.stringify(wsMessage));
            return true;
        } catch (error) {
            console.error(`🚨 发布消息到话题 ${topicName} 时出错:`, error);
            return false;
        }
    }
    
    subscribe(topicName, callback) {
        // 允许在未连接时也登记回调，连接建立后消息到达仍可回调
        try {
            this.callbacks[topicName] = callback;
            if (!this.connected) {
                console.warn(`⚠️ WebSocket未连接，已预登记订阅: ${topicName}`);
            } else {
                console.log(`👂 已订阅话题 ${topicName}`);
            }
            return true;
        } catch (error) {
            console.error(`🚨 订阅话题 ${topicName} 时出错:`, error);
            return false;
        }
    }
    
    unsubscribe(topicName) {
        if (!this.connected || !this.topics[topicName]) {
            return false;
        }
        
        try {
            this.topics[topicName].unsubscribe();
            delete this.callbacks[topicName];
            console.log(`🔇 已取消订阅话题 ${topicName}`);
            return true;
        } catch (error) {
            console.error(`🚨 取消订阅话题 ${topicName} 时出错:`, error);
            return false;
        }
    }
    
    // 获取话题信息
    getTopics() {
        return Object.keys(this.topics);
    }
    
    // 检查连接状态
    isConnected() {
        return this.connected;
    }
    
    // 获取连接质量
    getConnectionQuality() {
        if (!this.connected) return '断开';
        
        // 简单的连接质量评估
        const topicCount = Object.keys(this.topics).length;
        if (topicCount > 5) return '优秀';
        if (topicCount > 3) return '良好';
        if (topicCount > 1) return '一般';
        return '较差';
    }
}

// 创建全局ROS2 Bridge实例
window.ros2Bridge = new ROS2Bridge();
