/**
 * 路径导航系统 - 重写版本
 * 确保所有按钮功能正常工作
 */
class WaypointSystem {
    constructor() {
        console.log('🎯 初始化Waypoint系统...');
        
        // 状态管理
        this.isRecording = false;
        this.isFollowing = false;
        this.isPaused = false;  // 是否暂停跟踪
        this.pausedWaypointIndex = 0;  // 暂停时的waypoint索引
        this.waypoints = [];
        this.currentPathName = '';
        this.selectedWaypointFile = '';
        
        // APRILTAG定位管理
        this.currentApriltagIds = [];  // 存储所有当前检测到的APRILTAG ID列表
        this.selectedApriltagId = null;  // 用户选择的APRILTAG ID（当检测到多个时）
        this.currentLocationNode = null;  // 当前选择的起点节点
        this.selectedDestinationNodeId = null;  // 选择的终点节点ID
        
        // 录制时间管理
        this.recordingStartTime = null;
        this.recordingTimer = null;
        
        // 依赖检查
        this.ros2Bridge = null;
        this.robotController = null;
        
        // 初始化标志
        this.initialized = false;
        
        // 绑定方法
        this.handleButtonDown = this.handleButtonDown.bind(this);
        this.handleButtonUp = this.handleButtonUp.bind(this);
    }
    
    /**
     * 初始化系统
     */
    async initialize() {
        console.log('🔧 开始初始化Waypoint系统...');
        
        try {
            // 等待DOM加载完成
        if (document.readyState === 'loading') {
                await new Promise(resolve => {
                    document.addEventListener('DOMContentLoaded', resolve);
                });
            }
            
            // 等待依赖模块加载
            await this.waitForDependencies();
            
            // 设置事件监听器
            this.setupEventListeners();
            
            // 初始化UI状态
            this.initializeUI();
            
            // 加载数据
            await this.loadInitialData();
            
            this.initialized = true;
            console.log('✅ Waypoint系统初始化完成');
            
        } catch (error) {
            console.error('❌ Waypoint系统初始化失败:', error);
            this.showNotification('Waypoint系统初始化失败', 'error');
        }
    }
    
    /**
     * 等待依赖模块加载
     */
    async waitForDependencies() {
        console.log('⏳ 等待依赖模块加载...');
        
        let attempts = 0;
        const maxAttempts = 50; // 5秒超时
        
        while (attempts < maxAttempts) {
            this.ros2Bridge = window.ros2Bridge;
            this.robotController = window.robotController;
            
            if (this.ros2Bridge && this.robotController) {
                console.log('✅ 依赖模块已加载');
                return;
            }
            
            await new Promise(resolve => setTimeout(resolve, 100));
            attempts++;
        }
        
        throw new Error('依赖模块加载超时');
    }
    
    /**
     * 设置事件监听器
     */
    setupEventListeners() {
        console.log('🔧 设置事件监听器...');
        
        // 录制控制按钮
        this.setupButton('startWaypointRecordingBtn', () => this.startRecording());
        this.setupButton('stopWaypointRecordingBtn', () => this.stopRecording());
        this.setupButton('saveWaypointsBtn', () => this.saveWaypoints());
        
        // 跟踪控制按钮
        this.setupButton('startWaypointFollowingBtn', () => this.startFollowing());
        this.setupButton('pauseWaypointFollowingBtn', () => this.togglePause());
        this.setupButton('stopWaypointFollowingBtn', () => this.stopFollowing());
        
        // 路径管理按钮（已废弃，不再需要刷新文件按钮）
        
        // 输入框事件
        this.setupInputEvents();
        
        // 文件选择器事件
        this.setupFileSelector();
        
        // 机器人控制按钮
        this.setupRobotControlButtons();
        
        // Waypoint状态监听
        this.setupWaypointStatusListener();
        
        // 紧急停止状态监听
        this.setupEmergencyStopListener();
        
        // APRILTAG定位监听
        this.setupApriltagListener();
        
        // 路线规划更新监听
        this.setupRoutePlannerListener();
        
        console.log('✅ 事件监听器设置完成');
    }
    
    /**
     * 设置位置监听（从/robot_location话题获取位置信息）
     */
    setupApriltagListener() {
        if (!this.ros2Bridge) {
            console.warn('⚠️ ROS2桥接未初始化，无法订阅位置话题');
            return;
        }
        
        // 等待连接建立后再订阅
        if (!this.ros2Bridge.connected) {
            setTimeout(() => this.setupApriltagListener(), 1000);
            return;
        }
        
        // 订阅机器人定位话题（复用机器人状态中的数据流）
        try {
            this.ros2Bridge.subscribe('/robot_location', (message) => {
                this.handleRobotLocation(message);
            });
            console.log('✅ 位置监听器已设置，已订阅 /robot_location 话题');
        } catch (error) {
            console.error('❌ 订阅位置话题失败:', error);
        }
    }
    
    /**
     * 处理机器人定位消息（从/robot_location话题）
     */
    handleRobotLocation(message) {
        try {
            console.log('📨 收到机器人定位消息:', message);
            
            // 从消息中提取位置信息
            const location = message.location || message.data || '未定位';
            
            // 如果位置是"未定位"，清空当前选择
            if (location === '未定位' || !location || location.trim() === '') {
                this.selectedApriltagId = null;
                this.currentLocationNode = null;
                this.updateCurrentLocationDisplay();
                this.updateDestinationNodeList();
                return;
            }
            
            // 完全从路线规划节点图中查找对应关系
            // 先尝试根据位置名称查找节点
            let node = this.findNodeByLocationName(location);
            
            // 如果位置名称是"未知位置(ID:xxx)"格式，尝试从ID查找
            if (!node && location.startsWith('未知位置(ID:')) {
                const match = location.match(/未知位置\(ID:(\d+)\)/);
                if (match) {
                    const tagId = parseInt(match[1]);
                    if (!isNaN(tagId)) {
                        // 根据APRILTAG ID查找节点
                        const nodes = this.getNodesByApriltagId(tagId);
                        if (nodes.length > 0) {
                            node = nodes[0];
                            this.selectedApriltagId = tagId;
                        }
                    }
                }
            }
            
            if (node) {
                // 找到了对应的节点
                this.currentLocationNode = node;
                
                // 从节点的markerId获取APRILTAG ID
                if (node.node.markerId !== null && node.node.markerId !== undefined) {
                    this.selectedApriltagId = node.node.markerId;
                    console.log(`🎯 从路线规划节点"${node.node.name}"获取APRILTAG ID:`, this.selectedApriltagId);
                } else {
                    // 如果节点没有配置markerId，清空selectedApriltagId
                    this.selectedApriltagId = null;
                    console.warn(`⚠️ 节点"${node.node.name}"未配置APRILTAG ID`);
                }
                
                // 更新UI
                this.updateCurrentLocationDisplay();
                this.updateDestinationNodeList();
            } else {
                // 没找到对应节点
                console.warn(`⚠️ 在路线规划中未找到位置"${location}"对应的节点`);
                this.currentLocationNode = null;
                this.selectedApriltagId = null;
                this.updateCurrentLocationDisplay();
                this.updateDestinationNodeList();
            }
        } catch (error) {
            console.error('❌ 处理机器人定位时出错:', error, message);
        }
    }
    
    /**
     * 根据位置名称查找节点（不依赖APRILTAG ID）
     */
    findNodeByLocationName(locationName) {
        if (!window.routePlanner || !window.routePlanner.nodes) {
            return null;
        }
        
        // 遍历所有节点，查找名称匹配的节点
        for (const [nodeId, node] of window.routePlanner.nodes) {
            if (node.name === locationName) {
                return { id: nodeId, node: node };
            }
        }
        
        return null;
    }
    
    /**
     * 定期清理过期的APRILTAG ID（如果长时间未检测到，从列表中移除）
     * 注意：这个方法需要在外部定期调用，或者通过定时器实现
     */
    cleanupStaleApriltagIds() {
        // 这里可以添加逻辑来清理长时间未检测到的APRILTAG ID
        // 目前暂时保留所有检测到的ID，直到用户手动选择或重新检测
    }
    
    /**
     * 监听路线规划页面的更新（当节点或连接发生变化时）
     */
    setupRoutePlannerListener() {
        // 使用定时器定期检查路线规划是否更新，并同步机器人状态中的位置信息
        setInterval(() => {
            if (window.routePlanner && window.routePlanner.nodes) {
                // 更新终点节点列表
                this.updateDestinationNodeList();
                
                // 同步机器人状态中的位置信息
                this.syncLocationFromRobotStatus();
            }
        }, 1000); // 每1秒检查一次
    }
    
    /**
     * 从机器人状态中同步位置信息
     */
    syncLocationFromRobotStatus() {
        const robotLocationElement = document.getElementById('robotCurrentLocation');
        if (!robotLocationElement) {
            return;
        }
        
        const currentLocation = robotLocationElement.textContent;
        
        // 如果位置发生变化，更新路径导航中的位置
        if (currentLocation && currentLocation !== '未定位') {
            // 完全从路线规划节点图中查找
            let node = this.findNodeByLocationName(currentLocation);
            
            // 如果位置名称是"未知位置(ID:xxx)"格式，尝试从ID查找
            if (!node && currentLocation.startsWith('未知位置(ID:')) {
                const match = currentLocation.match(/未知位置\(ID:(\d+)\)/);
                if (match) {
                    const tagId = parseInt(match[1]);
                    if (!isNaN(tagId)) {
                        const nodes = this.getNodesByApriltagId(tagId);
                        if (nodes.length > 0) {
                            node = nodes[0];
                        }
                    }
                }
            }
            
            // 检查节点是否发生变化
            const nodeChanged = node !== this.currentLocationNode;
            const nodeIdChanged = node && node.node.markerId !== this.selectedApriltagId;
            
            if (nodeChanged || nodeIdChanged) {
                this.currentLocationNode = node;
                
                // 从节点的markerId获取APRILTAG ID
                if (node && node.node.markerId !== null && node.node.markerId !== undefined) {
                    this.selectedApriltagId = node.node.markerId;
                } else {
                    this.selectedApriltagId = null;
                }
                
                this.updateCurrentLocationDisplay();
                this.updateDestinationNodeList();
            }
        } else if (this.selectedApriltagId !== null || this.currentLocationNode !== null) {
            // 如果机器人状态显示"未定位"，清空路径导航中的位置
            this.selectedApriltagId = null;
            this.currentLocationNode = null;
            this.updateCurrentLocationDisplay();
            this.updateDestinationNodeList();
        }
    }
    
    /**
     * 处理多个APRILTAG的情况
     */
    handleMultipleApriltags(apriltagIds) {
        // 创建选择对话框
        const message = `检测到多个APRILTAG，请选择当前位置:\n${apriltagIds.map(id => `  - APRILTAG ID: ${id}`).join('\n')}`;
        
        // 使用简单的prompt让用户选择（可以后续改进为更友好的UI）
        const userInput = prompt(message + '\n\n请输入要选择的APRILTAG ID:');
        if (userInput !== null) {
            const selectedId = parseInt(userInput);
            if (!isNaN(selectedId) && apriltagIds.includes(selectedId)) {
                this.selectedApriltagId = selectedId;
                this.updateCurrentLocationNode();
                this.updateCurrentLocationDisplay();
                this.updateDestinationNodeList();
            } else {
                this.showNotification('无效的APRILTAG ID', 'warning');
            }
        }
    }
    
    /**
     * 根据APRILTAG ID查找匹配的节点
     */
    getNodesByApriltagId(apriltagId) {
        if (!window.routePlanner || !window.routePlanner.nodes) {
            return [];
        }
        
        const matchingNodes = [];
        window.routePlanner.nodes.forEach((node, nodeId) => {
            if (node.markerId !== null && node.markerId !== undefined && node.markerId === apriltagId) {
                matchingNodes.push({ id: nodeId, node: node });
            }
        });
        
        return matchingNodes;
    }
    
    /**
     * 更新当前定位节点
     */
    updateCurrentLocationNode() {
        if (!this.selectedApriltagId) {
            this.currentLocationNode = null;
            return;
        }
        
        const matchingNodes = this.getNodesByApriltagId(this.selectedApriltagId);
        if (matchingNodes.length > 0) {
            // 如果找到多个匹配的节点，使用第一个
            this.currentLocationNode = matchingNodes[0];
        } else {
            this.currentLocationNode = null;
        }
    }
    
    /**
     * 获取可用的终点节点列表
     */
    getAvailableDestinationNodes() {
        if (!window.routePlanner || !window.routePlanner.nodes) {
            return [];
        }
        
        const availableNodes = [];
        const currentNodeId = this.currentLocationNode ? this.currentLocationNode.id : null;
        
        window.routePlanner.nodes.forEach((node, nodeId) => {
            // 排除当前节点（起点）
            if (nodeId !== currentNodeId) {
                availableNodes.push({
                    id: nodeId,
                    name: node.name,
                    type: node.type,
                    markerId: node.markerId
                });
            }
        });
        
        return availableNodes;
    }
    
    /**
     * 查找两个节点之间的连接
     */
    findConnectionBetweenNodes(fromNodeId, toNodeId) {
        if (!window.routePlanner || !window.routePlanner.connections) {
            return null;
        }
        
        // 遍历所有连接，查找匹配的连接
        for (const [connId, connection] of window.routePlanner.connections) {
            if (connection.fromNodeId === fromNodeId && connection.toNodeId === toNodeId) {
                return {
                    id: connId,
                    connection: connection,
                    waypointsFile: connection.waypointsFile
                };
            }
        }
        
        return null;
    }
    
    /**
     * 设置单个按钮
     */
    setupButton(buttonId, handler) {
        const button = document.getElementById(buttonId);
        if (button) {
            button.addEventListener('click', handler);
            console.log(`✅ 按钮 ${buttonId} 事件监听器已设置`);
        } else {
            console.warn(`⚠️ 按钮 ${buttonId} 未找到`);
        }
    }
    
    /**
     * 设置输入框事件
     */
    setupInputEvents() {
        const pathNameInput = document.getElementById('waypointPathName');
        if (pathNameInput) {
            pathNameInput.addEventListener('input', (e) => {
                this.currentPathName = e.target.value;
                this.updateUI();
            });
        }
    }
    
    /**
     * 设置文件选择器（已废弃，改为终点节点选择器）
     */
    setupFileSelector() {
        // 改为设置终点节点选择器
        const destinationSelect = document.getElementById('destinationNodeSelect');
        if (destinationSelect) {
            destinationSelect.addEventListener('change', (e) => {
                this.selectedDestinationNodeId = e.target.value;
                this.updateFollowingButtonStates();
            });
        }
    }
    
    /**
     * 设置机器人控制按钮
     */
    setupRobotControlButtons() {
        const controlButtons = document.querySelectorAll('.waypoint-robot-controls .control-btn');
        if (controlButtons.length > 0) {
            controlButtons.forEach(btn => {
                btn.addEventListener('mousedown', this.handleButtonDown);
                btn.addEventListener('mouseup', this.handleButtonUp);
                btn.addEventListener('mouseleave', this.handleButtonUp);
            });
            console.log(`✅ 设置了 ${controlButtons.length} 个机器人控制按钮`);
        }
    }
    
    /**
     * 设置Waypoint状态监听
     */
    setupWaypointStatusListener() {
        document.addEventListener('waypoint_status', (event) => {
            const status = event.detail.status;
            console.log('📨 收到Waypoint状态更新:', status);
            
            if (status === 'completed') {
                // Waypoint跟踪完成，切换到空闲状态
                this.onWaypointCompleted();
            } else if (status === 'stopped') {
                // Waypoint跟踪停止，切换到空闲状态
                this.onWaypointStopped();
            }
        });
        console.log('✅ Waypoint状态监听器已设置');
    }
    
    /**
     * 设置紧急停止状态监听
     */
    setupEmergencyStopListener() {
        // 使用定时器定期检查紧急停止状态
        setInterval(() => {
            this.checkEmergencyStopState();
        }, 200); // 每200ms检查一次
        console.log('✅ 紧急停止状态监听器已设置');
    }
    
    /**
     * 检查紧急停止状态并更新UI
     */
    checkEmergencyStopState() {
        const isEmergencyStop = this.isEmergencyStopActive();
        if (isEmergencyStop && this.isFollowing) {
            // 如果紧急停止激活且正在跟踪，停止跟踪
            console.log('🚨 检测到紧急停止，停止跟踪');
            this.stopFollowing();
        }
        // 更新UI以反映紧急停止状态
        this.updateFollowingUI();
        this.updateFollowingButtonStates();
    }
    
    /**
     * 检查是否处于紧急停止状态
     */
    isEmergencyStopActive() {
        if (window.robotController) {
            return window.robotController.emergencyStopActive === true;
        }
        return false;
    }
    
    /**
     * 检查AprilTag是否被检测到
     */
    isAprilTagDetected() {
        const statusElement = document.getElementById('apriltagStatus');
        if (statusElement) {
            const statusText = statusElement.textContent || '';
            // 检查状态文本：包含"检测到"但不包含"未检测到"
            // 状态格式：检测到 X 个AprilTag: [ids] 或 未检测到AprilTag
            if (statusText.includes('未检测到')) {
                return false;
            }
            return statusText.includes('检测到');
        }
        return false;
    }
    
    /**
     * 初始化UI状态
     */
    initializeUI() {
        console.log('🎨 初始化UI状态...');
        
        // 更新录制UI
        this.updateRecordingUI();
        
        // 更新跟踪UI
        this.updateFollowingUI();
        
        // 更新按钮状态
        this.updateFollowingButtonStates();
        
        console.log('✅ UI状态初始化完成');
    }
    
    /**
     * 加载初始数据
     */
    async loadInitialData() {
        console.log('📂 加载初始数据...');
        
        try {
            // 加载保存的路径
            await this.loadSavedPaths();
            
            // 不再加载waypoint文件列表，改为从路线规划节点获取
            // 更新终点节点列表
            this.updateDestinationNodeList();
            
            // 更新当前位置显示
            this.updateCurrentLocationDisplay();
            
            console.log('✅ 初始数据加载完成');
        } catch (error) {
            console.error('❌ 加载初始数据失败:', error);
        }
    }
    
    /**
     * 更新当前位置显示
     */
    updateCurrentLocationDisplay() {
        const locationText = document.getElementById('currentLocationText');
        const locationInfo = document.getElementById('currentLocationInfo');
        
        if (!locationText || !locationInfo) {
            return;
        }
        
        // 从机器人状态中获取当前位置（如果可用）
        const robotLocationElement = document.getElementById('robotCurrentLocation');
        const currentLocation = robotLocationElement ? robotLocationElement.textContent : null;
        
        // 如果机器人状态显示"未定位"，显示未检测
        if (!currentLocation || currentLocation === '未定位') {
            locationText.textContent = '未检测到APRILTAG';
            locationInfo.className = 'location-info location-unknown';
            return;
        }
        
        // 如果找到了对应的节点，显示详细信息
        if (this.currentLocationNode) {
            const node = this.currentLocationNode.node;
            const nodeTypeNames = {
                'charging': '充电桩',
                'loading': '装载点',
                'delivery': '送达点',
                'custom': '自定义'
            };
            const typeName = nodeTypeNames[node.type] || '未知';
            
            if (this.selectedApriltagId !== null) {
                locationText.textContent = `${node.name} (${typeName}) - APRILTAG ID: ${this.selectedApriltagId}`;
            } else {
                locationText.textContent = `${node.name} (${typeName})`;
            }
            locationInfo.className = 'location-info location-configured';
        } else {
            // 如果没找到节点，但机器人状态有位置信息，显示提示信息
            // 提示用户需要在路线规划页面创建对应的节点
            locationText.textContent = `${currentLocation} (未在路线规划中配置)`;
            locationInfo.className = 'location-info location-unconfigured';
            locationInfo.title = `当前位置"${currentLocation}"已被识别，但路线规划中未找到对应节点。\n\n解决方法：\n1. 切换到"路线规划"标签页\n2. 创建名称为"${currentLocation}"的节点\n3. 在节点属性中设置对应的APRILTAG ID（当前检测到的ID）\n4. 保存路线配置`;
        }
    }
    
    /**
     * 更新终点节点列表
     */
    updateDestinationNodeList() {
        const destinationSelect = document.getElementById('destinationNodeSelect');
        if (!destinationSelect) {
            return;
        }
        
        // 清空现有选项
        destinationSelect.innerHTML = '<option value="">请选择终点...</option>';
        
        // 检查路线规划是否可用
        if (!window.routePlanner || !window.routePlanner.nodes) {
            const option = document.createElement('option');
            option.value = '';
            option.textContent = '路线规划未加载';
            option.disabled = true;
            destinationSelect.appendChild(option);
            return;
        }
        
        // 获取可用终点节点
        const availableNodes = this.getAvailableDestinationNodes();
        
        if (availableNodes.length === 0) {
            const option = document.createElement('option');
            option.value = '';
            option.textContent = '没有可用的终点节点';
            option.disabled = true;
            destinationSelect.appendChild(option);
            return;
        }
        
        // 添加节点选项
        availableNodes.forEach(node => {
            const nodeTypeNames = {
                'charging': '充电桩',
                'loading': '装载点',
                'delivery': '送达点',
                'custom': '自定义'
            };
            const typeName = nodeTypeNames[node.type] || '未知';
            
            const option = document.createElement('option');
            option.value = node.id;
            option.textContent = `${node.name} (${typeName})`;
            destinationSelect.appendChild(option);
        });
        
        // 如果之前选择了终点，尝试恢复选择
        if (this.selectedDestinationNodeId) {
            destinationSelect.value = this.selectedDestinationNodeId;
        }
    }
    
    /**
     * 机器人控制按钮处理
     */
    handleButtonDown(event) {
        if (!this.robotController) return;
        
        const action = event.target.dataset.action;
        if (action) {
            this.robotController.currentKeys.add(action);
            this.robotController.updateMovement();
            this.robotController.updateButtonStates();
        }
    }
    
    handleButtonUp(event) {
        if (!this.robotController) return;
        
        const action = event.target.dataset.action;
        if (action) {
            this.robotController.currentKeys.delete(action);
            this.robotController.updateMovement();
            this.robotController.updateButtonStates();
        }
    }
    
    /**
     * 开始录制
     */
    async startRecording() {
        console.log('🎬 开始录制waypoints...');
        
        try {
        if (!this.currentPathName.trim()) {
            this.showNotification('请输入路径名称', 'warning');
            return;
        }
        
            // 调用ROS2服务
            await this.callService('/start_recording');
            
            this.isRecording = true;
            this.waypoints = [];
            this.recordingStartTime = Date.now();
            
            // 开始录制时间更新
            this.startRecordingTimer();
            
            this.updateRecordingUI();
            this.showNotification('开始录制waypoints', 'success');
            
            console.log('✅ 录制已开始');
        } catch (error) {
            console.error('❌ 开始录制失败:', error);
            this.showNotification('开始录制失败: ' + error.message, 'error');
        }
    }
    
    /**
     * 停止录制
     */
    async stopRecording() {
        console.log('⏹️ 停止录制waypoints...');
        
        try {
            // 调用ROS2服务
            await this.callService('/stop_recording');
            
            this.isRecording = false;
            
            // 停止录制时间更新
            this.stopRecordingTimer();
            
            this.updateRecordingUI();
            this.showNotification('已停止录制waypoints', 'success');
            
            console.log('✅ 录制已停止');
        } catch (error) {
            console.error('❌ 停止录制失败:', error);
            this.showNotification('停止录制失败: ' + error.message, 'error');
        }
    }
    
    /**
     * 保存waypoints
     */
    async saveWaypoints() {
        console.log('💾 保存waypoints...');
        
        try {
        if (!this.currentPathName.trim()) {
            this.showNotification('请输入路径名称', 'warning');
            return;
        }
        
            // 调用ROS2服务
            await this.callService('/save_waypoints', {
                path_name: this.currentPathName
            });
            
            this.showNotification('Waypoints已保存', 'success');
            
            // 刷新文件列表
            await this.loadWaypointFiles();
            
            console.log('✅ Waypoints已保存');
        } catch (error) {
            console.error('❌ 保存waypoints失败:', error);
            this.showNotification('保存失败: ' + error.message, 'error');
        }
    }
    
    /**
     * 开始跟踪（从头开始）
     */
    async startFollowing() {
        console.log('🚀 开始跟踪waypoints...');
        
        try {
            // 检查紧急停止状态
            if (this.isEmergencyStopActive()) {
                this.showNotification('紧急停止状态下无法开始跟踪，请先复位', 'warning');
                return;
            }
            
            // 检查是否检测到APRILTAG（从机器人状态中检查）
            const robotLocationElement = document.getElementById('robotCurrentLocation');
            const currentLocation = robotLocationElement ? robotLocationElement.textContent : null;
            
            if (!currentLocation || currentLocation === '未定位') {
                this.showNotification('请确保APRILTAG可以被识别', 'warning');
                return;
            }
            
            // 检查当前位置节点是否配置
            if (!this.currentLocationNode) {
                this.showNotification(`当前位置"${currentLocation}"未在路线规划中配置，请在路线规划页面创建名称为"${currentLocation}"的节点并设置对应的APRILTAG ID`, 'warning');
                return;
            }
            
            // 检查是否选择了终点
            if (!this.selectedDestinationNodeId || this.selectedDestinationNodeId === '') {
                this.showNotification('请选择终点', 'warning');
                return;
            }
            
            // 查找从起点到终点的连接
            const connection = this.findConnectionBetweenNodes(
                this.currentLocationNode.id,
                this.selectedDestinationNodeId
            );
            
            if (!connection || !connection.waypointsFile) {
                this.showNotification('起点到终点之间没有配置路径，请在路线规划页面添加连接', 'warning');
                return;
            }
            
            // 确保清除所有暂停状态和索引（开始跟踪总是从头开始）
            this.isPaused = false;
            this.pausedWaypointIndex = 0;
            
            // 设置路径文件路径
            await this.callService('/set_waypoints_file_path', {
                file_path: connection.waypointsFile
            });
            
            // 开始跟踪（从头开始，索引0）
            // 注意：start_following服务会检查resume_waypoint_index参数
            // 如果为0或未设置，会从索引0开始
            await this.callService('/start_following');
            
            this.isFollowing = true;
            this.selectedWaypointFile = connection.waypointsFile; // 保存当前使用的文件路径
            this.updateFollowingUI();
            
            const startNode = this.currentLocationNode.node;
            const endNode = window.routePlanner.nodes.get(this.selectedDestinationNodeId);
            this.showNotification(`开始跟踪: ${startNode.name} -> ${endNode ? endNode.name : '未知'}`, 'success');
            
            // 发布自动导航目标点到状态机
            this.publishNavigationGoal();
            
            // 通知机器人控制器开始waypoint跟踪
            if (window.robotController) {
                window.robotController.setWaypointFollowing(true);
            }
            
            console.log('✅ 跟踪已开始，从第0个waypoint开始');
        } catch (error) {
            console.error('❌ 开始跟踪失败:', error);
            this.showNotification('开始跟踪失败: ' + error.message, 'error');
        }
    }
    
    /**
     * 切换暂停/继续跟踪
     */
    async togglePause() {
        if (this.isPaused) {
            // 当前是暂停状态，点击后继续跟踪
            await this.resumeFollowing();
        } else {
            // 当前是跟踪状态，点击后暂停跟踪
            await this.pauseFollowing();
        }
    }
    
    /**
     * 继续跟踪（从暂停点）
     */
    async resumeFollowing() {
        console.log('▶️ 继续跟踪waypoints...');
        
        try {
            if (!this.isPaused) {
                this.showNotification('当前未在暂停状态', 'warning');
                return;
            }
            
            if (this.pausedWaypointIndex <= 0) {
                this.showNotification('无效的暂停点索引', 'error');
                return;
            }
            
            // 检查是否有有效的文件路径
            if (!this.selectedWaypointFile) {
                this.showNotification('路径文件未设置，无法继续跟踪', 'error');
                return;
            }
            
            // 设置路径文件路径（确保文件已加载）
            await this.callService('/set_waypoints_file_path', {
                file_path: this.selectedWaypointFile
            });
            
            // 从暂停点继续跟踪
            await this.callService('/resume_following', {
                waypoint_index: this.pausedWaypointIndex
            });
            
            this.isFollowing = true;
            this.isPaused = false;
            const resumeIndex = this.pausedWaypointIndex;
            this.pausedWaypointIndex = 0;
            
            this.updateFollowingUI();
            this.showNotification(`继续跟踪，从第${resumeIndex}个waypoint开始`, 'success');
            
            // 发布自动导航目标点到状态机
            this.publishNavigationGoal();
            
            // 通知机器人控制器开始waypoint跟踪
            if (window.robotController) {
                window.robotController.setWaypointFollowing(true);
            }
            
            console.log(`✅ 跟踪已继续，从第${resumeIndex}个waypoint开始`);
        } catch (error) {
            console.error('❌ 继续跟踪失败:', error);
            this.showNotification('继续跟踪失败: ' + error.message, 'error');
        }
    }
    
    /**
     * 暂停跟踪
     */
    async pauseFollowing() {
        console.log('⏸️ 暂停跟踪waypoints...');
        
        try {
            if (!this.isFollowing) {
                this.showNotification('当前未在跟踪中', 'warning');
                return;
            }
            
            // 调用ROS2服务暂停跟踪，并获取当前waypoint索引
            const result = await this.callService('/pause_following');
            
            // 获取当前waypoint索引（从服务返回）
            const waypointIndex = result?.waypoint_index || 0;
            this.pausedWaypointIndex = waypointIndex;
            
            this.isFollowing = false;
            this.isPaused = true;
            this.updateFollowingUI();
            this.showNotification(`已暂停跟踪，当前到达第${this.pausedWaypointIndex}个waypoint`, 'success');
            
            // 通知机器人控制器停止waypoint跟踪
            if (window.robotController) {
                window.robotController.setWaypointFollowing(false);
            }
            
            console.log(`✅ 跟踪已暂停，暂停点索引: ${this.pausedWaypointIndex}`);
        } catch (error) {
            console.error('❌ 暂停跟踪失败:', error);
            this.showNotification('暂停跟踪失败: ' + error.message, 'error');
        }
    }
    
    publishNavigationGoal() {
        if (window.ros2Bridge && window.ros2Bridge.isConnected()) {
            // 发布一个默认的导航目标点（可以根据实际需要修改）
            const goalMsg = {
                pose: {
                    position: {
                        x: 1.0,
                        y: 0.0,
                        z: 0.0
                    },
                    orientation: {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0
                    }
                }
            };
            console.log('🔍 准备发送导航目标点:', goalMsg);
            const success = window.ros2Bridge.publish('/goal_pose', goalMsg);
            console.log('📤 已发送自动导航目标点到状态机, 结果:', success);
        } else {
            console.warn('⚠️ ROS2桥接未连接，无法发送导航目标点');
        }
    }
    
    /**
     * 停止跟踪
     */
    async stopFollowing() {
        console.log('⏹️ 停止跟踪waypoints...');
        
        try {
            // 调用ROS2服务
            await this.callService('/stop_following');
            
            // 清除所有跟踪状态，确保下次从头开始
            this.isFollowing = false;
            this.isPaused = false;  // 停止时清除暂停状态
            this.pausedWaypointIndex = 0;  // 重置暂停点索引
            
            // 更新UI
            this.updateFollowingUI();
            this.showNotification('已停止跟踪waypoints，状态已清零', 'success');
            
            // 通知机器人控制器停止waypoint跟踪
            if (window.robotController) {
                window.robotController.setWaypointFollowing(false);
            }
            
            console.log('✅ 跟踪已停止，所有状态已清零，下次将从第一个点开始');
        } catch (error) {
            console.error('❌ 停止跟踪失败:', error);
            this.showNotification('停止跟踪失败: ' + error.message, 'error');
        }
    }
    
    /**
     * Waypoint跟踪完成回调
     */
    onWaypointCompleted() {
        console.log('✅ Waypoint跟踪已完成');
        
        // 清空所有跟踪状态，确保下次从头开始
        this.isFollowing = false;
        this.isPaused = false;
        this.pausedWaypointIndex = 0;
        
        this.updateFollowingUI();
        this.showNotification('Waypoint跟踪已完成', 'success');
        
        // 通知机器人控制器停止waypoint跟踪
        if (window.robotController) {
            window.robotController.setWaypointFollowing(false);
        }
        
        // 确保状态机切换到空闲状态（通过WebSocket发送空的目标点或等待状态机自动切换）
        // 状态机已经通过waypoint_status话题接收到了完成消息，会自动切换
        
        console.log('✅ 已清空所有跟踪状态，下次将从第一个点开始');
    }
    
    /**
     * Waypoint跟踪停止回调
     */
    onWaypointStopped() {
        console.log('⏹️ Waypoint跟踪已停止');
        this.isFollowing = false;
        this.updateFollowingUI();
        this.showNotification('Waypoint跟踪已停止', 'info');
        
        // 通知机器人控制器停止waypoint跟踪
        if (window.robotController) {
            window.robotController.setWaypointFollowing(false);
        }
    }
    
    /**
     * 加载waypoints
     */
    async loadWaypoints() {
        console.log('📂 加载waypoints...');
        
        try {
            if (!this.selectedWaypointFile) {
                this.showNotification('请选择路径文件', 'warning');
            return;
        }
        
            // 设置路径文件路径
            await this.callService('/set_waypoints_file_path', {
                file_path: this.selectedWaypointFile
            });
            
            this.showNotification('Waypoints已加载', 'success');
            
            console.log('✅ Waypoints已加载');
        } catch (error) {
            console.error('❌ 加载waypoints失败:', error);
            this.showNotification('加载失败: ' + error.message, 'error');
        }
    }
    
    /**
     * 删除waypoints
     */
    async deleteWaypoints() {
        console.log('🗑️ 删除waypoints...');
        
        try {
            if (!this.selectedWaypointFile) {
                this.showNotification('请选择要删除的文件', 'warning');
            return;
        }
        
            if (!confirm('确定要删除选中的路径文件吗？')) {
                return;
            }
            
            // 这里需要实现删除文件的逻辑
            this.showNotification('删除功能待实现', 'info');
            
            console.log('✅ Waypoints删除功能待实现');
        } catch (error) {
            console.error('❌ 删除waypoints失败:', error);
            this.showNotification('删除失败: ' + error.message, 'error');
        }
    }
    
    /**
     * 刷新路径文件列表
     */
    async refreshWaypointFiles() {
        console.log('🔄 刷新路径文件列表...');
        
        try {
            await this.loadWaypointFiles();
            this.showNotification('文件列表已刷新', 'success');
        } catch (error) {
            console.error('❌ 刷新文件列表失败:', error);
            this.showNotification('刷新失败: ' + error.message, 'error');
        }
    }
    
    /**
     * 调用ROS2服务
     */
    async callService(serviceName, data = {}) {
        console.log('🔧 调用服务:', serviceName);
        
        try {
            const response = await fetch('http://localhost:8081/ros2_service', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    service: serviceName,
                    args: data
                })
            });
            
            if (!response.ok) {
                throw new Error(`HTTP错误: ${response.status}`);
            }
            
            const result = await response.json();
            console.log('✅ 服务调用成功:', serviceName);
            return result;
            
        } catch (error) {
            console.error('❌ 调用服务失败:', error);
            throw error;
        }
    }
    
    /**
     * 加载保存的路径
     */
    async loadSavedPaths() {
        // 这里可以实现从localStorage或其他存储加载路径的逻辑
        console.log('📂 加载保存的路径...');
    }
    
    /**
     * 加载路径文件列表
     */
    async loadWaypointFiles() {
        console.log('📂 加载路径文件列表...');
        
        try {
            const response = await fetch('http://localhost:8081/waypoint_files');
            const data = await response.json();
            
            console.log('📊 API响应数据:', data);
            
            // 检查响应格式
            if (!data.success) {
                throw new Error('API返回失败状态');
            }
            
            const files = data.files || [];
            console.log('📁 文件列表:', files);
            
            const fileSelect = document.getElementById('waypointFileSelect');
            if (fileSelect) {
                fileSelect.innerHTML = '<option value="">请选择路径文件...</option>';
                
                if (Array.isArray(files)) {
                    files.forEach(file => {
                        const option = document.createElement('option');
                        option.value = file;
                        option.textContent = file;
                        fileSelect.appendChild(option);
                    });
                } else {
                    console.warn('⚠️ files不是数组:', typeof files, files);
                }
            }
            
            console.log(`✅ 加载了 ${files.length} 个路径文件`);
        } catch (error) {
            console.error('❌ 加载路径文件列表失败:', error);
        }
    }
    
    /**
     * 更新录制UI
     */
    updateRecordingUI() {
        const startBtn = document.getElementById('startWaypointRecordingBtn');
        const stopBtn = document.getElementById('stopWaypointRecordingBtn');
        const status = document.getElementById('waypointRecordingStatus');
        const timeElement = document.getElementById('waypointRecordingTime');
        
        if (startBtn) startBtn.disabled = this.isRecording;
        if (stopBtn) stopBtn.disabled = !this.isRecording;
        if (status) {
            status.textContent = this.isRecording ? '录制中' : '未录制';
            status.style.color = this.isRecording ? '#e74c3c' : '#666';
        }
        if (timeElement && !this.isRecording) {
            timeElement.textContent = '00:00:00';
        }
    }
    
    /**
     * 更新跟踪UI
     */
    updateFollowingUI() {
        const startBtn = document.getElementById('startWaypointFollowingBtn');
        const pauseBtn = document.getElementById('pauseWaypointFollowingBtn');
        const stopBtn = document.getElementById('stopWaypointFollowingBtn');
        const status = document.getElementById('waypointFollowingStatus');
        
        // 更新按钮状态
        if (startBtn) {
            const isEmergencyStop = this.isEmergencyStopActive();
            startBtn.disabled = isEmergencyStop || this.isFollowing || this.isPaused;
        }
        
        if (pauseBtn) {
            pauseBtn.disabled = !this.isFollowing && !this.isPaused;
            // 更新按钮文本和图标
            if (this.isPaused) {
                pauseBtn.innerHTML = '<i class="fas fa-play"></i> 继续跟踪';
                pauseBtn.className = 'waypoint-btn pause continue';
            } else {
                pauseBtn.innerHTML = '<i class="fas fa-pause"></i> 暂停跟踪';
                pauseBtn.className = 'waypoint-btn pause';
            }
        }
        
        if (stopBtn) {
            stopBtn.disabled = !this.isFollowing && !this.isPaused;
        }
        
        // 更新状态显示
        if (status) {
            if (this.isPaused) {
                status.textContent = `已暂停 (第${this.pausedWaypointIndex}个waypoint)`;
                status.style.color = '#f39c12';
            } else if (this.isFollowing) {
                status.textContent = '跟踪中';
                status.style.color = '#3498db';
            } else {
                status.textContent = '未跟踪';
                status.style.color = '#666';
            }
        }
    }
    
    /**
     * 更新跟踪按钮状态
     */
    updateFollowingButtonStates() {
        const startBtn = document.getElementById('startWaypointFollowingBtn');
        if (startBtn) {
            const isEmergencyStop = this.isEmergencyStopActive();
            const hasCurrentLocation = this.currentLocationNode !== null;
            const hasDestination = this.selectedDestinationNodeId !== null && this.selectedDestinationNodeId !== '';
            startBtn.disabled = isEmergencyStop || !hasCurrentLocation || !hasDestination || this.isFollowing || this.isPaused;
        }
    }
    
    /**
     * 更新UI
     */
    updateUI() {
        this.updateRecordingUI();
        this.updateFollowingUI();
        this.updateFollowingButtonStates();
    }
    
    /**
     * 开始录制时间更新
     */
    startRecordingTimer() {
        this.stopRecordingTimer(); // 清除现有定时器
        
        this.recordingTimer = setInterval(() => {
            this.updateRecordingTime();
        }, 100); // 每100ms更新一次
    }
    
    /**
     * 停止录制时间更新
     */
    stopRecordingTimer() {
        if (this.recordingTimer) {
            clearInterval(this.recordingTimer);
            this.recordingTimer = null;
        }
    }
    
    /**
     * 更新录制时间显示
     */
    updateRecordingTime() {
        if (!this.isRecording || !this.recordingStartTime) {
            return;
        }
        
        const elapsed = Date.now() - this.recordingStartTime;
        const timeString = this.formatTime(elapsed);
        
        const timeElement = document.getElementById('waypointRecordingTime');
        if (timeElement) {
            timeElement.textContent = timeString;
        }
    }
    
    /**
     * 格式化时间显示
     */
    formatTime(milliseconds) {
        const totalSeconds = Math.floor(milliseconds / 1000);
        const hours = Math.floor(totalSeconds / 3600);
        const minutes = Math.floor((totalSeconds % 3600) / 60);
        const seconds = totalSeconds % 60;
        
        return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
    }
    
    /**
     * 显示通知
     */
    showNotification(message, type = 'info') {
        console.log(`📢 ${type.toUpperCase()}: ${message}`);
        
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
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
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
}

// 创建全局实例
window.waypointSystem = new WaypointSystem();

// 自动初始化
document.addEventListener('DOMContentLoaded', () => {
    if (window.waypointSystem) {
        window.waypointSystem.initialize();
    }
});

// 如果DOM已经加载完成，立即初始化
if (document.readyState !== 'loading') {
    if (window.waypointSystem) {
        window.waypointSystem.initialize();
    }
}





