/**
 * Waypoint导航系统 - 重写版本
 * 确保所有按钮功能正常工作
 */
class WaypointSystem {
    constructor() {
        console.log('🎯 初始化Waypoint系统...');
        
        // 状态管理
        this.isRecording = false;
        this.isFollowing = false;
        this.waypoints = [];
        this.currentPathName = '';
        this.selectedWaypointFile = '';
        
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
        this.setupButton('stopWaypointFollowingBtn', () => this.stopFollowing());
        
        // 路径管理按钮
        this.setupButton('loadWaypointsBtn', () => this.loadWaypoints());
        this.setupButton('deleteWaypointsBtn', () => this.deleteWaypoints());
        this.setupButton('refreshWaypointFilesBtn', () => this.refreshWaypointFiles());
        
        // 输入框事件
        this.setupInputEvents();
        
        // 文件选择器事件
        this.setupFileSelector();
        
        // 机器人控制按钮
        this.setupRobotControlButtons();
        
        console.log('✅ 事件监听器设置完成');
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
     * 设置文件选择器
     */
    setupFileSelector() {
        const fileSelect = document.getElementById('waypointFileSelect');
        if (fileSelect) {
            fileSelect.addEventListener('change', (e) => {
                this.selectedWaypointFile = e.target.value;
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
            
            // 加载waypoint文件列表
            await this.loadWaypointFiles();
            
            console.log('✅ 初始数据加载完成');
        } catch (error) {
            console.error('❌ 加载初始数据失败:', error);
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
     * 开始跟踪
     */
    async startFollowing() {
        console.log('🚀 开始跟踪waypoints...');
        
        try {
            if (!this.selectedWaypointFile) {
                this.showNotification('请选择waypoint文件', 'warning');
                return;
            }
            
            // 设置waypoint文件路径
            await this.callService('/set_waypoints_file_path', {
                file_path: this.selectedWaypointFile
            });
            
            // 开始跟踪
            await this.callService('/start_following');
            
            this.isFollowing = true;
            this.updateFollowingUI();
            this.showNotification('开始跟踪waypoints', 'success');
            
            console.log('✅ 跟踪已开始');
        } catch (error) {
            console.error('❌ 开始跟踪失败:', error);
            this.showNotification('开始跟踪失败: ' + error.message, 'error');
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
            
            this.isFollowing = false;
            this.updateFollowingUI();
            this.showNotification('已停止跟踪waypoints', 'success');
            
            console.log('✅ 跟踪已停止');
        } catch (error) {
            console.error('❌ 停止跟踪失败:', error);
            this.showNotification('停止跟踪失败: ' + error.message, 'error');
        }
    }
    
    /**
     * 加载waypoints
     */
    async loadWaypoints() {
        console.log('📂 加载waypoints...');
        
        try {
            if (!this.selectedWaypointFile) {
                this.showNotification('请选择waypoint文件', 'warning');
            return;
        }
        
            // 设置waypoint文件路径
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
        
            if (!confirm('确定要删除选中的waypoint文件吗？')) {
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
     * 刷新waypoint文件列表
     */
    async refreshWaypointFiles() {
        console.log('🔄 刷新waypoint文件列表...');
        
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
     * 加载waypoint文件列表
     */
    async loadWaypointFiles() {
        console.log('📂 加载waypoint文件列表...');
        
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
                fileSelect.innerHTML = '<option value="">请选择waypoints文件...</option>';
                
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
            
            console.log(`✅ 加载了 ${files.length} 个waypoint文件`);
        } catch (error) {
            console.error('❌ 加载waypoint文件列表失败:', error);
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
        const stopBtn = document.getElementById('stopWaypointFollowingBtn');
        const status = document.getElementById('waypointFollowingStatus');
        
        if (startBtn) startBtn.disabled = this.isFollowing;
        if (stopBtn) stopBtn.disabled = !this.isFollowing;
        if (status) {
            status.textContent = this.isFollowing ? '跟踪中' : '未跟踪';
            status.style.color = this.isFollowing ? '#3498db' : '#666';
        }
    }
    
    /**
     * 更新跟踪按钮状态
     */
    updateFollowingButtonStates() {
        const startBtn = document.getElementById('startWaypointFollowingBtn');
        if (startBtn) {
            startBtn.disabled = !this.selectedWaypointFile || this.isFollowing;
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
        
        // 这里可以实现更复杂的通知系统
        // 目前只是简单的console输出
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





