/**
 * 路线规划模块
 * 支持拖拽创建节点、节点间连接、加载waypoints文件
 */

class RoutePlanner {
    constructor() {
        console.log('🗺️ 初始化路线规划模块...');
        
        // 数据管理
        this.nodes = new Map(); // 节点数据 {id: nodeData}
        this.connections = new Map(); // 连接数据 {id: connectionData}
        this.selectedNodeId = null;
        this.connectionStartNodeId = null;
        
        // 画布状态
        this.scale = 1.0;
        this.panX = 0;
        this.panY = 0;
        this.isDragging = false;
        this.wasDragging = false; // 标记是否刚刚完成拖拽
        this.dragStartX = 0;
        this.dragStartY = 0;
        this.isPanning = false;
        
        // 渲染优化
        this.renderScheduled = false; // 标记是否已安排渲染
        this.draggedNodeElement = null; // 正在拖拽的节点元素（用于直接更新位置）
        
        // DOM元素引用
        this.canvas = null;
        this.nodesLayer = null;
        this.connectionsLayer = null;
        this.nodePropertiesPanel = null;
        this.connectionPropertiesPanel = null;
        this.loadedFilesList = null;
        
        // 选中的连接
        this.selectedConnectionId = null;
        
        // 节点计数器
        this.nodeIdCounter = 0;
        this.connectionIdCounter = 0;
        
        // 已加载的文件
        this.loadedFiles = [];
        
        // 初始化标志
        this.initialized = false;
        
        // 绑定方法
        this.handleCanvasClick = this.handleCanvasClick.bind(this);
        this.handleCanvasMouseDown = this.handleCanvasMouseDown.bind(this);
        this.handleCanvasMouseMove = this.handleCanvasMouseMove.bind(this);
        this.handleCanvasMouseUp = this.handleCanvasMouseUp.bind(this);
        this.handleCanvasWheel = this.handleCanvasWheel.bind(this);
    }
    
    /**
     * 初始化模块
     */
    async initialize() {
        if (this.initialized) return;
        
        try {
            // 等待DOM加载
            if (document.readyState === 'loading') {
                await new Promise(resolve => {
                    document.addEventListener('DOMContentLoaded', resolve);
                });
            }
            
            // 获取DOM元素
            this.canvas = document.getElementById('routePlannerCanvas');
            this.nodesLayer = document.getElementById('nodes-layer');
            this.connectionsLayer = document.getElementById('connections-layer');
            this.nodePropertiesPanel = document.getElementById('nodePropertiesPanel');
            this.connectionPropertiesPanel = document.getElementById('connectionPropertiesPanel');
            this.loadedFilesList = document.getElementById('loadedFilesList');
            
            if (!this.canvas || !this.nodesLayer || !this.connectionsLayer) {
                throw new Error('找不到必要的DOM元素');
            }
            
            // 设置画布尺寸
            this.setupCanvas();
            
            // 设置事件监听器
            this.setupEventListeners();
            
            // 初始化视图
            this.resetView();
            
            this.initialized = true;
            console.log('✅ 路线规划模块初始化完成');
            
        } catch (error) {
            console.error('❌ 路线规划模块初始化失败:', error);
        }
    }
    
    /**
     * 设置画布
     */
    setupCanvas() {
        const container = this.canvas.parentElement;
        const updateSize = () => {
            // 使用offsetWidth和offsetHeight确保获取实际尺寸
            const width = container.offsetWidth || container.clientWidth;
            const height = container.offsetHeight || container.clientHeight;
            
            if (width > 0 && height > 0) {
                this.canvas.setAttribute('width', width);
                this.canvas.setAttribute('height', height);
                this.render();
            } else {
                // 如果尺寸为0，延迟重试
                setTimeout(updateSize, 100);
            }
        };
        
        // 初始设置
        updateSize();
        
        // 监听窗口大小变化
        window.addEventListener('resize', updateSize);
        
        // 监听标签页切换（确保切换时重新计算尺寸）
        const observer = new MutationObserver(() => {
            setTimeout(updateSize, 50);
        });
        
        if (container) {
            observer.observe(container, {
                attributes: true,
                attributeFilter: ['style', 'class']
            });
        }
    }
    
    /**
     * 设置事件监听器
     */
    setupEventListeners() {
        // 工具栏按钮
        const addNodeBtn = document.getElementById('addNodeBtn');
        const loadWaypointsBtn = document.getElementById('loadWaypointsBtn');
        const waypointsFileInput = document.getElementById('waypointsFileInput');
        const clearCanvasBtn = document.getElementById('clearCanvasBtn');
        const zoomInBtn = document.getElementById('zoomInBtn');
        const zoomOutBtn = document.getElementById('zoomOutBtn');
        const resetViewBtn = document.getElementById('resetViewBtn');
        
        if (addNodeBtn) {
            addNodeBtn.addEventListener('click', () => this.addNodeAtCenter());
        }
        
        // 连接模式标志
        this.connectionMode = false;
        this.connectionModeBtn = null;
        
        if (loadWaypointsBtn && waypointsFileInput) {
            loadWaypointsBtn.addEventListener('click', () => waypointsFileInput.click());
            waypointsFileInput.addEventListener('change', (e) => this.handleFileLoad(e));
        }
        
        // 保存和导入路线按钮
        const saveRouteBtn = document.getElementById('saveRouteBtn');
        const importRouteBtn = document.getElementById('importRouteBtn');
        const routeFileInput = document.getElementById('routeFileInput');
        
        if (saveRouteBtn) {
            saveRouteBtn.addEventListener('click', () => this.saveRoute());
        }
        
        if (importRouteBtn && routeFileInput) {
            importRouteBtn.addEventListener('click', () => routeFileInput.click());
            routeFileInput.addEventListener('change', (e) => this.handleRouteImport(e));
        }
        
        if (clearCanvasBtn) {
            clearCanvasBtn.addEventListener('click', () => this.clearCanvas());
        }
        
        if (zoomInBtn) {
            zoomInBtn.addEventListener('click', () => this.zoomIn());
        }
        
        if (zoomOutBtn) {
            zoomOutBtn.addEventListener('click', () => this.zoomOut());
        }
        
        if (resetViewBtn) {
            resetViewBtn.addEventListener('click', () => this.resetView());
        }
        
        // 连接模式按钮（如果存在）
        const connectModeBtn = document.getElementById('connectModeBtn');
        if (connectModeBtn) {
            this.connectionModeBtn = connectModeBtn;
            connectModeBtn.addEventListener('click', () => this.toggleConnectionMode());
        }
        
        // 画布事件
        this.canvas.addEventListener('click', this.handleCanvasClick);
        this.canvas.addEventListener('mousedown', this.handleCanvasMouseDown);
        this.canvas.addEventListener('mousemove', this.handleCanvasMouseMove);
        this.canvas.addEventListener('mouseup', this.handleCanvasMouseUp);
        this.canvas.addEventListener('wheel', this.handleCanvasWheel);
        this.canvas.addEventListener('dblclick', this.handleCanvasDoubleClick);
        
        // 在document上监听mouseup，防止鼠标移出画布时拖拽状态无法释放
        document.addEventListener('mouseup', (e) => {
            if (this.isDragging || this.isPanning) {
                this.handleCanvasMouseUp(e);
            }
        });
        
        // 防止右键菜单
        this.canvas.addEventListener('contextmenu', (e) => e.preventDefault());
    }
    
    /**
     * 处理节点点击
     */
    handleNodeClick(node, e) {
        if (this.connectionMode) {
            // 连接模式
            if (this.connectionStartNodeId === null) {
                // 还没有起始节点，设置第一个节点
                this.connectionStartNodeId = node.id;
                this.selectNode(node.id);
                console.log(`🔗 选择起始节点: ${node.name}，请点击目标节点创建连接`);
                this.render();
            } else if (this.connectionStartNodeId === node.id) {
                // 点击同一个节点，取消起始节点选择
                this.connectionStartNodeId = null;
                this.selectNode(node.id);
                console.log('🔗 取消起始节点选择，请重新选择起始节点');
                this.render();
            } else {
                // 创建连接
                const existingConnection = this.findConnection(this.connectionStartNodeId, node.id);
                if (!existingConnection) {
                    const fromNode = this.nodes.get(this.connectionStartNodeId);
                    const connection = this.addConnection(this.connectionStartNodeId, node.id);
                    if (connection) {
                        console.log(`✅ 创建连接成功: ${fromNode?.name} -> ${node.name}`);
                    } else {
                        console.error(`❌ 创建连接失败: ${this.connectionStartNodeId} -> ${node.id}`);
                    }
                } else {
                    console.log('⚠️ 连接已存在');
                }
                // 保持连接模式，将当前节点设为新的起始节点，可以继续连接
                this.connectionStartNodeId = node.id;
                this.selectNode(node.id);
                this.render(); // 确保立即渲染
            }
        } else {
            // 普通模式：选择节点或开始连接
            if (e.shiftKey || e.ctrlKey || e.metaKey) {
                // 按住Shift/Ctrl/Cmd键：开始连接模式
                this.enterConnectionMode(node.id);
            } else {
                this.selectNode(node.id);
            }
        }
    }
    
    /**
     * 处理画布点击
     */
    handleCanvasClick(e) {
        // 如果刚刚完成拖拽，不处理点击（避免拖拽后误触发点击）
        if (this.wasDragging) {
            this.wasDragging = false;
            return;
        }
        
        // 如果点击的是连接线，不处理（连接线有自己的点击事件）
        if (e.target.getAttribute('data-connection-id')) {
            return;
        }
        
        const point = this.getCanvasPoint(e);
        const clickedNode = this.getNodeAtPoint(point.x, point.y);
        
        // 取消连接选择
        if (this.selectedConnectionId) {
            this.deselectConnection();
        }
        
        if (clickedNode) {
            if (this.connectionMode) {
                // 连接模式
                if (this.connectionStartNodeId === null) {
                    // 还没有起始节点，设置第一个节点
                    this.connectionStartNodeId = clickedNode.id;
                    this.selectNode(clickedNode.id);
                    console.log(`🔗 选择起始节点: ${clickedNode.name}，请点击目标节点创建连接`);
                } else if (this.connectionStartNodeId === clickedNode.id) {
                    // 点击同一个节点，取消起始节点选择
                    this.connectionStartNodeId = null;
                    this.selectNode(clickedNode.id);
                    console.log('🔗 取消起始节点选择，请重新选择起始节点');
                } else {
                    // 创建连接
                    const existingConnection = this.findConnection(this.connectionStartNodeId, clickedNode.id);
                    if (!existingConnection) {
                        const fromNode = this.nodes.get(this.connectionStartNodeId);
                        const connection = this.addConnection(this.connectionStartNodeId, clickedNode.id);
                        if (connection) {
                            console.log(`✅ 创建连接成功: ${fromNode?.name} -> ${clickedNode.name}`);
                        } else {
                            console.error(`❌ 创建连接失败: ${this.connectionStartNodeId} -> ${clickedNode.id}`);
                        }
                    } else {
                        console.log('⚠️ 连接已存在');
                    }
                    // 保持连接模式，将当前节点设为新的起始节点，可以继续连接
                    this.connectionStartNodeId = clickedNode.id;
                    this.selectNode(clickedNode.id);
                    this.render(); // 确保立即渲染
                }
            } else {
                // 普通模式：选择节点或开始连接
                if (e.shiftKey || e.ctrlKey || e.metaKey) {
                    // 按住Shift/Ctrl/Cmd键：开始连接模式
                    this.enterConnectionMode(clickedNode.id);
                } else {
                    this.selectNode(clickedNode.id);
                }
            }
        } else {
            // 点击空白处：取消连接模式
            if (this.connectionMode) {
                this.exitConnectionMode();
            }
            this.deselectNode();
        }
    }
    
    /**
     * 处理画布双击
     */
    handleCanvasDoubleClick(e) {
        const point = this.getCanvasPoint(e);
        const clickedNode = this.getNodeAtPoint(point.x, point.y);
        
        if (clickedNode) {
            // 双击节点：进入连接模式
            if (!this.connectionMode) {
                this.enterConnectionMode(clickedNode.id);
            } else {
                // 如果已在连接模式，退出
                this.exitConnectionMode();
            }
        }
    }
    
    /**
     * 进入连接模式
     */
    enterConnectionMode(nodeId) {
        this.connectionMode = true;
        this.connectionStartNodeId = nodeId;
        this.canvas.style.cursor = 'crosshair';
        this.selectNode(nodeId);
        
        if (this.connectionModeBtn) {
            this.connectionModeBtn.classList.add('active');
        }
        
        console.log('🔗 进入连接模式，点击另一个节点创建连接');
    }
    
    /**
     * 退出连接模式
     */
    exitConnectionMode() {
        this.connectionMode = false;
        this.connectionStartNodeId = null;
        this.canvas.style.cursor = 'grab';
        
        if (this.connectionModeBtn) {
            this.connectionModeBtn.classList.remove('active');
        }
        
        this.render();
    }
    
    /**
     * 切换连接模式
     */
    toggleConnectionMode() {
        if (this.connectionMode) {
            this.exitConnectionMode();
        } else {
            // 进入连接模式
            // 如果有选中的节点，从该节点开始；否则等待用户点击第一个节点
            if (this.selectedNodeId) {
                this.enterConnectionMode(this.selectedNodeId);
            } else {
                // 进入连接模式，但还没有起始节点
                this.connectionMode = true;
                this.connectionStartNodeId = null;
                this.canvas.style.cursor = 'crosshair';
                
                if (this.connectionModeBtn) {
                    this.connectionModeBtn.classList.add('active');
                }
                
                console.log('🔗 进入连接模式，请点击第一个节点开始连接');
            }
        }
    }
    
    /**
     * 查找连接
     */
    findConnection(fromNodeId, toNodeId) {
        for (const connection of this.connections.values()) {
            if ((connection.fromNodeId === fromNodeId && connection.toNodeId === toNodeId) ||
                (connection.fromNodeId === toNodeId && connection.toNodeId === fromNodeId)) {
                return connection;
            }
        }
        return null;
    }
    
    /**
     * 处理画布鼠标按下
     */
    handleCanvasMouseDown(e) {
        const point = this.getCanvasPoint(e);
        
        if (e.button === 0) { // 左键
            const clickedNode = this.getNodeAtPoint(point.x, point.y);
            
            if (clickedNode) {
                // 在连接模式下，不启动拖拽，让点击事件处理连接
                if (this.connectionMode) {
                    // 连接模式下不拖拽，只处理点击
                    return;
                }
                // 开始拖拽节点
                this.isDragging = true;
                this.selectedNodeId = clickedNode.id;
                // 计算拖拽起始偏移量（鼠标位置相对于节点中心）
                this.dragStartX = point.x - clickedNode.x;
                this.dragStartY = point.y - clickedNode.y;
                
                // 找到被拖拽的节点元素，用于直接更新位置
                if (this.nodesLayer) {
                    this.draggedNodeElement = this.nodesLayer.querySelector(`[data-node-id="${clickedNode.id}"]`);
                }
                
                // 设置鼠标样式
                if (this.canvas) {
                    this.canvas.style.cursor = 'grabbing';
                }
                
                this.selectNode(clickedNode.id);
                
                // 阻止默认行为，防止文本选择
                e.preventDefault();
                e.stopPropagation();
            } else {
                // 开始平移画布
                this.isPanning = true;
                this.panStartX = point.x;
                this.panStartY = point.y;
            }
        }
    }
    
    /**
     * 处理画布鼠标移动
     */
    handleCanvasMouseMove(e) {
        const point = this.getCanvasPoint(e);
        
        if (this.isDragging && this.selectedNodeId) {
            // 拖拽节点 - 直接更新节点位置和DOM，不完整重绘
            const node = this.nodes.get(this.selectedNodeId);
            if (node) {
                const newX = point.x - this.dragStartX;
                const newY = point.y - this.dragStartY;
                
                // 更新节点数据
                node.x = newX;
                node.y = newY;
                
                // 直接更新DOM元素位置，避免完整重绘
                if (this.draggedNodeElement) {
                    const group = this.draggedNodeElement;
                    // group的transform由render()统一管理，这里只更新节点内部坐标
                    
                    // 更新圆圈位置（节点坐标是逻辑坐标，直接使用）
                    const circle = group.querySelector('circle');
                    if (circle) {
                        circle.setAttribute('cx', newX);
                        circle.setAttribute('cy', newY);
                    }
                    
                    // 更新文本位置
                    const text = group.querySelector('text');
                    if (text) {
                        text.setAttribute('x', newX);
                        text.setAttribute('y', newY - 25);
                    }
                } else {
                    // 如果找不到元素，使用节流的完整渲染
                    this.scheduleRender();
                }
                
                // 更新连接线位置（只更新与被拖拽节点相关的连接）
                this.updateConnectionsForNode(this.selectedNodeId);
            }
        } else if (this.isPanning) {
            // 平移画布 - 只更新transform，不重绘内容
            const deltaX = point.x - this.panStartX;
            const deltaY = point.y - this.panStartY;
            this.panX += deltaX;
            this.panY += deltaY;
            this.panStartX = point.x;
            this.panStartY = point.y;
            
            // 直接更新transform，不重绘
            const transform = `translate(${this.panX}, ${this.panY}) scale(${this.scale})`;
            if (this.nodesLayer) {
                this.nodesLayer.setAttribute('transform', transform);
            }
            if (this.connectionsLayer) {
                this.connectionsLayer.setAttribute('transform', transform);
            }
        }
    }
    
    /**
     * 安排渲染（使用requestAnimationFrame节流）
     */
    scheduleRender() {
        if (this.renderScheduled) return;
        
        this.renderScheduled = true;
        requestAnimationFrame(() => {
            this.render();
            this.renderScheduled = false;
        });
    }
    
    /**
     * 更新与指定节点相关的连接线位置
     */
    updateConnectionsForNode(nodeId) {
        if (!this.connectionsLayer) return;
        
        // 找到所有与该节点相关的连接
        for (const connection of this.connections.values()) {
            if (connection.fromNodeId === nodeId || connection.toNodeId === nodeId) {
                const fromNode = this.nodes.get(connection.fromNodeId);
                const toNode = this.nodes.get(connection.toNodeId);
                
                if (!fromNode || !toNode) continue;
                
                // 找到对应的path元素
                const path = this.connectionsLayer.querySelector(`[data-connection-id="${connection.id}"]`);
                if (path) {
                    const dx = toNode.x - fromNode.x;
                    const dy = toNode.y - fromNode.y;
                    const distance = Math.sqrt(dx * dx + dy * dy);
                    const NODE_RADIUS = 20;
                    
                    if (distance > 0) {
                        const startX = fromNode.x + (dx / distance) * NODE_RADIUS;
                        const startY = fromNode.y + (dy / distance) * NODE_RADIUS;
                        const endX = toNode.x - (dx / distance) * NODE_RADIUS;
                        const endY = toNode.y - (dy / distance) * NODE_RADIUS;
                        
                        path.setAttribute('d', `M ${startX} ${startY} L ${endX} ${endY}`);
                        
                        // 更新文件名文本位置（如果有）
                        // 文本可能在path之后，也可能在connectionsLayer中作为独立元素
                        let text = path.nextElementSibling;
                        if (!text || text.tagName !== 'text') {
                            // 尝试在connectionsLayer中查找对应的文本元素
                            const allTexts = this.connectionsLayer.querySelectorAll('text');
                            for (const t of allTexts) {
                                // 检查文本内容是否匹配连接的文件名
                                if (connection.waypointsFile && t.textContent === connection.waypointsFile) {
                                    // 检查位置是否接近（允许一些误差）
                                    const textX = parseFloat(t.getAttribute('x') || 0);
                                    const textY = parseFloat(t.getAttribute('y') || 0);
                                    const midX = (startX + endX) / 2;
                                    const midY = (startY + endY) / 2;
                                    const dist = Math.sqrt(Math.pow(textX - midX, 2) + Math.pow(textY - midY, 2));
                                    if (dist < 50) { // 如果距离小于50像素，认为是同一个文本
                                        text = t;
                                        break;
                                    }
                                }
                            }
                        }
                        
                        if (text && text.tagName === 'text') {
                            text.setAttribute('x', (startX + endX) / 2);
                            text.setAttribute('y', (startY + endY) / 2 - 5);
                        }
                    }
                }
            }
        }
    }
    
    /**
     * 处理画布鼠标释放
     */
    handleCanvasMouseUp(e) {
        if (this.isDragging) {
            this.wasDragging = true;
            // 拖拽结束后，完整渲染一次确保所有元素位置正确
            this.render();
            // 延迟重置，避免立即触发点击事件
            setTimeout(() => {
                this.wasDragging = false;
            }, 100);
        }
        
        // 重置拖拽状态
        const wasDragging = this.isDragging;
        const wasPanning = this.isPanning;
        this.isDragging = false;
        this.isPanning = false;
        this.draggedNodeElement = null;
        
        // 如果之前正在拖拽或平移，重置鼠标样式
        if (wasDragging || wasPanning) {
            if (this.canvas) {
                this.canvas.style.cursor = 'default';
            }
        }
    }
    
    /**
     * 处理画布滚轮
     */
    handleCanvasWheel(e) {
        e.preventDefault();
        const delta = e.deltaY > 0 ? 0.9 : 1.1;
        const point = this.getCanvasPoint(e);
        this.zoomAtPoint(point.x, point.y, delta);
    }
    
    /**
     * 获取画布坐标点
     */
    getCanvasPoint(e) {
        const rect = this.canvas.getBoundingClientRect();
        // 如果事件有clientX和clientY，使用它们
        if (e.clientX !== undefined && e.clientY !== undefined) {
            return {
                x: (e.clientX - rect.left - this.panX) / this.scale,
                y: (e.clientY - rect.top - this.panY) / this.scale
            };
        }
        // 否则尝试从SVG坐标获取（用于节点点击事件）
        if (e.target && e.target.getAttribute) {
            const cx = parseFloat(e.target.getAttribute('cx') || '0');
            const cy = parseFloat(e.target.getAttribute('cy') || '0');
            if (cx && cy) {
                return { x: cx, y: cy };
            }
        }
        return { x: 0, y: 0 };
    }
    
    /**
     * 在指定点获取节点
     */
    getNodeAtPoint(x, y) {
        const NODE_RADIUS = 20;
        for (const node of this.nodes.values()) {
            const dx = x - node.x;
            const dy = y - node.y;
            const distance = Math.sqrt(dx * dx + dy * dy);
            if (distance <= NODE_RADIUS) {
                return node;
            }
        }
        return null;
    }
    
    /**
     * 在中心添加节点
     */
    addNodeAtCenter() {
        const centerX = this.canvas.getAttribute('width') / 2;
        const centerY = this.canvas.getAttribute('height') / 2;
        this.addNode(centerX, centerY);
    }
    
    /**
     * 添加节点（站点）
     */
    addNode(x, y, data = {}) {
        const nodeId = `node_${++this.nodeIdCounter}`;
        const node = {
            id: nodeId,
            name: data.name || `站点 ${this.nodeIdCounter}`,
            x: x,
            y: y,
            type: data.type || 'custom',
            markerId: data.markerId || null, // APRILTAG标记ID
            // 站点位置（可选，用于显示实际坐标）
            position: data.position || null,
            customProps: data.customProps || {}
        };
        
        this.nodes.set(nodeId, node);
        this.render();
        this.selectNode(nodeId);
        
        return node;
    }
    
    /**
     * 删除节点
     */
    deleteNode(nodeId) {
        // 删除节点
        this.nodes.delete(nodeId);
        
        // 删除相关连接
        const connectionsToDelete = [];
        for (const [connId, conn] of this.connections.entries()) {
            if (conn.fromNodeId === nodeId || conn.toNodeId === nodeId) {
                connectionsToDelete.push(connId);
            }
        }
        connectionsToDelete.forEach(id => this.connections.delete(id));
        
        if (this.selectedNodeId === nodeId) {
            this.deselectNode();
        }
        
        this.render();
    }
    
    /**
     * 选择节点
     */
    selectNode(nodeId) {
        this.selectedNodeId = nodeId;
        this.updatePropertiesPanel();
        this.render();
    }
    
    /**
     * 取消选择节点
     */
    deselectNode() {
        this.selectedNodeId = null;
        this.updatePropertiesPanel();
        this.render();
    }
    
    /**
     * 选择连接
     */
    selectConnection(connId) {
        this.selectedConnectionId = connId;
        this.selectedNodeId = null; // 取消节点选择
        this.updatePropertiesPanel();
        this.updateConnectionPropertiesPanel();
        this.render();
    }
    
    /**
     * 取消选择连接
     */
    deselectConnection() {
        this.selectedConnectionId = null;
        this.updateConnectionPropertiesPanel();
        this.render();
    }
    
    /**
     * 更新连接属性面板
     */
    updateConnectionPropertiesPanel() {
        if (!this.connectionPropertiesPanel) return;
        
        if (!this.selectedConnectionId) {
            this.connectionPropertiesPanel.innerHTML = '<p class="no-selection">未选择连接（单击连接线选择）</p>';
            return;
        }
        
        const connection = this.connections.get(this.selectedConnectionId);
        if (!connection) return;
        
        const fromNode = this.nodes.get(connection.fromNodeId);
        const toNode = this.nodes.get(connection.toNodeId);
        
        // 获取已加载的文件列表
        const fileOptions = this.loadedFiles.map(file => 
            `<option value="${this.escapeHtml(file.name)}" ${connection.waypointsFile === file.name ? 'selected' : ''}>${this.escapeHtml(file.name)}</option>`
        ).join('');
        
        const html = `
            <div class="property-form">
                <div class="property-field">
                    <label>起点站点:</label>
                    <input type="text" value="${this.escapeHtml(fromNode?.name || '未知')}" disabled>
                </div>
                <div class="property-field">
                    <label>终点站点:</label>
                    <input type="text" value="${this.escapeHtml(toNode?.name || '未知')}" disabled>
                </div>
                <div class="property-field">
                    <label>Waypoints文件:</label>
                    <select id="connectionWaypointsFile" onchange="window.routePlanner.updateConnectionWaypointsFile('${connection.id}', this.value)">
                        <option value="">-- 未选择 --</option>
                        ${fileOptions}
                    </select>
                </div>
                ${connection.waypointsFile ? `
                <div class="property-field">
                    <label>文件信息:</label>
                    <div style="font-size: 0.85rem; color: #666; padding: 0.5rem; background: #f8f9fa; border-radius: 4px;">
                        ${connection.waypointsData?.waypoints?.length || 0} 个waypoints
                    </div>
                </div>
                ` : ''}
                <div class="property-actions">
                    <button class="delete-btn" onclick="window.routePlanner.deleteConnection('${connection.id}')">
                        <i class="fas fa-trash"></i> 删除连接
                    </button>
                </div>
            </div>
        `;
        
        this.connectionPropertiesPanel.innerHTML = html;
    }
    
    /**
     * 更新连接的waypoints文件
     */
    updateConnectionWaypointsFile(connId, filename) {
        const connection = this.connections.get(connId);
        if (!connection) return;
        
        if (filename) {
            // 查找文件数据
            const fileData = this.loadedFiles.find(f => f.name === filename);
            if (fileData) {
                connection.waypointsFile = filename;
                connection.waypointsData = fileData.data;
                console.log(`✅ 为连接设置waypoints文件: ${filename}`);
            }
        } else {
            connection.waypointsFile = null;
            connection.waypointsData = null;
        }
        
        this.updateConnectionPropertiesPanel();
        this.render();
    }
    
    /**
     * 更新属性面板
     */
    updatePropertiesPanel() {
        if (!this.nodePropertiesPanel) return;
        
        if (!this.selectedNodeId) {
            this.nodePropertiesPanel.innerHTML = '<p class="no-selection">未选择站点</p>';
            return;
        }
        
        const node = this.nodes.get(this.selectedNodeId);
        if (!node) return;
        
        const html = `
            <div class="property-form">
                <div class="property-field">
                    <label>站点名称:</label>
                    <input type="text" id="nodeNameInput" value="${this.escapeHtml(node.name)}" 
                           onchange="window.routePlanner.updateNodeProperty('name', this.value)">
                </div>
                <div class="property-field">
                    <label>站点类型:</label>
                    <select id="nodeTypeInput" onchange="window.routePlanner.updateNodeProperty('type', this.value)">
                        <option value="charging" ${node.type === 'charging' ? 'selected' : ''}>充电桩</option>
                        <option value="loading" ${node.type === 'loading' ? 'selected' : ''}>装载点</option>
                        <option value="delivery" ${node.type === 'delivery' ? 'selected' : ''}>送达点</option>
                        <option value="custom" ${node.type === 'custom' ? 'selected' : ''}>自定义</option>
                    </select>
                </div>
                <div class="property-field">
                    <label>标记ID (APRILTAG):</label>
                    <input type="number" id="nodeMarkerIdInput" 
                           value="${node.markerId !== null && node.markerId !== undefined ? node.markerId : ''}" 
                           placeholder="输入APRILTAG ID"
                           onchange="window.routePlanner.updateNodeProperty('markerId', this.value ? parseInt(this.value) : null)">
                </div>
                <div class="property-actions">
                    <button class="delete-btn" onclick="window.routePlanner.deleteNode('${node.id}')">
                        <i class="fas fa-trash"></i> 删除站点
                    </button>
                </div>
            </div>
        `;
        
        this.nodePropertiesPanel.innerHTML = html;
    }
    
    /**
     * 更新节点属性
     */
    updateNodeProperty(property, value) {
        if (!this.selectedNodeId) return;
        
        const node = this.nodes.get(this.selectedNodeId);
        if (!node) return;
        
        switch (property) {
            case 'name':
                node.name = value;
                break;
            case 'type':
                node.type = value;
                break;
            case 'markerId':
                // 处理标记ID：如果为空字符串或null，设置为null；否则转换为整数
                if (value === '' || value === null || value === undefined) {
                    node.markerId = null;
                } else {
                    const markerId = parseInt(value);
                    node.markerId = isNaN(markerId) ? null : markerId;
                }
                break;
        }
        
        this.render();
    }
    
    /**
     * 添加连接（站点之间的路径）
     */
    addConnection(fromNodeId, toNodeId, waypointsFile = null) {
        // 检查是否已存在连接
        const existing = this.findConnection(fromNodeId, toNodeId);
        if (existing) {
            // 如果已存在，更新waypoints文件
            if (waypointsFile) {
                existing.waypointsFile = waypointsFile;
                this.render();
            }
            return existing;
        }
        
        const connId = `conn_${++this.connectionIdCounter}`;
        const connection = {
            id: connId,
            fromNodeId: fromNodeId,
            toNodeId: toNodeId,
            waypointsFile: waypointsFile || null, // waypoints文件路径
            waypointsData: null // 存储waypoints数据（可选）
        };
        
        this.connections.set(connId, connection);
        console.log(`📝 添加连接: ${fromNodeId} -> ${toNodeId}, 连接ID: ${connId}, 总连接数: ${this.connections.size}`);
        
        // 验证连接已添加
        const verifyConnection = this.connections.get(connId);
        if (verifyConnection) {
            console.log(`✅ 连接已成功添加到Map: ${verifyConnection.id}`);
        } else {
            console.error(`❌ 连接添加失败，无法从Map中获取: ${connId}`);
        }
        
        this.render();
        
        return connection;
    }
    
    /**
     * 删除连接
     */
    deleteConnection(connId) {
        this.connections.delete(connId);
        if (this.selectedConnectionId === connId) {
            this.deselectConnection();
        }
        this.render();
    }
    
    /**
     * 渲染画布
     */
    render() {
        if (!this.canvas || !this.nodesLayer || !this.connectionsLayer) return;
        
        // 如果正在拖拽，不完整重绘（位置已在handleCanvasMouseMove中更新）
        if (this.isDragging) {
            return;
        }
        
        // 清空图层
        this.connectionsLayer.innerHTML = '';
        this.nodesLayer.innerHTML = '';
        
        // 重置拖拽元素引用
        this.draggedNodeElement = null;
        
        // 应用变换
        const transform = `translate(${this.panX}, ${this.panY}) scale(${this.scale})`;
        this.nodesLayer.setAttribute('transform', transform);
        this.connectionsLayer.setAttribute('transform', transform);
        
        // 先渲染连接（在节点下方）
        let renderedConnections = 0;
        for (const connection of this.connections.values()) {
            try {
                this.renderConnection(connection);
                renderedConnections++;
            } catch (error) {
                console.error(`❌ 渲染连接失败: ${connection.id}`, error);
            }
        }
        
        // 再渲染节点（在连接上方）
        for (const node of this.nodes.values()) {
            this.renderNode(node);
        }
        
        // 调试信息
        if (this.connections.size > 0) {
            console.log(`📊 渲染连接: ${renderedConnections}/${this.connections.size} 条成功`);
            if (renderedConnections < this.connections.size) {
                console.warn(`⚠️ 有 ${this.connections.size - renderedConnections} 条连接未能渲染`);
            }
        }
    }
    
    /**
     * 渲染节点
     */
    renderNode(node) {
        const isSelected = node.id === this.selectedNodeId;
        const isConnectionStart = node.id === this.connectionStartNodeId;
        const isConnectionMode = this.connectionMode;
        const NODE_RADIUS = 20;
        
        // 节点颜色
        const colors = {
            'charging': '#27ae60',  // 充电桩：绿色
            'loading': '#f39c12',   // 装载点：黄色
            'delivery': '#3498db',  // 送达点：蓝色
            'custom': '#9b59b6'     // 自定义：紫色
        };
        const color = colors[node.type] || colors.custom;
        
        // 创建节点组
        const group = document.createElementNS('http://www.w3.org/2000/svg', 'g');
        group.setAttribute('class', 'route-node');
        group.setAttribute('data-node-id', node.id);
        
        // 节点圆圈
        const circle = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
        circle.setAttribute('cx', node.x);
        circle.setAttribute('cy', node.y);
        circle.setAttribute('r', NODE_RADIUS);
        circle.setAttribute('fill', color);
        
        // 根据状态设置边框
        if (isConnectionStart) {
            // 连接模式的起始节点：红色边框
            circle.setAttribute('stroke', '#e74c3c');
            circle.setAttribute('stroke-width', 3);
            circle.setAttribute('opacity', '0.9');
        } else if (isConnectionMode && !isConnectionStart) {
            // 连接模式下但未选中的节点：黄色边框（提示可以点击）
            circle.setAttribute('stroke', '#f39c12');
            circle.setAttribute('stroke-width', 2.5);
        } else {
            // 普通状态
            circle.setAttribute('stroke', isSelected ? '#2c3e50' : '#fff');
            circle.setAttribute('stroke-width', isSelected ? 3 : 2);
        }
        
        circle.setAttribute('cursor', isConnectionMode ? 'pointer' : 'move');
        circle.setAttribute('pointer-events', 'all');
        
        // 节点标签
        const text = document.createElementNS('http://www.w3.org/2000/svg', 'text');
        text.setAttribute('x', node.x);
        text.setAttribute('y', node.y - NODE_RADIUS - 5);
        text.setAttribute('text-anchor', 'middle');
        text.setAttribute('font-size', '12');
        text.setAttribute('fill', '#2c3e50');
        text.setAttribute('font-weight', isSelected || isConnectionStart ? 'bold' : 'normal');
        text.setAttribute('pointer-events', 'none');
        text.textContent = node.name;
        
        // 为节点组添加点击事件
        const handleNodeClickEvent = (e) => {
            e.stopPropagation();
            e.preventDefault();
            console.log(`🖱️ 节点被点击: ${node.name} (${node.id}), 连接模式: ${this.connectionMode}, 起始节点: ${this.connectionStartNodeId}`);
            this.handleNodeClick(node, e);
        };
        
        group.addEventListener('click', handleNodeClickEvent);
        // 也为圆圈添加点击事件，确保能捕获
        circle.addEventListener('click', handleNodeClickEvent);
        
        group.appendChild(circle);
        group.appendChild(text);
        this.nodesLayer.appendChild(group);
    }
    
    /**
     * 渲染连接
     */
    renderConnection(connection) {
        const fromNode = this.nodes.get(connection.fromNodeId);
        const toNode = this.nodes.get(connection.toNodeId);
        
        if (!fromNode || !toNode) {
            console.warn(`⚠️ 连接 ${connection.id} 的节点不存在: from=${connection.fromNodeId}, to=${connection.toNodeId}`);
            return;
        }
        
        const isSelected = connection.id === this.selectedConnectionId;
        const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        const dx = toNode.x - fromNode.x;
        const dy = toNode.y - fromNode.y;
        const distance = Math.sqrt(dx * dx + dy * dy);
        const NODE_RADIUS = 20;
        
        // 计算起点和终点（考虑节点半径）
        const startX = fromNode.x + (dx / distance) * NODE_RADIUS;
        const startY = fromNode.y + (dy / distance) * NODE_RADIUS;
        const endX = toNode.x - (dx / distance) * NODE_RADIUS;
        const endY = toNode.y - (dy / distance) * NODE_RADIUS;
        
        path.setAttribute('d', `M ${startX} ${startY} L ${endX} ${endY}`);
        path.setAttribute('stroke', isSelected ? '#e74c3c' : '#000000'); // 黑色连接线，选中时为红色
        path.setAttribute('stroke-width', isSelected ? '3' : '2');
        path.setAttribute('fill', 'none');
        path.setAttribute('marker-end', 'url(#arrowhead)');
        path.setAttribute('cursor', 'pointer');
        path.setAttribute('data-connection-id', connection.id);
        
        // 单击选择连接
        path.addEventListener('click', (e) => {
            e.stopPropagation();
            this.selectConnection(connection.id);
        });
        
        // 双击删除连接
        path.addEventListener('dblclick', (e) => {
            e.stopPropagation();
            if (confirm('确定要删除这条连接吗？')) {
                this.deleteConnection(connection.id);
            }
        });
        
        // 如果有waypoints文件，在连接中点显示文件名
        if (connection.waypointsFile) {
            const midX = (startX + endX) / 2;
            const midY = (startY + endY) / 2;
            
            const text = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            text.setAttribute('x', midX);
            text.setAttribute('y', midY - 5);
            text.setAttribute('text-anchor', 'middle');
            text.setAttribute('font-size', '10');
            text.setAttribute('fill', '#000000'); // 文件名也用黑色
            text.setAttribute('font-weight', 'bold');
            text.setAttribute('pointer-events', 'none');
            text.textContent = connection.waypointsFile;
            
            this.connectionsLayer.appendChild(text);
        }
        
        this.connectionsLayer.appendChild(path);
    }
    
    /**
     * 处理文件加载
     */
    async handleFileLoad(event) {
        // 首先尝试从服务器获取waypoints文件列表
        try {
            const response = await fetch('http://localhost:8081/waypoint_files');
            const result = await response.json();
            
            if (result.success && result.files && result.files.length > 0) {
                // 显示文件选择对话框
                const filename = await this.showWaypointFileSelector(result.files);
                if (filename) {
                    await this.loadWaypointFromServer(filename);
                }
            } else {
                // 如果没有服务器文件，使用本地文件选择
                const file = event.target.files[0];
                if (!file) return;
                
                const text = await file.text();
                const data = JSON.parse(text);
                this.loadWaypointsFile(data, file.name);
                
                // 添加到已加载文件列表
                this.loadedFiles.push({
                    name: file.name,
                    data: data
                });
                this.updateLoadedFilesList();
            }
            
            // 清空文件输入
            event.target.value = '';
            
        } catch (error) {
            console.error('加载waypoints文件失败:', error);
            // 如果服务器失败，尝试使用本地文件
            const file = event.target.files[0];
            if (file) {
                try {
                    const text = await file.text();
                    const data = JSON.parse(text);
                    this.loadWaypointsFile(data, file.name);
                    
                    // 添加到已加载文件列表
                    this.loadedFiles.push({
                        name: file.name,
                        data: data
                    });
                    this.updateLoadedFilesList();
                } catch (localError) {
                    alert('加载waypoints文件失败: ' + error.message);
                }
            } else {
                alert('加载waypoints文件失败: ' + error.message);
            }
            
            // 清空文件输入
            event.target.value = '';
        }
    }
    
    /**
     * 显示waypoints文件选择器
     */
    showWaypointFileSelector(files) {
        return new Promise((resolve) => {
            const fileList = files.map((f, i) => `${i + 1}. ${f}`).join('\n');
            const selection = prompt(`请选择要加载的waypoints文件（输入序号）:\n\n${fileList}\n\n输入0取消`);
            
            if (!selection || selection === '0') {
                resolve(null);
                return;
            }
            
            const index = parseInt(selection) - 1;
            if (index >= 0 && index < files.length) {
                resolve(files[index]);
            } else {
                alert('无效的选择');
                resolve(null);
            }
        });
    }
    
    /**
     * 从服务器加载waypoints文件
     */
    async loadWaypointFromServer(filename) {
        try {
            const response = await fetch(`http://localhost:8081/waypoint_file?name=${encodeURIComponent(filename)}`);
            
            if (!response.ok) {
                throw new Error(`HTTP错误: ${response.status} ${response.statusText}`);
            }
            
            const result = await response.json();
            
            if (result.success && result.data) {
                this.loadWaypointsFile(result.data, filename);
                
                // 添加到已加载文件列表
                this.loadedFiles.push({
                    name: filename,
                    data: result.data
                });
                this.updateLoadedFilesList();
            } else {
                throw new Error(result.error || '加载失败');
            }
        } catch (error) {
            console.error('从服务器加载waypoints文件失败:', error);
            
            // 提供更友好的错误提示
            let errorMessage = '加载waypoints文件失败: ';
            if (error.message.includes('NetworkError') || error.message.includes('Failed to fetch')) {
                errorMessage += '无法连接到服务器（localhost:8081）。\n\n请确保ROS2服务代理正在运行。';
            } else {
                errorMessage += error.message;
            }
            
            alert(errorMessage);
        }
    }
    
    /**
     * 加载waypoints文件
     * 创建两个站点（起点和终点），并用waypoints文件连接它们
     */
    loadWaypointsFile(data, filename) {
        if (!data.waypoints || !Array.isArray(data.waypoints)) {
            console.warn('无效的waypoints文件格式');
            return;
        }
        
        const waypoints = data.waypoints;
        if (waypoints.length === 0) return;
        
        // 获取起点和终点
        const startWaypoint = waypoints[0];
        const endWaypoint = waypoints[waypoints.length - 1];
        
        // 计算坐标范围，用于映射到画布
        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;
        
        waypoints.forEach(wp => {
            const x = wp.position.x;
            const y = wp.position.y;
            minX = Math.min(minX, x);
            maxX = Math.max(maxX, x);
            minY = Math.min(minY, y);
            maxY = Math.max(maxY, y);
        });
        
        const rangeX = maxX - minX || 1;
        const rangeY = maxY - minY || 1;
        const canvasWidth = parseFloat(this.canvas.getAttribute('width'));
        const canvasHeight = parseFloat(this.canvas.getAttribute('height'));
        const margin = 150;
        const scaleX = (canvasWidth - 2 * margin) / rangeX;
        const scaleY = (canvasHeight - 2 * margin) / rangeY;
        const scale = Math.min(scaleX, scaleY);
        
        // 提取文件名（去除路径和扩展名）
        const fileNameOnly = filename.replace(/^.*[\\\/]/, '').replace(/\.json$/, '');
        
        // 创建起点站点
        const startCanvasX = margin + (startWaypoint.position.x - minX) * scale;
        const startCanvasY = margin + (startWaypoint.position.y - minY) * scale;
        const startNode = this.addNode(startCanvasX, startCanvasY, {
            name: `${fileNameOnly}_起点`,
            type: 'custom',
            position: startWaypoint.position
        });
        
        // 创建终点站点
        const endCanvasX = margin + (endWaypoint.position.x - minX) * scale;
        const endCanvasY = margin + (endWaypoint.position.y - minY) * scale;
        const endNode = this.addNode(endCanvasX, endCanvasY, {
            name: `${fileNameOnly}_终点`,
            type: 'custom',
            position: endWaypoint.position
        });
        
        // 创建连接，关联waypoints文件
        const connection = this.addConnection(startNode.id, endNode.id, filename);
        connection.waypointsData = data; // 存储waypoints数据
        
        console.log(`✅ 加载waypoints文件: ${filename}`);
        console.log(`   起点站点: ${startNode.name}`);
        console.log(`   终点站点: ${endNode.name}`);
        console.log(`   Waypoints数量: ${waypoints.length}`);
        
        // 提示用户可以重命名站点
        setTimeout(() => {
            this.selectNode(startNode.id);
            console.log(`✅ 已创建两个站点连接，waypoints文件: ${filename}`);
        }, 100);
    }
    
    /**
     * 更新已加载文件列表
     */
    updateLoadedFilesList() {
        if (!this.loadedFilesList) return;
        
        if (this.loadedFiles.length === 0) {
            this.loadedFilesList.innerHTML = '<p class="no-files">暂无加载的路径</p>';
            return;
        }
        
        const html = this.loadedFiles.map((file, index) => {
            // 安全地获取waypoints数量
            let waypointsCount = 0;
            if (file.data && file.data.waypoints && Array.isArray(file.data.waypoints)) {
                waypointsCount = file.data.waypoints.length;
            }
            
            return `
            <div class="file-item">
                <i class="fas fa-file"></i>
                <span>${this.escapeHtml(file.name)}</span>
                <span class="file-waypoints-count">${waypointsCount > 0 ? waypointsCount + ' 个点' : '未加载'}</span>
            </div>
        `;
        }).join('');
        
        this.loadedFilesList.innerHTML = html;
    }
    
    /**
     * 清空画布
     */
    clearCanvas() {
        if (confirm('确定要清空画布吗？所有节点和连接将被删除。')) {
            this.nodes.clear();
            this.connections.clear();
            this.loadedFiles = [];
            this.selectedNodeId = null;
            this.updatePropertiesPanel();
            this.updateLoadedFilesList();
            this.render();
        }
    }
    
    /**
     * 缩放
     */
    zoomIn() {
        this.zoomAtPoint(
            parseFloat(this.canvas.getAttribute('width')) / 2,
            parseFloat(this.canvas.getAttribute('height')) / 2,
            1.2
        );
    }
    
    zoomOut() {
        this.zoomAtPoint(
            parseFloat(this.canvas.getAttribute('width')) / 2,
            parseFloat(this.canvas.getAttribute('height')) / 2,
            0.8
        );
    }
    
    zoomAtPoint(x, y, factor) {
        const newScale = Math.max(0.1, Math.min(5.0, this.scale * factor));
        const scaleChange = newScale / this.scale;
        
        this.panX = x - (x - this.panX) * scaleChange;
        this.panY = y - (y - this.panY) * scaleChange;
        this.scale = newScale;
        
        this.render();
    }
    
    /**
     * 重置视图
     */
    resetView() {
        this.scale = 1.0;
        this.panX = 0;
        this.panY = 0;
        this.render();
    }
    
    /**
     * 保存路线配置
     */
    async saveRoute() {
        if (this.nodes.size === 0) {
            alert('没有可保存的站点，请先创建站点');
            return;
        }
        
        // 准备保存的数据
        const routeData = {
            version: '1.0',
            metadata: {
                created_at: new Date().toISOString(),
                node_count: this.nodes.size,
                connection_count: this.connections.size
            },
            nodes: Array.from(this.nodes.values()).map(node => ({
                id: node.id,
                name: node.name,
                x: node.x,
                y: node.y,
                type: node.type,
                markerId: node.markerId || null,
                position: node.position,
                customProps: node.customProps
            })),
            connections: Array.from(this.connections.values()).map(conn => ({
                id: conn.id,
                fromNodeId: conn.fromNodeId,
                toNodeId: conn.toNodeId,
                waypointsFile: conn.waypointsFile,
                // 注意：waypointsData不保存，只保存文件名引用
            })),
            loadedFiles: this.loadedFiles.map(file => ({
                name: file.name,
                // 不保存完整的waypoints数据，只保存文件名
            }))
        };
        
        // 生成文件名（带时间戳）
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, -5);
        const filename = `route_config_${timestamp}.json`;
        
        // 首先尝试保存到服务器
        try {
            // 添加超时控制
            const controller = new AbortController();
            const timeoutId = setTimeout(() => controller.abort(), 5000); // 5秒超时
            
            const response = await fetch('http://localhost:8081/save_route', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    filename: filename,
                    data: routeData
                }),
                signal: controller.signal
            });
            
            clearTimeout(timeoutId);
            
            if (!response.ok) {
                const errorText = await response.text();
                console.error('服务器响应错误:', response.status, errorText);
                throw new Error(`HTTP错误: ${response.status} ${response.statusText}`);
            }
            
            const result = await response.json();
            
            if (result.success) {
                console.log('✅ 路线配置已保存到服务器:', result.filepath);
                alert(`路线配置已保存到routes文件夹:\n${filename}`);
                return;
            } else {
                throw new Error(result.error || '保存失败');
            }
        } catch (error) {
            console.warn('保存到服务器失败:', error);
            
            // 如果服务器不可用，提供下载到本地的备用方案
            let errorMessage = '';
            if (error.name === 'AbortError') {
                errorMessage = '连接服务器超时（超过5秒）。\n\n服务器可能响应缓慢或未正常运行。\n\n是否要下载到本地？';
            } else if (error.message.includes('NetworkError') || error.message.includes('Failed to fetch')) {
                errorMessage = '无法连接到服务器（localhost:8081）。\n\n请确保ROS2服务代理正在运行。\n\n是否要下载到本地？';
            } else {
                errorMessage = `保存到服务器失败: ${error.message}\n\n是否要下载到本地？`;
            }
            
            const downloadToLocal = confirm(errorMessage);
            
            if (downloadToLocal) {
                // 下载到本地
                this.downloadRouteToLocal(routeData, filename);
            }
        }
    }
    
    /**
     * 下载路线配置到本地
     */
    downloadRouteToLocal(routeData, filename) {
        try {
            // 转换为JSON字符串
            const jsonString = JSON.stringify(routeData, null, 2);
            
            // 创建下载链接
            const blob = new Blob([jsonString], { type: 'application/json' });
            const url = URL.createObjectURL(blob);
            const link = document.createElement('a');
            link.href = url;
            link.download = filename;
            
            // 触发下载
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            URL.revokeObjectURL(url);
            
            console.log('✅ 路线配置已下载到本地:', filename);
            alert(`路线配置已下载到本地:\n${filename}\n\n请手动将文件保存到项目的routes文件夹中。`);
        } catch (error) {
            console.error('下载到本地失败:', error);
            alert('下载到本地失败: ' + error.message);
        }
    }
    
    /**
     * 处理路线导入
     */
    async handleRouteImport(event) {
        // 首先尝试从服务器获取文件列表
        try {
            const response = await fetch('http://localhost:8081/route_files');
            
            if (!response.ok) {
                throw new Error(`HTTP错误: ${response.status}`);
            }
            
            const result = await response.json();
            
            if (result.success && result.files && result.files.length > 0) {
                // 显示文件选择对话框
                const filename = await this.showRouteFileSelector(result.files);
                if (filename) {
                    await this.loadRouteFromServer(filename);
                }
                // 清空文件输入
                if (event.target) {
                    event.target.value = '';
                }
                return;
            } else {
                // 如果没有服务器文件，提示用户使用本地文件
                console.log('服务器上没有路线文件，请选择本地文件');
            }
        } catch (error) {
            console.warn('从服务器获取路线文件列表失败，将使用本地文件:', error);
            // 如果服务器失败，继续尝试使用本地文件
        }
        
        // 使用本地文件选择
        const file = event.target && event.target.files && event.target.files[0];
        if (!file) {
            // 如果用户没有选择文件，提示用户
            alert('请选择一个路线配置文件（.json格式）');
            return;
        }
        
        try {
            const text = await file.text();
            const routeData = JSON.parse(text);
            this.importRoute(routeData);
        } catch (localError) {
            console.error('解析路线配置文件失败:', localError);
            alert('导入路线配置失败: ' + localError.message + '\n\n请确保文件格式正确。');
        }
        
        // 清空文件输入
        if (event.target) {
            event.target.value = '';
        }
    }
    
    /**
     * 显示路线文件选择器
     */
    showRouteFileSelector(files) {
        return new Promise((resolve) => {
            const fileList = files.map((f, i) => `${i + 1}. ${f}`).join('\n');
            const selection = prompt(`请选择要导入的路线文件（输入序号）:\n\n${fileList}\n\n输入0取消`);
            
            if (!selection || selection === '0') {
                resolve(null);
                return;
            }
            
            const index = parseInt(selection) - 1;
            if (index >= 0 && index < files.length) {
                resolve(files[index]);
            } else {
                alert('无效的选择');
                resolve(null);
            }
        });
    }
    
    /**
     * 从服务器加载路线文件
     */
    async loadRouteFromServer(filename) {
        try {
            const response = await fetch(`http://localhost:8081/route_file?name=${encodeURIComponent(filename)}`);
            
            if (!response.ok) {
                throw new Error(`HTTP错误: ${response.status} ${response.statusText}`);
            }
            
            const result = await response.json();
            
            if (result.success && result.data) {
                this.importRoute(result.data);
            } else {
                throw new Error(result.error || '加载失败');
            }
        } catch (error) {
            console.error('从服务器加载路线配置失败:', error);
            
            // 提供更友好的错误提示
            let errorMessage = '加载路线配置失败: ';
            if (error.message.includes('NetworkError') || error.message.includes('Failed to fetch')) {
                errorMessage += '无法连接到服务器（localhost:8081）。\n\n请确保ROS2服务代理正在运行。';
            } else {
                errorMessage += error.message;
            }
            
            alert(errorMessage);
            
            // 如果服务器加载失败，提示用户可以使用本地文件
            const useLocal = confirm(errorMessage + '\n\n是否要选择本地文件？');
            if (useLocal) {
                const fileInput = document.getElementById('routeFileInput');
                if (fileInput) {
                    fileInput.click();
                }
            }
        }
    }
    
    /**
     * 导入路线配置
     */
    importRoute(routeData) {
        // 验证数据格式
        if (!routeData.nodes || !Array.isArray(routeData.nodes)) {
            alert('无效的路线配置文件格式：缺少节点数据');
            return;
        }
        
        if (!routeData.connections || !Array.isArray(routeData.connections)) {
            alert('无效的路线配置文件格式：缺少连接数据');
            return;
        }
        
        // 清空当前画布
        this.nodes.clear();
        this.connections.clear();
        this.selectedNodeId = null;
        this.selectedConnectionId = null;
        this.connectionStartNodeId = null;
        this.connectionMode = false;
        
        // 重置计数器
        this.nodeIdCounter = 0;
        this.connectionIdCounter = 0;
        
        // 导入节点
        const nodeIdMap = new Map(); // 旧ID -> 新ID的映射
        routeData.nodes.forEach(nodeData => {
            const newNodeId = `node_${++this.nodeIdCounter}`;
            nodeIdMap.set(nodeData.id, newNodeId);
            
            const node = {
                id: newNodeId,
                name: nodeData.name || `站点 ${this.nodeIdCounter}`,
                x: nodeData.x || 0,
                y: nodeData.y || 0,
                type: nodeData.type || 'custom',
                markerId: nodeData.markerId || null, // APRILTAG标记ID
                position: nodeData.position || null,
                customProps: nodeData.customProps || {}
            };
            
            this.nodes.set(newNodeId, node);
        });
        
        // 导入连接
        routeData.connections.forEach(connData => {
            const newFromId = nodeIdMap.get(connData.fromNodeId);
            const newToId = nodeIdMap.get(connData.toNodeId);
            
            if (!newFromId || !newToId) {
                console.warn(`⚠️ 跳过无效连接: ${connData.fromNodeId} -> ${connData.toNodeId}`);
                return;
            }
            
            const connId = `conn_${++this.connectionIdCounter}`;
            const connection = {
                id: connId,
                fromNodeId: newFromId,
                toNodeId: newToId,
                waypointsFile: connData.waypointsFile || null,
                waypointsData: null // waypoints数据需要重新加载
            };
            
            this.connections.set(connId, connection);
        });
        
        // 导入已加载的文件列表（如果有）
        // 注意：导入时只保存了文件名，不保存完整数据
        // 如果连接关联了waypoints文件，需要从服务器或本地重新加载
        if (routeData.loadedFiles && Array.isArray(routeData.loadedFiles)) {
            this.loadedFiles = routeData.loadedFiles.map(file => ({
                name: file.name,
                data: null // 数据需要重新加载（从waypoints文件夹）
            }));
            
            // 尝试从服务器自动加载关联的waypoints文件
            this.loadWaypointsFilesForConnections(routeData.connections);
        }
        
        // 更新UI
        this.updatePropertiesPanel();
        this.updateConnectionPropertiesPanel();
        this.updateLoadedFilesList();
        
        // 重置视图并渲染
        this.resetView();
        
        console.log(`✅ 路线配置已导入: ${routeData.nodes.length} 个站点, ${routeData.connections.length} 条连接`);
        
        // 检查是否有需要加载的waypoints文件
        const connectionsWithWaypoints = Array.from(this.connections.values())
            .filter(conn => conn.waypointsFile && !conn.waypointsData);
        
        if (connectionsWithWaypoints.length > 0) {
            console.log(`📝 检测到 ${connectionsWithWaypoints.length} 个连接需要加载waypoints文件`);
            // 异步加载waypoints文件，不阻塞UI
            this.loadWaypointsFilesForConnections(routeData.connections).then(() => {
                this.updateLoadedFilesList();
            });
            alert(`路线配置已导入:\n- ${routeData.nodes.length} 个站点\n- ${routeData.connections.length} 条连接\n\n正在自动加载waypoints文件...`);
        } else {
            alert(`路线配置已导入:\n- ${routeData.nodes.length} 个站点\n- ${routeData.connections.length} 条连接`);
        }
    }
    
    /**
     * 为连接自动加载waypoints文件
     */
    async loadWaypointsFilesForConnections(connections) {
        if (!connections || !Array.isArray(connections)) return;
        
        // 收集所有需要加载的waypoints文件名
        const waypointsFiles = new Set();
        connections.forEach(conn => {
            if (conn.waypointsFile) {
                waypointsFiles.add(conn.waypointsFile);
            }
        });
        
        if (waypointsFiles.size === 0) return;
        
        console.log(`🔄 尝试自动加载 ${waypointsFiles.size} 个waypoints文件...`);
        
        let loadedCount = 0;
        let failedCount = 0;
        
        // 尝试从服务器加载每个waypoints文件
        for (const filename of waypointsFiles) {
            try {
                const response = await fetch(`http://localhost:8081/waypoint_file?name=${encodeURIComponent(filename)}`);
                
                if (response.ok) {
                    const result = await response.json();
                    if (result.success && result.data) {
                        // 添加到已加载文件列表
                        const existingFile = this.loadedFiles.find(f => f.name === filename);
                        if (existingFile) {
                            existingFile.data = result.data;
                        } else {
                            this.loadedFiles.push({
                                name: filename,
                                data: result.data
                            });
                        }
                        
                        // 更新所有使用该文件的连接
                        for (const connection of this.connections.values()) {
                            if (connection.waypointsFile === filename) {
                                connection.waypointsData = result.data;
                            }
                        }
                        
                        loadedCount++;
                        console.log(`✅ 自动加载waypoints文件成功: ${filename}`);
                    } else {
                        failedCount++;
                        console.warn(`⚠️ 服务器返回失败: ${filename}`, result.error);
                    }
                } else {
                    failedCount++;
                    console.warn(`⚠️ HTTP错误 ${response.status}: ${filename}`);
                }
            } catch (error) {
                failedCount++;
                console.warn(`⚠️ 自动加载waypoints文件失败: ${filename}`, error);
                // 继续加载其他文件，不中断
            }
        }
        
        console.log(`📊 自动加载完成: ${loadedCount} 成功, ${failedCount} 失败`);
        
        // 更新已加载文件列表
        this.updateLoadedFilesList();
        
        return { loadedCount, failedCount };
    }
    
    /**
     * HTML转义
     */
    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }
}

// 创建全局实例
window.routePlanner = new RoutePlanner();

// 自动初始化
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
        window.routePlanner.initialize();
    });
} else {
    window.routePlanner.initialize();
}

