#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import time
import threading
from enum import Enum

class RobotState(Enum):
    """机器人状态枚举"""
    IDLE = "idle"                    # 空闲状态
    MANUAL_CONTROL = "manual_control"  # 手动控制
    AUTO_NAVIGATION = "auto_navigation"  # 自动导航
    APRILTAG_TRACKING = "apriltag_tracking"  # AprilTag跟踪
    EMERGENCY_STOP = "emergency_stop"  # 紧急停止
    CHARGING = "charging"            # 充电中
    ERROR = "error"                  # 错误状态

class RobotStateMachine(Node):
    """机器人状态机节点"""
    
    def __init__(self):
        super().__init__('robot_state_machine')
        
        # 状态机参数
        self.current_state = RobotState.IDLE
        self.previous_state = RobotState.IDLE
        self.state_start_time = time.time()
        self.state_timeout = 30.0  # 状态超时时间（秒）
        
        # 状态标志
        self.manual_control_active = False
        self.auto_navigation_active = False
        self.apriltag_tracking_active = False
        self.emergency_stop_active = False
        self.charging_active = False
        self.error_condition = False
        
        # AprilTag跟踪超时设置
        self.declare_parameter('apriltag_timeout', 60.0)  # 默认60秒超时
        self.apriltag_timeout = self.get_parameter('apriltag_timeout').value
        self.last_apriltag_time = 0.0
        
        # 创建发布者
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # 创建订阅者
        self.manual_control_sub = self.create_subscription(
            Twist, '/manual_cmd_vel', self.manual_control_callback, 10
        )
        self.auto_nav_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.auto_navigation_callback, 10
        )
        self.apriltag_pose_sub = self.create_subscription(
            PoseStamped, '/apriltag_pose', self.apriltag_pose_callback, 10
        )
        self.emergency_stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_stop_callback, 10
        )
        self.charging_sub = self.create_subscription(
            Bool, '/charging_status', self.charging_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odometry_callback, 10
        )
        
        # 创建定时器
        self.state_timer = self.create_timer(0.1, self.state_machine_update)
        self.publish_timer = self.create_timer(1.0, self.publish_state)
        
        # 状态转换表
        self.state_transitions = {
            RobotState.IDLE: [RobotState.MANUAL_CONTROL, RobotState.AUTO_NAVIGATION, 
                             RobotState.APRILTAG_TRACKING, RobotState.EMERGENCY_STOP, RobotState.CHARGING],
            RobotState.MANUAL_CONTROL: [RobotState.IDLE, RobotState.EMERGENCY_STOP, RobotState.ERROR],
            RobotState.AUTO_NAVIGATION: [RobotState.IDLE, RobotState.EMERGENCY_STOP, RobotState.ERROR],
            RobotState.APRILTAG_TRACKING: [RobotState.IDLE, RobotState.EMERGENCY_STOP, RobotState.ERROR],
            RobotState.EMERGENCY_STOP: [RobotState.IDLE, RobotState.ERROR],
            RobotState.CHARGING: [RobotState.IDLE, RobotState.EMERGENCY_STOP],
            RobotState.ERROR: [RobotState.IDLE, RobotState.EMERGENCY_STOP]
        }
        
        self.get_logger().info('🤖 机器人状态机节点已启动')
        self.get_logger().info(f'📊 当前状态: {self.current_state.value}')
        self.get_logger().info(f'⏰ AprilTag跟踪超时: {self.apriltag_timeout}秒')
        
    def manual_control_callback(self, msg):
        """手动控制回调"""
        # 检查是否有非零速度命令
        has_velocity = (abs(msg.linear.x) > 0.001 or abs(msg.linear.y) > 0.001 or 
                       abs(msg.linear.z) > 0.001 or abs(msg.angular.x) > 0.001 or 
                       abs(msg.angular.y) > 0.001 or abs(msg.angular.z) > 0.001)
        
        if has_velocity:
            self.manual_control_active = True
            self.manual_control_time = time.time()
        else:
            # 检查手动控制超时
            if hasattr(self, 'manual_control_time'):
                if time.time() - self.manual_control_time > 2.0:  # 2秒无命令则停用手动控制
                    self.manual_control_active = False
    
    def auto_navigation_callback(self, msg):
        """自动导航回调"""
        self.auto_navigation_active = True
        self.auto_nav_goal = msg
        self.get_logger().info(f'🎯 收到导航目标: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
    
    def apriltag_pose_callback(self, msg):
        """AprilTag位姿回调"""
        self.apriltag_tracking_active = True
        self.apriltag_pose = msg
        self.last_apriltag_time = time.time()  # 记录最后收到AprilTag的时间
        self.get_logger().debug(f'🏷️ 收到AprilTag位姿: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
    
    def emergency_stop_callback(self, msg):
        """紧急停止回调"""
        self.emergency_stop_active = msg.data
        if self.emergency_stop_active:
            self.get_logger().warn('🚨 紧急停止激活!')
    
    def charging_callback(self, msg):
        """充电状态回调"""
        self.charging_active = msg.data
        if self.charging_active:
            self.get_logger().info('🔋 开始充电')
        else:
            self.get_logger().info('🔋 充电完成')
    
    def odometry_callback(self, msg):
        """里程计回调"""
        self.current_odom = msg
        # 可以在这里添加位置检查逻辑
    
    def state_machine_update(self):
        """状态机更新"""
        new_state = self.determine_next_state()
        
        if new_state != self.current_state:
            self.transition_to_state(new_state)
    
    def determine_next_state(self):
        """确定下一个状态"""
        # 紧急停止优先级最高
        if self.emergency_stop_active:
            return RobotState.EMERGENCY_STOP
        
        # 错误状态检查
        if self.error_condition:
            return RobotState.ERROR
        
        # 充电状态
        if self.charging_active:
            return RobotState.CHARGING
        
        # 根据当前状态和输入确定下一个状态
        if self.current_state == RobotState.IDLE:
            if self.manual_control_active:
                return RobotState.MANUAL_CONTROL
            elif self.auto_navigation_active:
                return RobotState.AUTO_NAVIGATION
            elif self.apriltag_tracking_active:
                return RobotState.APRILTAG_TRACKING
        
        elif self.current_state == RobotState.MANUAL_CONTROL:
            if not self.manual_control_active:
                return RobotState.IDLE
        
        elif self.current_state == RobotState.AUTO_NAVIGATION:
            if not self.auto_navigation_active:
                return RobotState.IDLE
        
        elif self.current_state == RobotState.APRILTAG_TRACKING:
            # 检查AprilTag跟踪超时
            if self.last_apriltag_time > 0 and (time.time() - self.last_apriltag_time) > self.apriltag_timeout:
                self.get_logger().info(f'⏰ AprilTag跟踪超时 ({self.apriltag_timeout}秒)，返回空闲状态')
                self.apriltag_tracking_active = False
                return RobotState.IDLE
            elif not self.apriltag_tracking_active:
                return RobotState.IDLE
        
        elif self.current_state == RobotState.EMERGENCY_STOP:
            if not self.emergency_stop_active:
                return RobotState.IDLE
        
        elif self.current_state == RobotState.CHARGING:
            if not self.charging_active:
                return RobotState.IDLE
        
        elif self.current_state == RobotState.ERROR:
            # 错误状态需要手动重置
            pass
        
        return self.current_state
    
    def transition_to_state(self, new_state):
        """状态转换"""
        if new_state in self.state_transitions.get(self.current_state, []):
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_start_time = time.time()
            
            self.get_logger().info(f'🔄 状态转换: {self.previous_state.value} -> {self.current_state.value}')
            self.execute_state_entry_actions()
        else:
            self.get_logger().warn(f'⚠️ 无效的状态转换: {self.current_state.value} -> {new_state.value}')
    
    def execute_state_entry_actions(self):
        """执行状态进入动作"""
        if self.current_state == RobotState.IDLE:
            self.stop_robot()
            self.get_logger().info('⏸️ 机器人进入空闲状态')
        
        elif self.current_state == RobotState.MANUAL_CONTROL:
            self.get_logger().info('🎮 机器人进入手动控制模式')
        
        elif self.current_state == RobotState.AUTO_NAVIGATION:
            self.get_logger().info('🧭 机器人进入自动导航模式')
        
        elif self.current_state == RobotState.APRILTAG_TRACKING:
            self.get_logger().info('🏷️ 机器人进入AprilTag跟踪模式')
        
        elif self.current_state == RobotState.EMERGENCY_STOP:
            self.stop_robot()
            self.get_logger().warn('🚨 机器人紧急停止!')
        
        elif self.current_state == RobotState.CHARGING:
            self.stop_robot()
            self.get_logger().info('🔋 机器人进入充电状态')
        
        elif self.current_state == RobotState.ERROR:
            self.stop_robot()
            self.get_logger().error('❌ 机器人进入错误状态')
    
    def stop_robot(self):
        """停止机器人"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
    
    def publish_state(self):
        """发布当前状态"""
        state_msg = String()
        state_msg.data = self.current_state.value
        self.state_pub.publish(state_msg)
        
        # 发布状态信息
        status_msg = String()
        status_info = {
            'state': self.current_state.value,
            'previous_state': self.previous_state.value,
            'state_duration': f'{time.time() - self.state_start_time:.1f}s',
            'manual_control': self.manual_control_active,
            'auto_navigation': self.auto_navigation_active,
            'apriltag_tracking': self.apriltag_tracking_active,
            'emergency_stop': self.emergency_stop_active,
            'charging': self.charging_active,
            'error': self.error_condition
        }
        status_msg.data = str(status_info)
        self.status_pub.publish(status_msg)
    
    def reset_error_state(self):
        """重置错误状态"""
        if self.current_state == RobotState.ERROR:
            self.error_condition = False
            self.transition_to_state(RobotState.IDLE)
            self.get_logger().info('✅ 错误状态已重置')
    
    def force_state_change(self, new_state_str):
        """强制状态转换（用于调试）"""
        try:
            new_state = RobotState(new_state_str)
            if new_state in self.state_transitions.get(self.current_state, []):
                self.transition_to_state(new_state)
                return True
            else:
                self.get_logger().warn(f'⚠️ 无法强制转换到状态: {new_state_str}')
                return False
        except ValueError:
            self.get_logger().error(f'❌ 无效的状态名称: {new_state_str}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        state_machine = RobotStateMachine()
        rclpy.spin(state_machine)
    except KeyboardInterrupt:
        pass
    finally:
        state_machine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
