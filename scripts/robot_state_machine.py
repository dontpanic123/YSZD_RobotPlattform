#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
import time
import threading
import json
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
        
        # 当前定位状态
        self.current_location = "未定位"  # 默认状态为未定位
        self.last_apriltag_detection_time = 0.0  # 最后检测到AprilTag的时间戳
        
        # 手动控制保持时间设置
        self.declare_parameter('manual_control_hold_time', 1.5)  # 默认1.5秒保持时间
        self.manual_control_hold_time = self.get_parameter('manual_control_hold_time').value
        self.manual_control_start_time = 0.0
        
        # 创建发布者
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.location_pub = self.create_publisher(String, '/robot_location', 10)  # 发布定位状态
        
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
        self.waypoint_status_sub = self.create_subscription(
            String, '/waypoint_following_status', self.waypoint_status_callback, 10
        )
        self.charging_sub = self.create_subscription(
            Bool, '/charging_status', self.charging_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odometry_callback, 10
        )
        
        # TF监听器，用于从TF变换中获取AprilTag ID
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建定时器
        self.state_timer = self.create_timer(0.1, self.state_machine_update)
        self.publish_timer = self.create_timer(1.0, self.publish_state)
        self.tf_check_timer = self.create_timer(0.5, self.check_apriltag_tf)  # 检查AprilTag TF变换
        
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
        self.get_logger().info(f'📍 当前定位: {self.current_location}')
        self.get_logger().info(f'⏰ AprilTag跟踪超时: {self.apriltag_timeout}秒')
        self.get_logger().info(f'🎮 手动控制保持时间: {self.manual_control_hold_time}秒')
        self.get_logger().info(f'📡 订阅话题: /apriltag_pose')
        self.get_logger().info(f'📡 发布话题: /robot_location')
        
    def manual_control_callback(self, msg):
        """手动控制回调"""
        # 检查是否有非零速度命令
        has_velocity = (abs(msg.linear.x) > 0.001 or abs(msg.linear.y) > 0.001 or 
                       abs(msg.linear.z) > 0.001 or abs(msg.angular.x) > 0.001 or 
                       abs(msg.angular.y) > 0.001 or abs(msg.angular.z) > 0.001)
        
        if has_velocity:
            # 如果当前是空闲状态且收到运动指令，记录开始时间
            if self.current_state == RobotState.IDLE and not self.manual_control_active:
                self.manual_control_start_time = time.time()
                self.get_logger().info(f'🎮 收到运动指令，开始手动控制保持 {self.manual_control_hold_time} 秒')
            
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
        self.get_logger().info('🔔 收到AprilTag位姿消息！')
        self.apriltag_tracking_active = True
        self.apriltag_pose = msg
        self.last_apriltag_time = time.time()  # 记录最后收到AprilTag的时间
        
        # 从frame_id中提取tag_id
        frame_id = msg.header.frame_id
        self.get_logger().info(f'📋 frame_id: {frame_id}')
        tag_id = None
        
        try:
            # frame_id格式可能是 'apriltag_{id}' 或 'camera_link'
            if 'apriltag_' in frame_id:
                tag_id_str = frame_id.split('apriltag_')[-1]
                tag_id = int(tag_id_str)
                self.get_logger().info(f'✅ 从frame_id提取tag_id: {tag_id} (frame_id: {frame_id})')
            elif frame_id.isdigit():
                tag_id = int(frame_id)
                self.get_logger().info(f'✅ 从frame_id提取tag_id: {tag_id} (frame_id: {frame_id})')
            else:
                # 如果frame_id是'camera_link'，尝试从TF变换中获取
                self.get_logger().info(f'⚠️ frame_id是 {frame_id}，无法直接提取tag_id，将尝试从TF变换获取')
        except (ValueError, AttributeError) as e:
            self.get_logger().warn(f'⚠️ 无法从frame_id提取tag_id: {frame_id}, 错误: {e}')
        
        # 如果从frame_id无法提取，尝试从TF变换中获取
        if tag_id is None:
            self.get_logger().info('🔍 尝试从TF变换中获取tag_id...')
            tag_id = self.get_tag_id_from_tf()
            if tag_id is not None:
                self.get_logger().info(f'✅ 从TF变换获取到tag_id: {tag_id}')
            else:
                self.get_logger().warn('❌ 无法从TF变换获取tag_id')
        
        # 根据tag_id更新定位状态
        if tag_id is not None:
            old_location = self.current_location
            if tag_id == 0:
                self.current_location = "充电桩"
            elif tag_id == 1:
                self.current_location = "装载点"
            elif tag_id == 2:
                self.current_location = "送达点"
            else:
                # 其他ID保持当前定位或设置为未知
                self.current_location = f"未知位置(ID:{tag_id})"
            
            # 更新最后检测时间
            self.last_apriltag_detection_time = time.time()
            
            # 如果定位状态发生变化，记录日志
            if old_location != self.current_location:
                self.get_logger().info(f'📍 定位更新: {old_location} -> {self.current_location} (AprilTag ID: {tag_id})')
            else:
                self.get_logger().info(f'📍 定位状态保持不变: {self.current_location} (AprilTag ID: {tag_id})')
        else:
            self.get_logger().warn(f'⚠️ 无法提取AprilTag ID，frame_id: {frame_id}')
        
        self.get_logger().info(f'🏷️ 收到AprilTag位姿: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}), 当前定位: {self.current_location}')
    
    def get_tag_id_from_tf(self):
        """从TF变换中获取AprilTag ID"""
        try:
            # 检查所有可能的AprilTag TF变换
            # AprilTag检测器通常发布从camera_link到apriltag_{id}的变换
            for tag_id in range(10):  # 检查ID 0-9
                child_frame_id = f'apriltag_{tag_id}'
                try:
                    # 尝试查找TF变换：从camera_link到apriltag_{id}
                    # 使用now()时间点，但允许一定的时间容差
                    now = self.get_clock().now()
                    transform = self.tf_buffer.lookup_transform(
                        'camera_link',
                        child_frame_id,
                        now,
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    # 如果找到了TF变换，说明这个tag_id存在
                    self.get_logger().info(f'✅ 从TF变换找到AprilTag ID: {tag_id} (child_frame_id: {child_frame_id})')
                    return tag_id
                except Exception as e:
                    # 这个tag_id不存在，继续检查下一个
                    continue
        except Exception as e:
            self.get_logger().warn(f'从TF变换获取tag_id失败: {e}')
        
        return None
    
    def check_apriltag_tf(self):
        """定期检查AprilTag TF变换，更新定位状态"""
        tag_id = self.get_tag_id_from_tf()
        if tag_id is not None:
            old_location = self.current_location
            if tag_id == 0:
                self.current_location = "充电桩"
            elif tag_id == 1:
                self.current_location = "装载点"
            elif tag_id == 2:
                self.current_location = "送达点"
            else:
                self.current_location = f"未知位置(ID:{tag_id})"
            
            # 更新最后检测时间
            self.last_apriltag_detection_time = time.time()
            
            if old_location != self.current_location:
                self.get_logger().info(f'📍 从TF变换更新定位: {old_location} -> {self.current_location} (AprilTag ID: {tag_id})')
    
    def waypoint_status_callback(self, msg):
        """Waypoint状态回调"""
        status = msg.data
        if status == 'completed':
            self.get_logger().info('✅ Waypoint跟踪已完成，切换到空闲状态')
            self.auto_navigation_active = False
        elif status == 'stopped':
            self.get_logger().info('⏹️ Waypoint跟踪已停止，切换到空闲状态')
            self.auto_navigation_active = False
    
    def emergency_stop_callback(self, msg):
        """紧急停止回调"""
        previous_state = self.emergency_stop_active
        self.emergency_stop_active = msg.data
        
        # 只在状态变化时记录日志，避免重复日志
        if self.emergency_stop_active and not previous_state:
            self.get_logger().warn('🚨 紧急停止激活!')
            # 紧急停止时重置导航状态
            self.auto_navigation_active = False
            self.apriltag_tracking_active = False
        elif not self.emergency_stop_active and previous_state:
            self.get_logger().info('✅ 紧急停止已解除，重置所有状态')
            # 解除紧急停止时重置所有活动状态，确保回到空闲状态
            self.auto_navigation_active = False
            self.manual_control_active = False
            self.apriltag_tracking_active = False
    
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
            # 检查手动控制保持时间
            if self.manual_control_start_time > 0:
                elapsed_time = time.time() - self.manual_control_start_time
                if elapsed_time >= self.manual_control_hold_time:
                    self.get_logger().info(f'⏰ 手动控制保持时间到达 ({self.manual_control_hold_time}秒)，返回空闲状态')
                    self.manual_control_active = False
                    self.manual_control_start_time = 0.0
                    return RobotState.IDLE
            
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
            'current_location': self.current_location,  # 添加当前定位
            'manual_control': self.manual_control_active,
            'auto_navigation': self.auto_navigation_active,
            'apriltag_tracking': self.apriltag_tracking_active,
            'emergency_stop': self.emergency_stop_active,
            'charging': self.charging_active,
            'error': self.error_condition
        }
        status_msg.data = str(status_info)
        self.status_pub.publish(status_msg)
        
        # 发布定位状态（包含时间戳）
        location_data = {
            'location': self.current_location,
            'last_detection_time': self.last_apriltag_detection_time,
            'current_time': time.time()
        }
        location_msg = String()
        location_msg.data = json.dumps(location_data)
        self.location_pub.publish(location_msg)
    
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
