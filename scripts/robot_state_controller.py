#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
import time

class RobotStateController(Node):
    """机器人状态控制器 - 用于测试和调试状态机"""
    
    def __init__(self):
        super().__init__('robot_state_controller')
        
        # 创建发布者
        self.manual_cmd_pub = self.create_publisher(Twist, '/manual_cmd_vel', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.charging_pub = self.create_publisher(Bool, '/charging_status', 10)
        
        # 创建订阅者
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, '/robot_status', self.status_callback, 10
        )
        
        # 状态跟踪
        self.current_state = "unknown"
        self.status_info = {}
        
        # 创建定时器用于测试
        self.test_timer = self.create_timer(5.0, self.run_test_sequence)
        self.test_step = 0
        
        self.get_logger().info('🎮 机器人状态控制器已启动')
        self.get_logger().info('📋 可用命令:')
        self.get_logger().info('  - 手动控制: 发送速度命令到 /manual_cmd_vel')
        self.get_logger().info('  - 自动导航: 发送目标点到 /goal_pose')
        self.get_logger().info('  - 紧急停止: 发送True到 /emergency_stop')
        self.get_logger().info('  - 充电状态: 发送True到 /charging_status')
    
    def state_callback(self, msg):
        """状态回调"""
        if self.current_state != msg.data:
            self.current_state = msg.data
            self.get_logger().info(f'📊 机器人状态: {self.current_state}')
    
    def status_callback(self, msg):
        """状态信息回调"""
        try:
            self.status_info = eval(msg.data)
            self.get_logger().debug(f'📈 状态信息: {self.status_info}')
        except:
            pass
    
    def run_test_sequence(self):
        """运行测试序列"""
        if self.test_step == 0:
            self.get_logger().info('🧪 开始状态机测试序列...')
            self.test_step += 1
        
        elif self.test_step == 1:
            # 测试手动控制
            self.get_logger().info('🎮 测试手动控制...')
            cmd = Twist()
            cmd.linear.x = 0.1
            self.manual_cmd_pub.publish(cmd)
            self.test_step += 1
        
        elif self.test_step == 2:
            # 停止手动控制
            self.get_logger().info('⏸️ 停止手动控制...')
            cmd = Twist()
            self.manual_cmd_pub.publish(cmd)
            self.test_step += 1
        
        elif self.test_step == 3:
            # 测试自动导航
            self.get_logger().info('🧭 测试自动导航...')
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = 1.0
            goal.pose.position.y = 1.0
            goal.pose.orientation.w = 1.0
            self.goal_pose_pub.publish(goal)
            self.test_step += 1
        
        elif self.test_step == 4:
            # 测试紧急停止
            self.get_logger().info('🚨 测试紧急停止...')
            emergency = Bool()
            emergency.data = True
            self.emergency_stop_pub.publish(emergency)
            self.test_step += 1
        
        elif self.test_step == 5:
            # 取消紧急停止
            self.get_logger().info('✅ 取消紧急停止...')
            emergency = Bool()
            emergency.data = False
            self.emergency_stop_pub.publish(emergency)
            self.test_step += 1
        
        elif self.test_step == 6:
            # 测试充电状态
            self.get_logger().info('🔋 测试充电状态...')
            charging = Bool()
            charging.data = True
            self.charging_pub.publish(charging)
            self.test_step += 1
        
        elif self.test_step == 7:
            # 完成充电
            self.get_logger().info('🔋 完成充电...')
            charging = Bool()
            charging.data = False
            self.charging_pub.publish(charging)
            self.test_step += 1
        
        else:
            self.get_logger().info('✅ 测试序列完成')
            self.test_timer.cancel()
    
    def send_manual_command(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """发送手动控制命令"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.angular.z = angular_z
        self.manual_cmd_pub.publish(cmd)
        self.get_logger().info(f'🎮 发送手动命令: 线性({linear_x}, {linear_y}), 角速度({angular_z})')
    
    def send_navigation_goal(self, x, y, z=0.0):
        """发送导航目标"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self.goal_pose_pub.publish(goal)
        self.get_logger().info(f'🧭 发送导航目标: ({x}, {y}, {z})')
    
    def trigger_emergency_stop(self, active=True):
        """触发紧急停止"""
        emergency = Bool()
        emergency.data = active
        self.emergency_stop_pub.publish(emergency)
        status = "激活" if active else "取消"
        self.get_logger().info(f'🚨 紧急停止{status}')
    
    def set_charging_status(self, charging=True):
        """设置充电状态"""
        charging_msg = Bool()
        charging_msg.data = charging
        self.charging_pub.publish(charging_msg)
        status = "开始" if charging else "完成"
        self.get_logger().info(f'🔋 充电{status}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = RobotStateController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
