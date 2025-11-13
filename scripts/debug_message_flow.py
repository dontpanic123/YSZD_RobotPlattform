#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import time

class MessageFlowDebugger(Node):
    """调试消息流程"""
    
    def __init__(self):
        super().__init__('message_flow_debugger')
        
        # 创建发布者
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 创建订阅者
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10
        )
        
        self.current_state = "unknown"
        self.test_start_time = time.time()
        self.test_phase = 0  # 0: 等待空闲, 1: 测试紧急停止, 2: 测试导航, 3: 完成
        
        self.get_logger().info('🔍 消息流程调试器启动')
        self.get_logger().info('📋 将测试紧急停止和自动导航消息')
        
        # 开始测试
        self.create_timer(2.0, self.test_timer)
        
    def state_callback(self, msg):
        """状态回调"""
        if msg.data != self.current_state:
            self.current_state = msg.data
            self.get_logger().info(f'🔄 状态变化: {self.current_state}')
    
    def test_timer(self):
        """测试定时器"""
        if self.test_phase == 0:
            # 测试紧急停止
            self.get_logger().info('📤 发送紧急停止信号...')
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)
            self.test_phase = 1
            self.test_start_time = time.time()
            
        elif self.test_phase == 1:
            elapsed = time.time() - self.test_start_time
            if elapsed > 3.0:  # 等待3秒
                # 解除紧急停止
                self.get_logger().info('📤 解除紧急停止...')
                emergency_msg = Bool()
                emergency_msg.data = False
                self.emergency_stop_pub.publish(emergency_msg)
                self.test_phase = 2
                self.test_start_time = time.time()
                
        elif self.test_phase == 2:
            elapsed = time.time() - self.test_start_time
            if elapsed > 2.0:  # 等待2秒
                # 测试自动导航
                self.get_logger().info('📤 发送导航目标点...')
                goal = PoseStamped()
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.header.frame_id = 'map'
                goal.pose.position.x = 2.0
                goal.pose.position.y = 1.0
                goal.pose.position.z = 0.0
                goal.pose.orientation.w = 1.0
                self.goal_pose_pub.publish(goal)
                self.test_phase = 3
                self.test_start_time = time.time()
                
        elif self.test_phase == 3:
            elapsed = time.time() - self.test_start_time
            if elapsed > 5.0:  # 等待5秒
                self.get_logger().info('🎉 测试完成!')
                self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        debugger = MessageFlowDebugger()
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        pass
    finally:
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






