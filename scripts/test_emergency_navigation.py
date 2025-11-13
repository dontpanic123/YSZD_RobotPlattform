#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time

class EmergencyNavigationTest(Node):
    """测试紧急停止和自动导航功能"""
    
    def __init__(self):
        super().__init__('emergency_navigation_test')
        
        # 创建发布者
        self.emergency_stop_pub = self.create_publisher(String, '/emergency_stop', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 创建订阅者
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10
        )
        
        self.current_state = "unknown"
        self.test_start_time = time.time()
        self.test_phase = 0  # 0: 等待空闲, 1: 测试导航, 2: 测试紧急停止, 3: 完成
        
        self.get_logger().info('🧪 紧急停止和自动导航测试开始')
        self.get_logger().info('📋 测试步骤:')
        self.get_logger().info('   1. 等待机器人进入空闲状态')
        self.get_logger().info('   2. 发送导航目标点 -> 应该进入自动导航状态')
        self.get_logger().info('   3. 发送紧急停止信号 -> 应该进入紧急停止状态')
        self.get_logger().info('   4. 解除紧急停止 -> 应该返回空闲状态')
        
        # 开始测试
        self.create_timer(0.1, self.test_timer)
        
    def state_callback(self, msg):
        """状态回调"""
        if msg.data != self.current_state:
            self.current_state = msg.data
            self.get_logger().info(f'🔄 状态变化: {self.current_state}')
            
            if self.current_state == 'idle' and self.test_phase == 0:
                self.get_logger().info('✅ 机器人进入空闲状态，开始测试自动导航')
                self.test_phase = 1
                self.test_start_time = time.time()
            elif self.current_state == 'auto_navigation' and self.test_phase == 1:
                self.get_logger().info('✅ 成功进入自动导航状态')
                self.test_phase = 2
                self.test_start_time = time.time()
            elif self.current_state == 'emergency_stop' and self.test_phase == 2:
                self.get_logger().info('✅ 成功进入紧急停止状态')
                self.test_phase = 3
                self.test_start_time = time.time()
            elif self.current_state == 'idle' and self.test_phase == 3:
                elapsed = time.time() - self.test_start_time
                if elapsed >= 1.0:  # 至少1秒后
                    self.get_logger().info('✅ 成功解除紧急停止返回空闲状态')
                    self.get_logger().info('🎉 测试完成!')
                    self.destroy_node()
                    return
    
    def test_timer(self):
        """测试定时器"""
        if self.test_phase == 1:
            # 发送导航目标点
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x = 2.0
            goal.pose.position.y = 1.0
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            
            self.goal_pose_pub.publish(goal)
            self.get_logger().info('📤 发送导航目标点')
            
            elapsed = time.time() - self.test_start_time
            if elapsed > 2.0:  # 等待2秒
                self.test_phase = 2
                self.test_start_time = time.time()
        
        elif self.test_phase == 2:
            # 发送紧急停止信号
            emergency_msg = String()
            emergency_msg.data = 'true'
            self.emergency_stop_pub.publish(emergency_msg)
            self.get_logger().info('📤 发送紧急停止信号')
            
            elapsed = time.time() - self.test_start_time
            if elapsed > 2.0:  # 等待2秒
                self.test_phase = 3
                self.test_start_time = time.time()
        
        elif self.test_phase == 3:
            # 解除紧急停止
            emergency_msg = String()
            emergency_msg.data = 'false'
            self.emergency_stop_pub.publish(emergency_msg)
            self.get_logger().info('📤 解除紧急停止')
            
            elapsed = time.time() - self.test_start_time
            if elapsed > 2.0:  # 等待2秒
                self.get_logger().warn('⚠️ 测试超时，可能有问题')
                self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = EmergencyNavigationTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






