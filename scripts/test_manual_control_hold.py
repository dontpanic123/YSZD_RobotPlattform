#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class ManualControlHoldTest(Node):
    """测试手动控制保持功能"""
    
    def __init__(self):
        super().__init__('manual_control_hold_test')
        
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/manual_cmd_vel', 10)
        
        # 创建订阅者
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10
        )
        
        self.current_state = "unknown"
        self.test_start_time = time.time()
        self.test_phase = 0  # 0: 等待空闲, 1: 发送指令, 2: 等待保持, 3: 完成
        
        self.get_logger().info('🧪 手动控制保持测试开始')
        self.get_logger().info('📋 测试步骤:')
        self.get_logger().info('   1. 等待机器人进入空闲状态')
        self.get_logger().info('   2. 发送运动指令 -> 应该进入手动控制状态')
        self.get_logger().info('   3. 等待1.5秒 -> 应该自动返回空闲状态')
        
        # 开始测试
        self.create_timer(0.1, self.test_timer)
        
    def state_callback(self, msg):
        """状态回调"""
        if msg.data != self.current_state:
            self.current_state = msg.data
            self.get_logger().info(f'🔄 状态变化: {self.current_state}')
            
            if self.current_state == 'idle' and self.test_phase == 0:
                self.get_logger().info('✅ 机器人进入空闲状态，开始发送运动指令')
                self.test_phase = 1
                self.test_start_time = time.time()
            elif self.current_state == 'manual_control' and self.test_phase == 1:
                self.get_logger().info('✅ 成功进入手动控制状态')
                self.test_phase = 2
                self.test_start_time = time.time()
            elif self.current_state == 'idle' and self.test_phase == 2:
                elapsed = time.time() - self.test_start_time
                if elapsed >= 1.0:  # 至少1秒后
                    self.get_logger().info('✅ 成功保持1.5秒后返回空闲状态')
                    self.get_logger().info('🎉 测试完成!')
                    self.test_phase = 3
                    self.destroy_node()
                    return
    
    def test_timer(self):
        """测试定时器"""
        if self.test_phase == 1:
            # 发送运动指令
            twist = Twist()
            twist.linear.x = 0.1  # 向前移动
            self.cmd_vel_pub.publish(twist)
            
            elapsed = time.time() - self.test_start_time
            if elapsed > 0.5:  # 发送0.5秒后停止
                self.get_logger().info('⏹️ 停止发送运动指令，等待保持时间')
                self.test_phase = 2
                self.test_start_time = time.time()
        
        elif self.test_phase == 2:
            elapsed = time.time() - self.test_start_time
            self.get_logger().info(f'⏱️ 等待保持时间: {elapsed:.1f}秒')
            
            if elapsed > 3.0:  # 超过3秒还没返回空闲，可能有问题
                self.get_logger().warn('⚠️ 测试超时，可能有问题')
                self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = ManualControlHoldTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



