#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class ManualControlFlowTest(Node):
    """测试手动控制流程"""
    
    def __init__(self):
        super().__init__('manual_control_flow_test')
        
        # 创建发布者
        self.manual_cmd_vel_pub = self.create_publisher(Twist, '/manual_cmd_vel', 10)
        
        # 创建订阅者
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10
        )
        
        self.current_state = "unknown"
        self.test_start_time = time.time()
        self.test_phase = 0  # 0: 等待空闲, 1: 发送指令, 2: 等待状态变化, 3: 完成
        
        self.get_logger().info('🧪 手动控制流程测试开始')
        self.get_logger().info('📋 测试步骤:')
        self.get_logger().info('   1. 等待机器人进入空闲状态')
        self.get_logger().info('   2. 发送运动指令到 /manual_cmd_vel')
        self.get_logger().info('   3. 检查状态是否变为 manual_control')
        self.get_logger().info('   4. 等待1.5秒后检查是否返回空闲')
        
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
            self.manual_cmd_vel_pub.publish(twist)
            self.get_logger().info('📤 发送运动指令到 /manual_cmd_vel')
            
            elapsed = time.time() - self.test_start_time
            if elapsed > 0.5:  # 发送0.5秒后停止
                self.get_logger().info('⏹️ 停止发送运动指令，等待状态变化')
                self.test_phase = 2
                self.test_start_time = time.time()
        
        elif self.test_phase == 2:
            elapsed = time.time() - self.test_start_time
            self.get_logger().info(f'⏱️ 等待状态变化: {elapsed:.1f}秒')
            
            if elapsed > 5.0:  # 超过5秒还没变化，可能有问题
                self.get_logger().warn('⚠️ 测试超时，可能有问题')
                self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = ManualControlFlowTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






