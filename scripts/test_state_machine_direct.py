#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time

class StateMachineDirectTest(Node):
    """直接测试状态机"""
    
    def __init__(self):
        super().__init__('state_machine_direct_test')
        
        # 创建发布者
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # 创建订阅者
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10
        )
        
        self.current_state = "unknown"
        self.test_start_time = time.time()
        
        self.get_logger().info('🧪 直接测试状态机')
        self.get_logger().info('📋 将发送紧急停止信号并监控状态变化')
        
        # 等待2秒后发送紧急停止
        self.create_timer(2.0, self.send_emergency_stop)
        
    def state_callback(self, msg):
        """状态回调"""
        if msg.data != self.current_state:
            self.current_state = msg.data
            self.get_logger().info(f'🔄 状态变化: {self.current_state}')
            
            if self.current_state == 'emergency_stop':
                self.get_logger().info('✅ 成功进入紧急停止状态')
                # 5秒后解除紧急停止
                self.create_timer(5.0, self.clear_emergency_stop)
    
    def send_emergency_stop(self):
        """发送紧急停止"""
        self.get_logger().info('📤 发送紧急停止信号...')
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)
        self.get_logger().info('✅ 紧急停止信号已发送')
    
    def clear_emergency_stop(self):
        """解除紧急停止"""
        self.get_logger().info('📤 解除紧急停止...')
        emergency_msg = Bool()
        emergency_msg.data = False
        self.emergency_stop_pub.publish(emergency_msg)
        self.get_logger().info('✅ 解除紧急停止信号已发送')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = StateMachineDirectTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



