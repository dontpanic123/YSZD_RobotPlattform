#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time

class EmergencyDirectTest(Node):
    """直接测试紧急停止消息"""
    
    def __init__(self):
        super().__init__('emergency_direct_test')
        
        # 创建发布者
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # 创建订阅者
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10
        )
        
        self.current_state = "unknown"
        
        self.get_logger().info('🧪 直接测试紧急停止消息')
        
        # 等待2秒后发送紧急停止
        self.create_timer(2.0, self.send_emergency_stop)
        
    def state_callback(self, msg):
        """状态回调"""
        if msg.data != self.current_state:
            self.current_state = msg.data
            self.get_logger().info(f'🔄 状态变化: {self.current_state}')
    
    def send_emergency_stop(self):
        """发送紧急停止"""
        self.get_logger().info('📤 发送紧急停止信号...')
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)
        self.get_logger().info('✅ 紧急停止信号已发送')
        
        # 5秒后解除
        self.create_timer(5.0, self.clear_emergency_stop)
    
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
        test_node = EmergencyDirectTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
