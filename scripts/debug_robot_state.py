#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class RobotStateDebugger(Node):
    """调试机器人状态传输"""
    
    def __init__(self):
        super().__init__('robot_state_debugger')
        
        # 创建发布者
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        
        # 创建订阅者
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10
        )
        
        self.get_logger().info('🔍 机器人状态调试器启动')
        self.get_logger().info('📋 将每5秒发布一次状态变化')
        
        # 状态列表
        self.states = ['idle', 'manual_control', 'auto_navigation', 'apriltag_tracking', 'emergency_stop', 'charging', 'error']
        self.current_index = 0
        
        # 创建定时器
        self.create_timer(5.0, self.publish_state)
        
    def state_callback(self, msg):
        """状态回调"""
        self.get_logger().info(f'📨 收到状态消息: {msg.data}')
    
    def publish_state(self):
        """发布状态"""
        state = self.states[self.current_index]
        msg = String()
        msg.data = state
        
        self.state_pub.publish(msg)
        self.get_logger().info(f'📤 发布状态: {state}')
        
        # 下一个状态
        self.current_index = (self.current_index + 1) % len(self.states)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        debugger = RobotStateDebugger()
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        pass
    finally:
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






