#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class StateDisplayTest(Node):
    """测试状态显示功能"""
    
    def __init__(self):
        super().__init__('state_display_test')
        
        # 创建发布者
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        
        self.get_logger().info('🧪 状态显示测试开始')
        self.get_logger().info('📋 将每3秒发布一次状态变化')
        
        # 状态列表
        self.states = ['idle', 'manual_control', 'auto_navigation', 'emergency_stop', 'idle']
        self.current_index = 0
        
        # 创建定时器
        self.create_timer(3.0, self.publish_state)
        
    def publish_state(self):
        """发布状态"""
        if self.current_index < len(self.states):
            state = self.states[self.current_index]
            msg = String()
            msg.data = state
            
            self.state_pub.publish(msg)
            self.get_logger().info(f'📤 发布状态: {state}')
            
            # 下一个状态
            self.current_index += 1
        else:
            self.get_logger().info('🎉 测试完成!')
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = StateDisplayTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






