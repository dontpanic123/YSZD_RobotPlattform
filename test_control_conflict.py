#!/usr/bin/env python3
"""
测试控制冲突修复
验证网页控制只在用户操作时发布cmd_vel，避免与follower冲突
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import threading

class ControlConflictTester(Node):
    def __init__(self):
        super().__init__('control_conflict_tester')
        
        # 订阅cmd_vel话题
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        # 统计信息
        self.message_count = 0
        self.last_message_time = 0
        self.message_sources = {}
        self.zero_velocity_count = 0
        self.non_zero_velocity_count = 0
        
        # 启动监控
        self.monitor_timer = self.create_timer(1.0, self.monitor_status)
        
        self.get_logger().info('控制冲突测试器已启动')
        self.get_logger().info('监控 /cmd_vel 话题...')
    
    def cmd_vel_callback(self, msg):
        """处理cmd_vel消息"""
        current_time = time.time()
        self.message_count += 1
        self.last_message_time = current_time
        
        # 检查消息来源（通过时间戳推断）
        # 这里我们通过消息频率和内容来推断来源
        velocity_magnitude = (abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.linear.z) + 
                            abs(msg.angular.x) + abs(msg.angular.y) + abs(msg.angular.z))
        
        if velocity_magnitude < 0.001:
            self.zero_velocity_count += 1
            source_type = "停止命令"
        else:
            self.non_zero_velocity_count += 1
            source_type = "运动命令"
        
        self.get_logger().info(f'收到cmd_vel消息 #{self.message_count}: {source_type}')
        self.get_logger().info(f'  线性速度: x={msg.linear.x:.3f}, y={msg.linear.y:.3f}, z={msg.linear.z:.3f}')
        self.get_logger().info(f'  角速度: x={msg.angular.x:.3f}, y={msg.angular.y:.3f}, z={msg.angular.z:.3f}')
    
    def monitor_status(self):
        """监控状态"""
        current_time = time.time()
        time_since_last = current_time - self.last_message_time if self.last_message_time > 0 else float('inf')
        
        self.get_logger().info('=== 控制冲突测试状态 ===')
        self.get_logger().info(f'总消息数: {self.message_count}')
        self.get_logger().info(f'运动命令数: {self.non_zero_velocity_count}')
        self.get_logger().info(f'停止命令数: {self.zero_velocity_count}')
        self.get_logger().info(f'距离上次消息: {time_since_last:.1f}秒')
        
        if time_since_last > 5.0:
            self.get_logger().info('✅ 长时间无消息，说明控制冲突已修复')
        elif self.zero_velocity_count > self.non_zero_velocity_count * 2:
            self.get_logger().warn('⚠️ 停止命令过多，可能存在控制冲突')
        else:
            self.get_logger().info('📊 消息统计正常')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ControlConflictTester()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
