#!/usr/bin/env python3
"""
测试摇杆防漂移功能
验证摇杆归中时的防漂移机制
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import threading

class JoystickDriftTester(Node):
    def __init__(self):
        super().__init__('joystick_drift_tester')
        
        # 订阅cmd_vel话题
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        # 统计信息
        self.message_count = 0
        self.last_message_time = 0
        self.zero_commands = 0
        self.motion_commands = 0
        self.drift_detected = 0
        self.last_velocity = None
        self.zero_sequence_start = None
        self.zero_sequence_count = 0
        
        # 启动监控
        self.monitor_timer = self.create_timer(2.0, self.monitor_status)
        
        self.get_logger().info('摇杆防漂移测试器已启动')
        self.get_logger().info('监控摇杆归中时的防漂移效果...')
    
    def cmd_vel_callback(self, msg):
        """处理cmd_vel消息"""
        current_time = time.time()
        self.message_count += 1
        self.last_message_time = current_time
        
        # 计算速度大小
        velocity_magnitude = (abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.linear.z) + 
                            abs(msg.angular.x) + abs(msg.angular.y) + abs(msg.angular.z))
        
        is_zero_command = velocity_magnitude < 0.001
        
        if is_zero_command:
            self.zero_commands += 1
            if self.zero_sequence_start is None:
                self.zero_sequence_start = current_time
                self.zero_sequence_count += 1
                self.get_logger().info(f'🛑 检测到归零序列 #{self.zero_sequence_count}')
        else:
            self.motion_commands += 1
            self.zero_sequence_start = None
        
        # 检测漂移
        if self.last_velocity is not None:
            velocity_change = abs(velocity_magnitude - self.last_velocity)
            if velocity_change > 0.1:  # 速度变化超过阈值
                self.drift_detected += 1
                self.get_logger().warn(f'⚠️ 检测到可能的漂移: 速度变化 {velocity_change:.3f}')
        
        self.last_velocity = velocity_magnitude
        
        # 详细日志
        self.get_logger().info(f'消息 #{self.message_count}: {"归零" if is_zero_command else "运动"}')
        self.get_logger().info(f'  线性: x={msg.linear.x:.3f}, y={msg.linear.y:.3f}, z={msg.linear.z:.3f}')
        self.get_logger().info(f'  角速度: x={msg.angular.x:.3f}, y={msg.angular.y:.3f}, z={msg.angular.z:.3f}')
        self.get_logger().info(f'  速度大小: {velocity_magnitude:.3f}')
    
    def monitor_status(self):
        """监控状态"""
        current_time = time.time()
        time_since_last = current_time - self.last_message_time if self.last_message_time > 0 else float('inf')
        
        self.get_logger().info('=== 摇杆防漂移测试状态 ===')
        self.get_logger().info(f'总消息数: {self.message_count}')
        self.get_logger().info(f'运动命令数: {self.motion_commands}')
        self.get_logger().info(f'归零命令数: {self.zero_commands}')
        self.get_logger().info(f'归零序列数: {self.zero_sequence_count}')
        self.get_logger().info(f'漂移检测数: {self.drift_detected}')
        self.get_logger().info(f'距离上次消息: {time_since_last:.1f}秒')
        
        # 分析结果
        if self.zero_commands > 0:
            self.get_logger().info('✅ 检测到归零命令，防漂移功能正常')
        else:
            self.get_logger().warn('⚠️ 未检测到归零命令')
        
        if self.drift_detected == 0:
            self.get_logger().info('✅ 未检测到漂移，防漂移机制有效')
        else:
            self.get_logger().warn(f'⚠️ 检测到 {self.drift_detected} 次可能的漂移')
        
        if self.zero_sequence_count > 0:
            self.get_logger().info(f'✅ 检测到 {self.zero_sequence_count} 个归零序列')
        
        if time_since_last > 5.0:
            self.get_logger().info('📊 长时间无消息，系统处于空闲状态')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = JoystickDriftTester()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
