#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import select
import tty
import termios
import threading
import time

class RemoteControlNode(Node):
    def __init__(self):
        super().__init__('remote_control_node')
        
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # 麦克纳姆轮参数
        self.max_linear_speed = 1.0  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.speed_increment = 0.1
        
        # 当前速度
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        # 创建定时器，定期发布速度命令
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # 设置终端为原始模式
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        self.get_logger().info('遥控节点已启动')
        self.get_logger().info('控制说明:')
        self.get_logger().info('  w/s: 前进/后退')
        self.get_logger().info('  a/d: 左移/右移')
        self.get_logger().info('  q/e: 左转/右转')
        self.get_logger().info('  x: 停止')
        self.get_logger().info('  +: 增加速度')
        self.get_logger().info('  -: 减少速度')
        self.get_logger().info('  Ctrl+C: 退出')
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
    
    def keyboard_listener(self):
        """键盘监听线程"""
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                self.process_key(key)
    
    def process_key(self, key):
        """处理键盘输入"""
        if key == '\x03':  # Ctrl+C
            self.cleanup()
            rclpy.shutdown()
            return
        
        # 麦克纳姆轮控制
        if key == 'w':  # 前进
            self.current_linear_x = min(self.current_linear_x + self.speed_increment, self.max_linear_speed)
            self.get_logger().info(f'前进: 线性速度 X = {self.current_linear_x:.2f}')
        elif key == 's':  # 后退
            self.current_linear_x = max(self.current_linear_x - self.speed_increment, -self.max_linear_speed)
            self.get_logger().info(f'后退: 线性速度 X = {self.current_linear_x:.2f}')
        elif key == 'a':  # 左移
            self.current_linear_y = min(self.current_linear_y + self.speed_increment, self.max_linear_speed)
            self.get_logger().info(f'左移: 线性速度 Y = {self.current_linear_y:.2f}')
        elif key == 'd':  # 右移
            self.current_linear_y = max(self.current_linear_y - self.speed_increment, -self.max_linear_speed)
            self.get_logger().info(f'右移: 线性速度 Y = {self.current_linear_y:.2f}')
        elif key == 'q':  # 左转
            self.current_angular_z = min(self.current_angular_z + self.speed_increment, self.max_angular_speed)
            self.get_logger().info(f'左转: 角速度 Z = {self.current_angular_z:.2f}')
        elif key == 'e':  # 右转
            self.current_angular_z = max(self.current_angular_z - self.speed_increment, -self.max_angular_speed)
            self.get_logger().info(f'右转: 角速度 Z = {self.current_angular_z:.2f}')
        elif key == 'x':  # 停止
            self.current_linear_x = 0.0
            self.current_linear_y = 0.0
            self.current_angular_z = 0.0
            self.get_logger().info('停止')
        elif key == '+':  # 增加速度
            self.max_linear_speed = min(self.max_linear_speed + 0.1, 2.0)
            self.max_angular_speed = min(self.max_angular_speed + 0.1, 2.0)
            self.get_logger().info(f'最大速度增加到: 线性={self.max_linear_speed:.2f}, 角速度={self.max_angular_speed:.2f}')
        elif key == '-':  # 减少速度
            self.max_linear_speed = max(self.max_linear_speed - 0.1, 0.1)
            self.max_angular_speed = max(self.max_angular_speed - 0.1, 0.1)
            self.get_logger().info(f'最大速度减少到: 线性={self.max_linear_speed:.2f}, 角速度={self.max_angular_speed:.2f}')
    
    def publish_velocity(self):
        """发布速度命令"""
        twist = Twist()
        twist.linear.x = self.current_linear_x
        twist.linear.y = self.current_linear_y
        twist.angular.z = self.current_angular_z
        
        self.cmd_vel_pub.publish(twist)
        
        # 发布状态信息
        status_msg = String()
        status_msg.data = f"Linear: x={self.current_linear_x:.2f}, y={self.current_linear_y:.2f}, Angular: z={self.current_angular_z:.2f}"
        self.status_pub.publish(status_msg)
    
    def cleanup(self):
        """清理资源"""
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        
        # 发送停止命令
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('已发送停止命令')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RemoteControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



