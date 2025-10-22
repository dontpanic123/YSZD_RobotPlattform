#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import numpy as np

class PositionCalculatorNode(Node):
    def __init__(self):
        super().__init__('position_calculator_node')
        
        # 创建发布者
        self.relative_pose_pub = self.create_publisher(PoseStamped, '/robot_relative_pose', 10)
        self.navigation_cmd_pub = self.create_publisher(Twist, '/navigation_cmd', 10)
        self.status_pub = self.create_publisher(String, '/position_status', 10)
        
        # 创建订阅者
        self.apriltag_pose_sub = self.create_subscription(
            PoseStamped,
            '/apriltag_pose',
            self.apriltag_pose_callback,
            10
        )
        
        self.robot_odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.robot_odom_callback,
            10
        )
        
        # 机器人位置和姿态
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # AprilTag位置
        self.apriltag_x = 0.0
        self.apriltag_y = 0.0
        self.apriltag_z = 0.0
        self.apriltag_detected = False
        
        # 目标位置
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        
        # 控制参数
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0
        self.position_tolerance = 0.1
        self.angle_tolerance = 0.1
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.calculate_relative_position)
        self.navigation_timer = self.create_timer(0.1, self.navigation_control)
        
        self.get_logger().info('位置计算节点已启动')
    
    def apriltag_pose_callback(self, msg):
        """处理AprilTag位姿"""
        self.apriltag_x = msg.pose.position.x
        self.apriltag_y = msg.pose.position.y
        self.apriltag_z = msg.pose.position.z
        self.apriltag_detected = True
        
        self.get_logger().debug(f'AprilTag位置: ({self.apriltag_x:.2f}, {self.apriltag_y:.2f}, {self.apriltag_z:.2f})')
    
    def robot_odom_callback(self, msg):
        """处理机器人里程计"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # 从四元数计算偏航角
        orientation = msg.pose.pose.orientation
        self.robot_theta = math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )
    
    def calculate_relative_position(self):
        """计算相对位置"""
        if not self.apriltag_detected:
            return
        
        # 计算机器人到AprilTag的相对位置
        # 这里假设AprilTag在机器人坐标系中的位置
        relative_x = self.apriltag_x
        relative_y = self.apriltag_y
        relative_z = self.apriltag_z
        
        # 计算距离和角度
        distance = math.sqrt(relative_x**2 + relative_y**2)
        angle = math.atan2(relative_y, relative_x)
        
        # 发布相对位姿
        relative_pose = PoseStamped()
        relative_pose.header.stamp = self.get_clock().now().to_msg()
        relative_pose.header.frame_id = 'base_link'
        
        relative_pose.pose.position.x = relative_x
        relative_pose.pose.position.y = relative_y
        relative_pose.pose.position.z = relative_z
        
        # 设置朝向AprilTag的旋转
        relative_pose.pose.orientation.w = math.cos(angle / 2.0)
        relative_pose.pose.orientation.x = 0.0
        relative_pose.pose.orientation.y = 0.0
        relative_pose.pose.orientation.z = math.sin(angle / 2.0)
        
        self.relative_pose_pub.publish(relative_pose)
        
        # 发布状态信息
        status_msg = String()
        status_msg.data = f"相对位置: 距离={distance:.2f}m, 角度={math.degrees(angle):.1f}°"
        self.status_pub.publish(status_msg)
    
    def navigation_control(self):
        """导航控制"""
        if not self.apriltag_detected:
            return
        
        # 计算到AprilTag的误差
        error_x = self.apriltag_x
        error_y = self.apriltag_y
        error_distance = math.sqrt(error_x**2 + error_y**2)
        error_angle = math.atan2(error_y, error_x)
        
        # 简单的PID控制
        cmd_vel = Twist()
        
        # 线性速度控制
        if error_distance > self.position_tolerance:
            # 朝向AprilTag移动
            cmd_vel.linear.x = min(self.max_linear_speed, error_distance * 0.5)
            cmd_vel.linear.y = 0.0  # 麦克纳姆轮可以侧向移动
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
        
        # 角速度控制
        if abs(error_angle) > self.angle_tolerance:
            cmd_vel.angular.z = max(-self.max_angular_speed, 
                                  min(self.max_angular_speed, error_angle * 2.0))
        else:
            cmd_vel.angular.z = 0.0
        
        # 发布导航命令
        self.navigation_cmd_pub.publish(cmd_vel)
        
        # 发布状态
        status_msg = String()
        status_msg.data = f"导航控制: 距离误差={error_distance:.2f}m, 角度误差={math.degrees(error_angle):.1f}°"
        self.status_pub.publish(status_msg)
    
    def set_target_position(self, x, y, theta=0.0):
        """设置目标位置"""
        self.target_x = x
        self.target_y = y
        self.target_theta = theta
        self.get_logger().info(f'目标位置设置为: ({x}, {y}, {theta})')
    
    def calculate_distance_to_target(self):
        """计算到目标的距离"""
        if not self.apriltag_detected:
            return float('inf')
        
        dx = self.apriltag_x - self.target_x
        dy = self.apriltag_y - self.target_y
        return math.sqrt(dx**2 + dy**2)
    
    def calculate_angle_to_target(self):
        """计算到目标的角度"""
        if not self.apriltag_detected:
            return 0.0
        
        dx = self.apriltag_x - self.target_x
        dy = self.apriltag_y - self.target_y
        return math.atan2(dy, dx)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PositionCalculatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()



















