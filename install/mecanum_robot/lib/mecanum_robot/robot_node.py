#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

class MecanumRobotNode(Node):
    def __init__(self):
        super().__init__('mecanum_robot_node')
        
        # 麦克纳姆轮参数
        self.wheel_radius = 0.05  # 轮子半径 (m)
        self.wheel_base = 0.3     # 轮距 (m) - 前后轮距离
        self.wheel_track = 0.3     # 轴距 (m) - 左右轮距离
        
        # 当前轮子角速度和位置
        self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]  # [front_left, front_right, rear_left, rear_right]
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]   # [front_left, front_right, rear_left, rear_right]
        
        # 机器人位置和姿态 (初始位置在原点)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # 创建订阅者和发布者
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        
        self.joint_state_pub = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry, 
            '/odom', 
            10
        )
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.odom_timer = self.create_timer(0.2, self.publish_odometry)  # 减少频率从0.05到0.2
        self.tf_timer = self.create_timer(0.1, self.publish_tf)
        
        self.get_logger().info('麦克纳姆轮机器人节点已启动')
        self.get_logger().info(f'初始位置: x={self.x}, y={self.y}, theta={self.theta}')
    
    def cmd_vel_callback(self, msg):
        """处理速度命令 - 麦克纳姆轮运动学逆解"""
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        # 麦克纳姆轮运动学逆解矩阵
        # 轮子顺序: [front_left, front_right, rear_left, rear_right]
        # 麦克纳姆轮运动学方程
        L = self.wheel_base / 2.0  # 前后轮距离的一半
        W = self.wheel_track / 2.0  # 左右轮距离的一半
        R = self.wheel_radius
        
        # 麦克纳姆轮运动学逆解
        # 每个轮子的角速度
        w1 = (vx - vy - (L + W) * wz) / R  # front_left
        w2 = (vx + vy + (L + W) * wz) / R  # front_right  
        w3 = (vx + vy - (L + W) * wz) / R  # rear_left
        w4 = (vx - vy + (L + W) * wz) / R  # rear_right
        
        self.wheel_velocities = [w1, w2, w3, w4]
        
        # 更新轮子位置（积分）
        dt = 0.1  # 与定时器频率一致
        for i in range(4):
            self.wheel_positions[i] += self.wheel_velocities[i] * dt
        
        self.get_logger().debug(f'收到速度命令: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}')
        self.get_logger().debug(f'轮子角速度: {[f"{v:.2f}" for v in self.wheel_velocities]}')
    
    def publish_joint_states(self):
        """发布关节状态"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        # 轮子关节名称
        joint_state.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint', 
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        
        # 轮子位置和角速度
        joint_state.position = self.wheel_positions
        joint_state.velocity = self.wheel_velocities
        
        self.joint_state_pub.publish(joint_state)
    
    def publish_odometry(self):
        """发布里程计信息"""
        # 麦克纳姆轮运动学正解
        L = self.wheel_base / 2.0
        W = self.wheel_track / 2.0
        R = self.wheel_radius
        
        # 从轮子角速度计算机器人速度
        w1, w2, w3, w4 = self.wheel_velocities
        
        vx = R * (w1 + w2 + w3 + w4) / 4.0
        vy = R * (-w1 + w2 + w3 - w4) / 4.0
        wz = R * (-w1 + w2 - w3 + w4) / (4.0 * (L + W))
        
        # 更新位置（积分）
        dt = 0.05
        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.theta += wz * dt
        
        # 创建里程计消息
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # 位置
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # 姿态（四元数）
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # 速度
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        
        self.odom_pub.publish(odom)
    
    def publish_tf(self):
        """发布TF变换"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # 位置
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # 姿态
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MecanumRobotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
