#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import math

class PathPublisherNode(Node):
    def __init__(self):
        super().__init__('path_publisher_node')
        
        # 创建发布者
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        
        # 创建订阅者
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 路径数据
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.last_x = None
        self.last_y = None
        self.min_distance = 0.02  # 最小距离阈值（更敏感）
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.publish_path)
        
        self.get_logger().info('路径发布节点已启动')
    
    def odom_callback(self, msg):
        """处理里程计数据"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 检查是否需要添加新的路径点
        if self.last_x is None or self.last_y is None:
            self.add_pose_to_path(x, y, msg.pose.pose.orientation)
            self.last_x = x
            self.last_y = y
        else:
            # 计算距离
            distance = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
            
            # 只有当距离超过阈值时才添加新的路径点
            if distance >= self.min_distance:
                self.add_pose_to_path(x, y, msg.pose.pose.orientation)
                self.last_x = x
                self.last_y = y
    
    def add_pose_to_path(self, x, y, orientation):
        """添加姿态到路径"""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation = orientation
        
        self.path.poses.append(pose_stamped)
        
        # 限制路径长度（保留最近200个点）
        if len(self.path.poses) > 200:
            self.path.poses.pop(0)
    
    def publish_path(self):
        """发布路径"""
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PathPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
