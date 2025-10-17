#!/usr/bin/env python3
"""
Nav2 Waypoint Follower
使用录制的waypoints进行路径跟踪
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import json
import os
import time
from datetime import datetime

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # 参数设置
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('loop_waypoints', False)
        self.declare_parameter('waypoint_timeout', 30.0)
        
        # 获取参数
        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.loop_waypoints = self.get_parameter('loop_waypoints').value
        self.waypoint_timeout = self.get_parameter('waypoint_timeout').value
        
        # 状态变量
        self.waypoints = []
        self.current_waypoint_index = 0
        self.is_following = False
        
        # 创建Action客户端
        self.follow_waypoints_client = ActionClient(
            self, 
            FollowWaypoints, 
            '/Follow_waypoints'
        )
        
        self.get_logger().info('Waypoint Follower 已启动')
        
        # 如果指定了waypoints文件，自动加载
        if self.waypoints_file:
            self.load_waypoints(self.waypoints_file)
            
    def load_waypoints(self, filepath):
        """加载waypoints文件"""
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
                
            self.waypoints = []
            for wp_data in data['waypoints']:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = 'map'
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                
                # 设置位置
                pose_stamped.pose.position.x = wp_data['position']['x']
                pose_stamped.pose.position.y = wp_data['position']['y']
                pose_stamped.pose.position.z = wp_data['position']['z']
                
                # 设置方向
                pose_stamped.pose.orientation.x = wp_data['orientation']['x']
                pose_stamped.pose.orientation.y = wp_data['orientation']['y']
                pose_stamped.pose.orientation.z = wp_data['orientation']['z']
                pose_stamped.pose.orientation.w = wp_data['orientation']['w']
                
                self.waypoints.append(pose_stamped)
                
            self.get_logger().info(f'成功加载 {len(self.waypoints)} 个waypoints')
            return True
            
        except Exception as e:
            self.get_logger().error(f'加载waypoints文件失败: {e}')
            return False
            
    def start_following(self):
        """开始跟踪waypoints"""
        if not self.waypoints:
            self.get_logger().error('没有waypoints可跟踪')
            return False
            
        if self.is_following:
            self.get_logger().warn('已经在跟踪waypoints')
            return False
            
        # 等待Action服务器
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowWaypoints Action服务器不可用')
            return False
            
        # 创建目标
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints
        
        # 发送目标
        self.follow_waypoints_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)
        
        self.is_following = True
        self.get_logger().info(f'开始跟踪 {len(self.waypoints)} 个waypoints')
        return True
        
    def goal_response_callback(self, future):
        """目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Waypoint跟踪目标被拒绝')
            self.is_following = False
            return
            
        self.get_logger().info('Waypoint跟踪目标已接受')
        
        # 等待结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
        
    def feedback_callback(self, feedback_msg):
        """反馈回调"""
        current_waypoint = feedback_msg.current_waypoint
        self.get_logger().info(f'当前waypoint: {current_waypoint}')
        
    def result_callback(self, future):
        """结果回调"""
        result = future.result().result
        self.is_following = False
        
        if result.missed_waypoints:
            self.get_logger().warn(f'错过了 {len(result.missed_waypoints)} 个waypoints')
        else:
            self.get_logger().info('成功完成所有waypoints跟踪')
            
        # 如果设置了循环，重新开始
        if self.loop_waypoints:
            self.get_logger().info('循环模式：重新开始跟踪waypoints')
            time.sleep(2.0)  # 等待2秒
            self.start_following()
            
    def stop_following(self):
        """停止跟踪"""
        if self.is_following:
            self.follow_waypoints_client.cancel_goal_async()
            self.is_following = False
            self.get_logger().info('停止waypoint跟踪')
            
    def get_status(self):
        """获取状态信息"""
        return {
            'is_following': self.is_following,
            'total_waypoints': len(self.waypoints),
            'current_index': self.current_waypoint_index
        }

def main(args=None):
    rclpy.init(args=args)
    
    node = WaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_following()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
