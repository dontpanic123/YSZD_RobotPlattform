#!/usr/bin/env python3
"""
Waypoint Recorder for Nav2
订阅机器人运动轨迹 生成Nav2兼容的waypoints文件
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from std_srvs.srv import Empty
import json
import os
import time
from datetime import datetime
import math

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        
        # 参数设置
        self.declare_parameter('recording', False)
        self.declare_parameter('waypoint_distance_threshold', 0.5)  # 米
        self.declare_parameter('waypoint_angle_threshold', 0.2)      # 弧度
        self.declare_parameter('output_directory', '/home/bd/Documents/Robot/agv_sim/waypoints')
        self.declare_parameter('topic_odom', '/odom')
        
        # 获取参数
        self.recording = self.get_parameter('recording').value
        self.distance_threshold = self.get_parameter('waypoint_distance_threshold').value
        self.angle_threshold = self.get_parameter('waypoint_angle_threshold').value
        self.output_dir = self.get_parameter('output_directory').value
        self.odom_topic = self.get_parameter('topic_odom').value
        
        # 状态变量
        self.waypoints = []
        self.last_waypoint = None
        self.recording_start_time = None
        self.recording_duration = 0.0
        
        # 创建输出目录
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 订阅里程计数据
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        
        # 发布路径用于可视化
        self.path_publisher = self.create_publisher(Path, '/recorded_path', 10)
        
        # 服务
        self.create_service(
            Empty,
            'start_recording',
            self.start_recording_callback
        )
        
        self.create_service(
            Empty,
            'stop_recording',
            self.stop_recording_callback
        )
        
        self.create_service(
            Empty,
            'save_waypoints',
            self.save_waypoints_callback
        )
        
        # 定时器
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'Waypoint Recorder 已启动')
        self.get_logger().info(f'输出目录: {self.output_dir}')
        self.get_logger().info(f'距离阈值: {self.distance_threshold}m')
        self.get_logger().info(f'角度阈值: {self.angle_threshold}rad')
        
    def odom_callback(self, msg):
        """处理里程计数据"""
        if not self.recording:
            return
            
        current_pose = msg.pose.pose
        current_time = msg.header.stamp
        
        # 检查是否需要添加新的waypoint
        if self.should_add_waypoint(current_pose):
            self.add_waypoint(current_pose, current_time)
            
    def should_add_waypoint(self, current_pose):
        """判断是否应该添加新的waypoint"""
        if self.last_waypoint is None:
            return True
            
        # 计算距离
        dx = current_pose.position.x - self.last_waypoint.position.x
        dy = current_pose.position.y - self.last_waypoint.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 计算角度差
        current_yaw = self.quaternion_to_yaw(current_pose.orientation)
        last_yaw = self.quaternion_to_yaw(self.last_waypoint.orientation)
        angle_diff = abs(self.normalize_angle(current_yaw - last_yaw))
        
        # 检查是否超过阈值
        return (distance > self.distance_threshold or 
                angle_diff > self.angle_threshold)
                
    def add_waypoint(self, pose, timestamp):
        """添加新的waypoint"""
        waypoint = {
            'position': {
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'z': float(pose.position.z)
            },
            'orientation': {
                'x': float(pose.orientation.x),
                'y': float(pose.orientation.y),
                'z': float(pose.orientation.z),
                'w': float(pose.orientation.w)
            },
            'timestamp': {
                'sec': timestamp.sec,
                'nanosec': timestamp.nanosec
            }
        }
        
        self.waypoints.append(waypoint)
        self.last_waypoint = pose
        
        self.get_logger().info(f'添加waypoint #{len(self.waypoints)}: '
                              f'({pose.position.x:.2f}, {pose.position.y:.2f})')
        
    def start_recording_callback(self, request, response):
        """开始录制"""
        if self.recording:
            self.get_logger().warn('已经在录制中')
            return response
            
        self.recording = True
        self.waypoints = []
        self.last_waypoint = None
        self.recording_start_time = time.time()
        
        self.get_logger().info('开始录制waypoints')
        return response
        
    def stop_recording_callback(self, request, response):
        """停止录制"""
        if not self.recording:
            self.get_logger().warn('当前没有在录制')
            return response
            
        self.recording = False
        if self.recording_start_time:
            self.recording_duration = time.time() - self.recording_start_time
            
        self.get_logger().info(f'停止录制，共录制了 {len(self.waypoints)} 个waypoints')
        return response
        
    def save_waypoints_callback(self, request, response):
        """保存waypoints到文件"""
        if not self.waypoints:
            self.get_logger().warn('没有waypoints可保存')
            return response
            
        # 生成文件名
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'waypoints_{timestamp}.json'
        filepath = os.path.join(self.output_dir, filename)
        
        # 创建Nav2兼容的waypoints数据
        nav2_waypoints = self.create_nav2_waypoints()
        
        # 保存到JSON文件
        with open(filepath, 'w') as f:
            json.dump(nav2_waypoints, f, indent=2)
            
        self.get_logger().info(f'Waypoints已保存到: {filepath}')
        self.get_logger().info(f'共保存 {len(self.waypoints)} 个waypoints')
        
        return response
        
    def create_nav2_waypoints(self):
        """创建Nav2兼容的waypoints数据"""
        nav2_data = {
            'waypoints': [],
            'metadata': {
                'total_waypoints': len(self.waypoints),
                'recording_duration': self.recording_duration,
                'distance_threshold': self.distance_threshold,
                'angle_threshold': self.angle_threshold,
                'created_at': datetime.now().isoformat()
            }
        }
        
        for i, wp in enumerate(self.waypoints):
            waypoint = {
                'id': i,
                'position': wp['position'],
                'orientation': wp['orientation'],
                'timestamp': wp['timestamp']
            }
            nav2_data['waypoints'].append(waypoint)
            
        return nav2_data
        
    def timer_callback(self):
        """定时器回调，发布路径用于可视化"""
        if not self.waypoints:
            return
            
        # 创建Path消息
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for wp in self.waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            pose_stamped.pose.position.x = wp['position']['x']
            pose_stamped.pose.position.y = wp['position']['y']
            pose_stamped.pose.position.z = wp['position']['z']
            
            pose_stamped.pose.orientation.x = wp['orientation']['x']
            pose_stamped.pose.orientation.y = wp['orientation']['y']
            pose_stamped.pose.orientation.z = wp['orientation']['z']
            pose_stamped.pose.orientation.w = wp['orientation']['w']
            
            path_msg.poses.append(pose_stamped)
            
        self.path_publisher.publish(path_msg)
        
    def quaternion_to_yaw(self, quaternion):
        """四元数转yaw角"""
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw
        
    def normalize_angle(self, angle):
        """角度归一化到[-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    
    node = WaypointRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
