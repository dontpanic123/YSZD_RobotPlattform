#!/usr/bin/env python3
"""
测试定位状态更新
检查AprilTag检测和定位状态更新是否正常工作
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time

class LocationUpdateTester(Node):
    def __init__(self):
        super().__init__('location_update_tester')
        
        # 订阅AprilTag位姿
        self.apriltag_pose_sub = self.create_subscription(
            PoseStamped, '/apriltag_pose', self.apriltag_pose_callback, 10
        )
        
        # 订阅定位状态
        self.location_sub = self.create_subscription(
            String, '/robot_location', self.location_callback, 10
        )
        
        # 订阅机器人状态
        self.robot_status_sub = self.create_subscription(
            String, '/robot_status', self.robot_status_callback, 10
        )
        
        self.apriltag_received = False
        self.location_received = False
        self.last_location = None
        
        self.get_logger().info('🔍 定位状态更新测试器已启动')
        self.get_logger().info('📡 订阅话题:')
        self.get_logger().info('   - /apriltag_pose')
        self.get_logger().info('   - /robot_location')
        self.get_logger().info('   - /robot_status')
        self.get_logger().info('⏳ 等待消息...')
        
        # 创建定时器，定期输出状态
        self.timer = self.create_timer(2.0, self.print_status)
    
    def apriltag_pose_callback(self, msg):
        """AprilTag位姿回调"""
        self.apriltag_received = True
        frame_id = msg.header.frame_id
        
        # 提取tag_id
        tag_id = None
        if 'apriltag_' in frame_id:
            try:
                tag_id_str = frame_id.split('apriltag_')[-1]
                tag_id = int(tag_id_str)
                self.get_logger().info(f'✅ 收到AprilTag位姿: frame_id={frame_id}, tag_id={tag_id}')
            except ValueError:
                self.get_logger().warn(f'⚠️ 无法从frame_id提取tag_id: {frame_id}')
        else:
            self.get_logger().warn(f'⚠️ frame_id格式不正确: {frame_id}')
    
    def location_callback(self, msg):
        """定位状态回调"""
        self.location_received = True
        location = msg.data
        
        if location != self.last_location:
            self.get_logger().info(f'📍 定位状态更新: {self.last_location} -> {location}')
            self.last_location = location
        else:
            self.get_logger().debug(f'📍 定位状态: {location} (未变化)')
    
    def robot_status_callback(self, msg):
        """机器人状态回调"""
        # 解析状态信息
        status_str = msg.data
        self.get_logger().debug(f'📊 机器人状态: {status_str}')
    
    def print_status(self):
        """定期输出状态"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('📊 当前状态:')
        self.get_logger().info(f'   AprilTag消息: {"✅ 已收到" if self.apriltag_received else "❌ 未收到"}')
        self.get_logger().info(f'   定位消息: {"✅ 已收到" if self.location_received else "❌ 未收到"}')
        self.get_logger().info(f'   当前定位: {self.last_location if self.last_location else "未知"}')
        self.get_logger().info('=' * 50)
        
        # 重置标志
        self.apriltag_received = False
        self.location_received = False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = LocationUpdateTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




