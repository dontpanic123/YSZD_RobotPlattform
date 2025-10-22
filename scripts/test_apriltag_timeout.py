#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time

class AprilTagTimeoutTest(Node):
    """测试AprilTag超时功能"""
    
    def __init__(self):
        super().__init__('apriltag_timeout_test')
        
        # 创建发布者
        self.apriltag_pose_pub = self.create_publisher(PoseStamped, '/apriltag_pose', 10)
        
        # 创建订阅者
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10
        )
        
        self.current_state = "unknown"
        self.test_start_time = time.time()
        
        self.get_logger().info('🧪 AprilTag超时测试开始')
        self.get_logger().info('📋 测试步骤:')
        self.get_logger().info('   1. 发送AprilTag位姿 -> 应该进入APRILTAG_TRACKING状态')
        self.get_logger().info('   2. 停止发送AprilTag位姿')
        self.get_logger().info('   3. 等待60秒 -> 应该自动返回IDLE状态')
        
        # 开始测试
        self.create_timer(1.0, self.test_timer)
        self.create_timer(0.5, self.send_apriltag_pose)
        
    def state_callback(self, msg):
        """状态回调"""
        if msg.data != self.current_state:
            self.current_state = msg.data
            self.get_logger().info(f'🔄 状态变化: {self.current_state}')
            
            if self.current_state == 'apriltag_tracking':
                self.get_logger().info('✅ 成功进入AprilTag跟踪状态')
            elif self.current_state == 'idle' and self.test_start_time > 0:
                elapsed = time.time() - self.test_start_time
                if elapsed > 50:  # 大约60秒后
                    self.get_logger().info('✅ 成功超时返回空闲状态')
                    self.get_logger().info('🎉 测试完成!')
                    self.destroy_node()
                    return
    
    def send_apriltag_pose(self):
        """发送AprilTag位姿"""
        if time.time() - self.test_start_time < 5.0:  # 前5秒发送AprilTag
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'apriltag_1'
            pose.pose.position.x = 1.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            self.apriltag_pose_pub.publish(pose)
    
    def test_timer(self):
        """测试定时器"""
        elapsed = time.time() - self.test_start_time
        
        if elapsed < 5:
            self.get_logger().info(f'⏱️ 测试进行中: {elapsed:.1f}秒 (发送AprilTag)')
        elif elapsed < 65:
            self.get_logger().info(f'⏱️ 测试进行中: {elapsed:.1f}秒 (等待超时)')
        else:
            self.get_logger().warn('⚠️ 测试超时，可能有问题')
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = AprilTagTimeoutTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
