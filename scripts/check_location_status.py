#!/usr/bin/env python3
"""
快速检查定位状态
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

def main():
    rclpy.init()
    node = Node('check_location')
    
    # 检查节点是否运行
    node.get_logger().info('🔍 检查机器人状态机节点...')
    node_list = node.get_node_names()
    if 'robot_state_machine' in node_list:
        node.get_logger().info('✅ 机器人状态机节点正在运行')
    else:
        node.get_logger().error('❌ 机器人状态机节点未运行！')
        node.get_logger().info('💡 请运行: ros2 run mecanum_robot robot_state_machine.py')
        rclpy.shutdown()
        return
    
    # 检查话题
    node.get_logger().info('🔍 检查话题...')
    topic_list = node.get_topic_names_and_types()
    
    apriltag_pose_exists = False
    robot_location_exists = False
    
    for topic_name, topic_types in topic_list:
        if '/apriltag_pose' in topic_name:
            apriltag_pose_exists = True
            node.get_logger().info(f'✅ 找到话题: {topic_name} (类型: {topic_types})')
        if '/robot_location' in topic_name:
            robot_location_exists = True
            node.get_logger().info(f'✅ 找到话题: {topic_name} (类型: {topic_types})')
    
    if not apriltag_pose_exists:
        node.get_logger().warn('⚠️ 未找到 /apriltag_pose 话题')
    if not robot_location_exists:
        node.get_logger().warn('⚠️ 未找到 /robot_location 话题')
    
    # 订阅定位状态
    location_received = [False]
    location_value = [None]
    
    def location_callback(msg):
        location_received[0] = True
        location_value[0] = msg.data
        node.get_logger().info(f'📍 当前定位: {msg.data}')
    
    location_sub = node.create_subscription(
        String, '/robot_location', location_callback, 10
    )
    
    # 等待消息
    node.get_logger().info('⏳ 等待定位状态消息（5秒）...')
    import time
    start_time = time.time()
    
    while time.time() - start_time < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if location_received[0]:
            break
    
    if location_received[0]:
        node.get_logger().info(f'✅ 定位状态: {location_value[0]}')
    else:
        node.get_logger().warn('⚠️ 5秒内未收到定位状态消息')
        node.get_logger().info('💡 可能的原因:')
        node.get_logger().info('   1. 机器人状态机节点未正确订阅 /apriltag_pose')
        node.get_logger().info('   2. AprilTag检测器未发布消息')
        node.get_logger().info('   3. 定位状态更新逻辑有问题')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()




