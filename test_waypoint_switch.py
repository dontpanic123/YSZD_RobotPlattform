#!/usr/bin/env python3
"""
测试路径点文件切换功能
验证更换waypoints.json文件后能正确加载新的路径点
"""

import rclpy
from rclpy.node import Node
from std_msgs.srv import SetString
from std_srvs.srv import Empty
import time
import json
import os

class WaypointSwitchTester(Node):
    def __init__(self):
        super().__init__('waypoint_switch_tester')
        
        # 创建服务客户端
        self.set_file_client = self.create_client(SetString, '/set_waypoints_file_path')
        self.start_following_client = self.create_client(Empty, '/start_following')
        self.stop_following_client = self.create_client(Empty, '/stop_following')
        
        # 等待服务可用
        self.wait_for_services()
        
        self.get_logger().info('路径点文件切换测试器已启动')
        self.get_logger().info('测试路径点文件切换功能...')
        
        # 开始测试
        self.run_test()
    
    def wait_for_services(self):
        """等待服务可用"""
        self.get_logger().info('等待服务可用...')
        
        # 等待设置文件服务
        while not self.set_file_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('等待 /set_waypoints_file_path 服务...')
        
        # 等待开始跟踪服务
        while not self.start_following_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('等待 /start_following 服务...')
        
        # 等待停止跟踪服务
        while not self.stop_following_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('等待 /stop_following 服务...')
        
        self.get_logger().info('✅ 所有服务已可用')
    
    def run_test(self):
        """运行测试"""
        try:
            # 获取可用的waypoint文件
            waypoint_files = self.get_waypoint_files()
            
            if len(waypoint_files) < 2:
                self.get_logger().error('❌ 需要至少2个waypoint文件进行测试')
                return
            
            self.get_logger().info(f'📁 找到 {len(waypoint_files)} 个waypoint文件')
            
            # 测试第一个文件
            self.test_waypoint_file(waypoint_files[0])
            time.sleep(2)
            
            # 测试第二个文件
            self.test_waypoint_file(waypoint_files[1])
            time.sleep(2)
            
            # 测试第三个文件（如果存在）
            if len(waypoint_files) > 2:
                self.test_waypoint_file(waypoint_files[2])
            
            self.get_logger().info('✅ 路径点文件切换测试完成')
            
        except Exception as e:
            self.get_logger().error(f'❌ 测试失败: {e}')
    
    def get_waypoint_files(self):
        """获取可用的waypoint文件"""
        waypoint_dir = '/home/bd/Documents/Robot/agv_sim/waypoints'
        files = []
        
        if os.path.exists(waypoint_dir):
            for filename in os.listdir(waypoint_dir):
                if filename.endswith('.json'):
                    files.append(filename)
        
        return sorted(files)
    
    def test_waypoint_file(self, filename):
        """测试单个waypoint文件"""
        try:
            self.get_logger().info(f'🧪 测试文件: {filename}')
            
            # 构建完整路径
            full_path = f'/home/bd/Documents/Robot/agv_sim/waypoints/{filename}'
            
            # 检查文件是否存在
            if not os.path.exists(full_path):
                self.get_logger().error(f'❌ 文件不存在: {full_path}')
                return False
            
            # 读取文件内容
            with open(full_path, 'r') as f:
                data = json.load(f)
                waypoints = data.get('waypoints', [])
            
            self.get_logger().info(f'📊 文件包含 {len(waypoints)} 个路径点')
            
            # 设置文件路径
            request = SetString.Request()
            request.data = full_path
            
            future = self.set_file_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ 成功设置文件: {response.message}')
                
                # 显示前几个路径点信息
                for i, wp in enumerate(waypoints[:3]):
                    pos = wp.get('position', {})
                    self.get_logger().info(f'  路径点 {i}: x={pos.get("x", 0):.2f}, y={pos.get("y", 0):.2f}')
                
                return True
            else:
                self.get_logger().error(f'❌ 设置文件失败: {response.message}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ 测试文件 {filename} 失败: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WaypointSwitchTester()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
