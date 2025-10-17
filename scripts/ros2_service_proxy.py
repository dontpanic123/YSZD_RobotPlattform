#!/usr/bin/env python3
"""
ROS2服务代理
提供HTTP接口来调用ROS2服务
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Empty
from rcl_interfaces.srv import SetParameters
import json
import threading
import os
import glob
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse

class ROS2ServiceProxy(Node):
    def __init__(self):
        super().__init__('ros2_service_proxy')
        
        # 创建服务客户端
        self.start_recording_client = self.create_client(Empty, '/start_recording')
        self.stop_recording_client = self.create_client(Empty, '/stop_recording')
        self.save_waypoints_client = self.create_client(Empty, '/save_waypoints')
        self.start_following_client = self.create_client(Empty, '/start_following')
        self.stop_following_client = self.create_client(Empty, '/stop_following')
        self.set_waypoints_file_client = self.create_client(Empty, '/set_waypoints_file')
        
        # 等待服务可用
        self.get_logger().info('等待ROS2服务可用...')
        self.wait_for_services()
        
        # 启动HTTP服务器
        self.start_http_server()
    
    def wait_for_services(self):
        """等待所有服务可用"""
        services = [
            (self.start_recording_client, '/start_recording'),
            (self.stop_recording_client, '/stop_recording'),
            (self.save_waypoints_client, '/save_waypoints'),
            (self.start_following_client, '/start_following'),
            (self.stop_following_client, '/stop_following'),
            (self.set_waypoints_file_client, '/set_waypoints_file')
        ]
        
        for client, name in services:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f'服务 {name} 不可用')
            else:
                self.get_logger().info(f'服务 {name} 已就绪')
    
    def start_http_server(self):
        """启动HTTP服务器"""
        def run_server():
            server = HTTPServer(('localhost', 8081), ServiceHandler)
            server.ros2_node = self
            self.get_logger().info('HTTP服务器启动在 localhost:8081')
            server.serve_forever()
        
        thread = threading.Thread(target=run_server, daemon=True)
        thread.start()
    
    def call_ros2_service(self, service_name, args=None):
        """调用ROS2服务"""
        try:
            if service_name == '/start_recording':
                future = self.start_recording_client.call_async(Empty.Request())
            elif service_name == '/stop_recording':
                future = self.stop_recording_client.call_async(Empty.Request())
            elif service_name == '/save_waypoints':
                future = self.save_waypoints_client.call_async(Empty.Request())
            elif service_name == '/start_following':
                # 处理waypoints_file参数
                if args and 'waypoints_file' in args:
                    waypoints_file = args['waypoints_file']
                    self.get_logger().info(f'开始跟踪waypoints文件: {waypoints_file}')
                    # 设置参数
                    self.set_parameter(Parameter('waypoints_file', waypoints_file))
                    # 首先设置waypoints文件
                    set_future = self.set_waypoints_file_client.call_async(Empty.Request())
                    rclpy.spin_until_future_complete(self, set_future, timeout_sec=2.0)
                    if set_future.done():
                        self.get_logger().info('Waypoints文件已设置')
                    else:
                        self.get_logger().warn('设置waypoints文件超时')
                future = self.start_following_client.call_async(Empty.Request())
            elif service_name == '/stop_following':
                future = self.stop_following_client.call_async(Empty.Request())
            elif service_name == '/set_waypoints_file_path':
                # 处理waypoints文件路径参数
                if args and 'file_path' in args:
                    file_path = args['file_path']
                    # 如果只是文件名，添加完整路径
                    if not os.path.isabs(file_path) and not file_path.startswith('/'):
                        # 构建完整的waypoints文件路径
                        full_path = os.path.join('/home/bd/Documents/Robot/agv_sim/waypoints', file_path)
                        self.get_logger().info(f'转换文件路径: {file_path} -> {full_path}')
                        file_path = full_path
                    
                    self.get_logger().info(f'设置waypoints文件路径: {file_path}')
                    # 通过参数客户端设置参数
                    param_client = self.create_client(SetParameters, '/simple_waypoint_follower/set_parameters')
                    if param_client.wait_for_service(timeout_sec=2.0):
                        param_request = SetParameters.Request()
                        param_request.parameters = [Parameter(name='waypoints_file', value=file_path).to_parameter_msg()]
                        param_future = param_client.call_async(param_request)
                        rclpy.spin_until_future_complete(self, param_future, timeout_sec=2.0)
                        if param_future.done():
                            self.get_logger().info('参数设置成功')
                        else:
                            self.get_logger().warn('参数设置超时')
                future = self.set_waypoints_file_client.call_async(Empty.Request())
            else:
                return {'error': f'未知服务: {service_name}'}
            
            # 等待服务调用完成
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                result = future.result()
                return {'success': True, 'result': str(result)}
            else:
                return {'error': '服务调用超时'}
                
        except Exception as e:
            return {'error': str(e)}
    
    def get_waypoint_files(self):
        """获取waypoints目录中的文件列表"""
        try:
            waypoints_dir = '/home/bd/Documents/Robot/agv_sim/waypoints'
            if not os.path.exists(waypoints_dir):
                return {'error': f'Waypoints目录不存在: {waypoints_dir}'}
            
            # 获取所有.json文件
            pattern = os.path.join(waypoints_dir, '*.json')
            files = glob.glob(pattern)
            
            # 只返回文件名，不包含路径
            filenames = [os.path.basename(f) for f in files]
            filenames.sort()  # 按文件名排序
            
            self.get_logger().info(f'找到 {len(filenames)} 个waypoint文件')
            return {'success': True, 'files': filenames}
            
        except Exception as e:
            self.get_logger().error(f'获取waypoint文件列表失败: {e}')
            return {'error': str(e)}

class ServiceHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/ros2_service':
            try:
                # 读取请求数据
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))
                
                service_name = data.get('service')
                args = data.get('args', {})
                
                # 调用ROS2服务
                result = self.server.ros2_node.call_ros2_service(service_name, args)
                
                # 发送响应
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps(result).encode('utf-8'))
                
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                error_response = {'error': str(e)}
                self.wfile.write(json.dumps(error_response).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()
    
    def do_GET(self):
        if self.path == '/waypoint_files':
            try:
                # 获取waypoint文件列表
                result = self.server.ros2_node.get_waypoint_files()
                
                # 发送响应
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps(result).encode('utf-8'))
                
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                error_response = {'error': str(e)}
                self.wfile.write(json.dumps(error_response).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()
    
    def do_OPTIONS(self):
        # 处理CORS预检请求
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

def main(args=None):
    rclpy.init(args=args)
    node = ROS2ServiceProxy()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
