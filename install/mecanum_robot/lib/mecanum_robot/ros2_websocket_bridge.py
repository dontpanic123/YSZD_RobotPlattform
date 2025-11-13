#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import asyncio
import websockets
from websockets.server import serve
import threading
import time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64
import numpy as np

class ROS2WebSocketBridge(Node):
    def __init__(self):
        super().__init__('ros2_websocket_bridge')
        
        # WebSocket服务器
        self.websocket_server = None
        self.connected_clients = set()
        self.server_thread = None
        
        # 控制状态管理
        self.manual_control_active = False  # 手动控制是否激活
        self.last_manual_control_time = 0   # 最后手动控制时间
        self.manual_control_timeout = 0.5  # 手动控制超时时间（秒）
        
        # ROS2发布者和订阅者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 订阅话题
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.apriltag_detection_sub = self.create_subscription(
            Image, '/apriltag_detection', self.apriltag_detection_callback, 10
        )
        self.apriltag_pose_sub = self.create_subscription(
            PoseStamped, '/apriltag_pose', self.apriltag_pose_callback, 10
        )
        self.apriltag_status_sub = self.create_subscription(
            String, '/apriltag_status', self.apriltag_status_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # OpenCV桥接
        self.bridge = CvBridge()
        
        # 启动WebSocket服务器
        self.start_websocket_server()
        
        # 启动手动控制状态检查定时器
        self.control_timer = self.create_timer(0.1, self.check_manual_control_status)
        
        self.get_logger().info('ROS2 WebSocket Bridge 已启动')
        self.get_logger().info('WebSocket服务器: ws://localhost:9090')
    
    def start_websocket_server(self):
        """启动WebSocket服务器"""
        def run_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.websocket_handler())
        
        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()
    
    async def websocket_handler(self):
        """WebSocket处理器"""
        async with serve(self.handle_client, "localhost", 9090):
            self.get_logger().info('WebSocket服务器已启动在端口 9090')
            await asyncio.Future()  # 保持服务器运行
    
    async def handle_client(self, websocket, path):
        """处理WebSocket客户端连接"""
        self.connected_clients.add(websocket)
        self.get_logger().info(f'客户端已连接: {websocket.remote_address}')
        
        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            if websocket in self.connected_clients:
                self.connected_clients.remove(websocket)
            self.get_logger().info(f'客户端已断开: {websocket.remote_address}')
    
    async def handle_message(self, websocket, message):
        """处理WebSocket消息"""
        try:
            data = json.loads(message)
            msg_type = data.get('type')
            
            if msg_type == 'cmd_vel':
                # 处理速度控制命令
                twist = Twist()
                twist.linear.x = float(data.get('linear_x', 0.0))
                twist.linear.y = float(data.get('linear_y', 0.0))
                twist.linear.z = float(data.get('linear_z', 0.0))
                twist.angular.x = float(data.get('angular_x', 0.0))
                twist.angular.y = float(data.get('angular_y', 0.0))
                twist.angular.z = float(data.get('angular_z', 0.0))
                
                # 检查是否有非零速度命令
                has_velocity = (abs(twist.linear.x) > 0.001 or 
                              abs(twist.linear.y) > 0.001 or 
                              abs(twist.linear.z) > 0.001 or
                              abs(twist.angular.x) > 0.001 or 
                              abs(twist.angular.y) > 0.001 or 
                              abs(twist.angular.z) > 0.001)
                
                if has_velocity:
                    # 有速度命令，激活手动控制
                    self.manual_control_active = True
                    self.last_manual_control_time = time.time()
                    self.cmd_vel_pub.publish(twist)
                    self.get_logger().debug(f'发布速度命令: {twist}')
                else:
                    # 零速度命令，检查是否应该发送归零命令
                    current_time = time.time()
                    if (self.manual_control_active and 
                        current_time - self.last_manual_control_time < self.manual_control_timeout):
                        # 在超时时间内，发送归零命令防止漂移
                        self.cmd_vel_pub.publish(twist)
                        self.get_logger().debug(f'发送归零命令防止漂移: {twist}')
                    else:
                        # 超时或未激活手动控制，不发送任何命令
                        self.manual_control_active = False
                        self.get_logger().debug('跳过零速度命令，避免与follower冲突')
                
            elif msg_type == 'goal_pose':
                # 处理目标点设置
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(data.get('x', 0.0))
                pose.pose.position.y = float(data.get('y', 0.0))
                pose.pose.position.z = float(data.get('z', 0.0))
                pose.pose.orientation.w = 1.0
                
                self.goal_pose_pub.publish(pose)
                self.get_logger().info(f'设置目标点: ({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z})')
                
            elif msg_type == 'ping':
                # 响应ping消息
                await websocket.send(json.dumps({'type': 'pong'}))
                
        except json.JSONDecodeError:
            self.get_logger().error(f'无效的JSON消息: {message}')
        except Exception as e:
            self.get_logger().error(f'处理消息时出错: {e}')
    
    def image_callback(self, msg):
        """处理摄像头图像"""
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 压缩图像
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            img_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # 发送到WebSocket客户端
            data = {
                'type': 'camera_image',
                'data': img_base64,
                'width': msg.width,
                'height': msg.height,
                'encoding': 'jpeg'
            }
            
            self.broadcast_to_clients(data)
            
        except Exception as e:
            self.get_logger().error(f'处理摄像头图像时出错: {e}')
    
    def apriltag_detection_callback(self, msg):
        """处理AprilTag检测图像"""
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 压缩图像
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            img_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # 发送到WebSocket客户端
            data = {
                'type': 'apriltag_detection',
                'data': img_base64,
                'width': msg.width,
                'height': msg.height,
                'encoding': 'jpeg'
            }
            
            self.broadcast_to_clients(data)
            
        except Exception as e:
            self.get_logger().error(f'处理AprilTag检测图像时出错: {e}')
    
    def apriltag_pose_callback(self, msg):
        """处理AprilTag位姿"""
        try:
            data = {
                'type': 'apriltag_pose',
                'position': {
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': msg.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.orientation.x,
                    'y': msg.pose.orientation.y,
                    'z': msg.pose.orientation.z,
                    'w': msg.pose.orientation.w
                }
            }
            
            self.broadcast_to_clients(data)
            
        except Exception as e:
            self.get_logger().error(f'处理AprilTag位姿时出错: {e}')
    
    def apriltag_status_callback(self, msg):
        """处理AprilTag状态"""
        try:
            data = {
                'type': 'apriltag_status',
                'status': msg.data
            }
            
            self.broadcast_to_clients(data)
            
        except Exception as e:
            self.get_logger().error(f'处理AprilTag状态时出错: {e}')
    
    def odom_callback(self, msg):
        """处理里程计数据"""
        try:
            data = {
                'type': 'odom',
                'header': {
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': msg.header.frame_id
                },
                'child_frame_id': msg.child_frame_id,
                'pose': {
                    'pose': {
                        'position': {
                            'x': msg.pose.pose.position.x,
                            'y': msg.pose.pose.position.y,
                            'z': msg.pose.pose.position.z
                        },
                        'orientation': {
                            'x': msg.pose.pose.orientation.x,
                            'y': msg.pose.pose.orientation.y,
                            'z': msg.pose.pose.orientation.z,
                            'w': msg.pose.pose.orientation.w
                        }
                    },
                    'covariance': list(msg.pose.covariance)
                },
                'twist': {
                    'twist': {
                        'linear': {
                            'x': msg.twist.twist.linear.x,
                            'y': msg.twist.twist.linear.y,
                            'z': msg.twist.twist.linear.z
                        },
                        'angular': {
                            'x': msg.twist.twist.angular.x,
                            'y': msg.twist.twist.angular.y,
                            'z': msg.twist.twist.angular.z
                        }
                    },
                    'covariance': list(msg.twist.covariance)
                }
            }
            
            self.broadcast_to_clients(data)
            
        except Exception as e:
            self.get_logger().error(f'处理里程计数据时出错: {e}')
    
    def broadcast_to_clients(self, data):
        """向所有客户端广播数据"""
        if not self.connected_clients:
            return
        
        message = json.dumps(data)
        
        # 在单独的线程中发送消息
        def send_message():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                for client in self.connected_clients.copy():
                    try:
                        loop.run_until_complete(client.send(message))
                    except Exception as e:
                        self.get_logger().debug(f'发送消息到客户端时出错: {e}')
                        self.connected_clients.discard(client)
            finally:
                loop.close()
        
        threading.Thread(target=send_message, daemon=True).start()
    
    def check_manual_control_status(self):
        """检查手动控制状态，超时后自动停用手动控制"""
        current_time = time.time()
        if (self.manual_control_active and 
            current_time - self.last_manual_control_time > self.manual_control_timeout):
            self.manual_control_active = False
            self.get_logger().debug('手动控制超时，停用手动控制模式')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ROS2WebSocketBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
