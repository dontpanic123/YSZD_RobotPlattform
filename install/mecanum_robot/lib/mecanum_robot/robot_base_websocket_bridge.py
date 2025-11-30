#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import asyncio
import websockets
from websockets.asyncio.client import connect
import socket
import sys
import os
import threading
import time
import math

# Import Protocol Buffers generated files
# Use the original location which has proper package structure
sys.path.insert(0, '/home/y1234/Documents/robot_ws/robot-demos-main/python')
from generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2

ACCEPTABLE_PROTOCOL_MAJOR_VERSION = 1


class RobotBaseWebSocketBridge(Node):
    def __init__(self):
        super().__init__('robot_base_websocket_bridge')
        
        # Declare parameters
        self.declare_parameter('websocket_url', 'ws://localhost:8439')
        self.declare_parameter('publish_odometry', True)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('max_reconnect_attempts', 10)
        
        # Get parameters
        self.websocket_url = self.get_parameter('websocket_url').get_parameter_value().string_value
        self.publish_odometry = self.get_parameter('publish_odometry').get_parameter_value().bool_value
        self.command_timeout = self.get_parameter('command_timeout').get_parameter_value().double_value
        self.reconnect_interval = self.get_parameter('reconnect_interval').get_parameter_value().double_value
        self.max_reconnect_attempts = self.get_parameter('max_reconnect_attempts').get_parameter_value().integer_value
        
        # ROS2 publishers and subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        if self.publish_odometry:
            self.odom_pub = self.create_publisher(
                Odometry,
                '/odom',
                10
            )
        
        # WebSocket connection state
        self.websocket = None
        self.connected = False
        self.reconnect_attempts = 0
        self.last_command_time = 0.0
        self.current_twist = Twist()
        
        # Threading for async WebSocket
        self.loop = None
        self.loop_thread = None
        self.shutdown_flag = False
        
        # Start WebSocket connection in a separate thread
        self.start_websocket_thread()
        
        self.get_logger().info(f'Robot Base WebSocket Bridge started')
        self.get_logger().info(f'WebSocket URL: {self.websocket_url}')
        self.get_logger().info(f'Publish odometry: {self.publish_odometry}')
    
    def start_websocket_thread(self):
        """Start the async event loop in a separate thread"""
        def run_loop():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self.websocket_manager())
        
        self.loop_thread = threading.Thread(target=run_loop, daemon=True)
        self.loop_thread.start()
    
    async def websocket_manager(self):
        """Manage WebSocket connection with reconnection logic"""
        while not self.shutdown_flag:
            try:
                await self.connect_and_run()
            except Exception as e:
                self.get_logger().error(f'WebSocket error: {e}')
            
            if self.shutdown_flag:
                break
            
            # Reconnection logic
            if self.reconnect_attempts < self.max_reconnect_attempts:
                self.reconnect_attempts += 1
                self.get_logger().warn(
                    f'Attempting to reconnect ({self.reconnect_attempts}/{self.max_reconnect_attempts})...'
                )
                await asyncio.sleep(self.reconnect_interval)
            else:
                self.get_logger().error('Max reconnection attempts reached. Stopping.')
                break
    
    async def connect_and_run(self):
        """Connect to WebSocket and run the main loop"""
        self.get_logger().info(f'Connecting to {self.websocket_url}...')
        
        try:
            async with connect(self.websocket_url) as websocket:
                # Set TCP_NODELAY for better performance
                try:
                    sock = websocket.transport.get_extra_info('socket')
                    if sock:
                        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except Exception as e:
                    self.get_logger().warn(f'Could not set TCP_NODELAY: {e}')
                
                self.websocket = websocket
                self.connected = True
                self.reconnect_attempts = 0
                self.get_logger().info('WebSocket connected successfully')
                
                # Initialize the base
                await self.initialize_base()
                
                # Set report frequency to 50Hz
                await self.set_report_frequency()
                
                # Start receiving and sending tasks
                receive_task = asyncio.create_task(self.receive_messages())
                send_task = asyncio.create_task(self.send_commands_loop())
                
                try:
                    await asyncio.gather(receive_task, send_task)
                except asyncio.CancelledError:
                    pass
                finally:
                    receive_task.cancel()
                    send_task.cancel()
                    await asyncio.gather(receive_task, send_task, return_exceptions=True)
                    
                    # Deinitialize the base
                    await self.deinitialize_base()
                    
        except Exception as e:
            self.connected = False
            self.get_logger().error(f'WebSocket connection error: {e}')
            raise
    
    async def initialize_base(self):
        """Send initialization command to the base"""
        try:
            api_down = public_api_down_pb2.APIDown()
            api_down.base_command.api_control_initialize = True
            await self.websocket.send(api_down.SerializeToString())
            self.get_logger().info('Base initialization command sent')
            await asyncio.sleep(0.1)  # Small delay after initialization
        except Exception as e:
            self.get_logger().error(f'Failed to initialize base: {e}')
    
    async def deinitialize_base(self):
        """Send deinitialization command to the base"""
        try:
            if self.websocket:
                api_down = public_api_down_pb2.APIDown()
                api_down.base_command.api_control_initialize = False
                await self.websocket.send(api_down.SerializeToString())
                self.get_logger().info('Base deinitialization command sent')
        except Exception as e:
            self.get_logger().error(f'Failed to deinitialize base: {e}')
    
    async def set_report_frequency(self):
        """Set report frequency to 50Hz"""
        try:
            api_down = public_api_down_pb2.APIDown()
            api_down.set_report_frequency = public_api_types_pb2.ReportFrequency.Rf50Hz
            await self.websocket.send(api_down.SerializeToString())
            self.get_logger().info('Report frequency set to 50Hz')
            await asyncio.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f'Failed to set report frequency: {e}')
    
    async def receive_messages(self):
        """Receive and process messages from the base"""
        try:
            async for message in self.websocket:
                if isinstance(message, bytes):
                    await self.process_base_message(message)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            self.get_logger().error(f'Error receiving messages: {e}')
    
    async def process_base_message(self, message_bytes):
        """Process a binary message from the base"""
        try:
            api_up = public_api_up_pb2.APIUp()
            api_up.ParseFromString(message_bytes)
            
            # Check for log messages
            if api_up.HasField("log"):
                self.get_logger().warn(f'Log from base: {api_up.log}')
            
            # Check protocol version
            if api_up.protocol_major_version != ACCEPTABLE_PROTOCOL_MAJOR_VERSION:
                self.get_logger().warn(
                    f'Protocol major version mismatch: expected {ACCEPTABLE_PROTOCOL_MAJOR_VERSION}, '
                    f'got {api_up.protocol_major_version}'
                )
            
            # Process base status and odometry
            if api_up.HasField('base_status'):
                base_status = api_up.base_status
                
                if base_status.HasField('estimated_odometry') and self.publish_odometry:
                    await self.publish_odometry_from_base(base_status.estimated_odometry)
                    
        except Exception as e:
            self.get_logger().error(f'Error processing base message: {e}')
    
    async def publish_odometry_from_base(self, odometry):
        """Convert and publish odometry from base to ROS2"""
        try:
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            # Position
            odom_msg.pose.pose.position.x = float(odometry.pos_x)
            odom_msg.pose.pose.position.y = float(odometry.pos_y)
            odom_msg.pose.pose.position.z = float(odometry.pos_z)
            
            # Orientation (assuming 2D, z-axis rotation from pos_z)
            # If pos_z is the angle, convert to quaternion
            theta = float(odometry.pos_z)
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
            odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
            
            # Velocity
            odom_msg.twist.twist.linear.x = float(odometry.speed_x)
            odom_msg.twist.twist.linear.y = float(odometry.speed_y)
            odom_msg.twist.twist.angular.z = float(odometry.speed_z)
            
            self.odom_pub.publish(odom_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {e}')
    
    def cmd_vel_callback(self, msg):
        """Callback for /cmd_vel topic"""
        self.current_twist = msg
        self.last_command_time = time.time()
        
        # Schedule command to be sent via async loop
        if self.loop and self.connected:
            asyncio.run_coroutine_threadsafe(
                self.send_move_command(msg),
                self.loop
            )
    
    async def send_move_command(self, twist):
        """Convert Twist to Protocol Buffers and send to base"""
        try:
            if not self.connected or not self.websocket:
                return
            
            api_down = public_api_down_pb2.APIDown()
            api_down.base_command.simple_move_command.xyz_speed.speed_x = float(twist.linear.x)
            api_down.base_command.simple_move_command.xyz_speed.speed_y = float(twist.linear.y)
            api_down.base_command.simple_move_command.xyz_speed.speed_z = float(twist.angular.z)
            
            await self.websocket.send(api_down.SerializeToString())
            
        except Exception as e:
            self.get_logger().error(f'Error sending move command: {e}')
    
    async def send_commands_loop(self):
        """Periodically send commands at 50Hz (every 20ms)"""
        try:
            while self.connected and not self.shutdown_flag:
                # Check if we have a recent command
                current_time = time.time()
                if current_time - self.last_command_time < self.command_timeout:
                    # Send the current command
                    await self.send_move_command(self.current_twist)
                else:
                    # Send zero command if timeout
                    zero_twist = Twist()
                    await self.send_move_command(zero_twist)
                
                await asyncio.sleep(0.02)  # 50Hz = 20ms
                
        except asyncio.CancelledError:
            pass
        except Exception as e:
            self.get_logger().error(f'Error in send commands loop: {e}')
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down Robot Base WebSocket Bridge...')
        self.shutdown_flag = True
        self.connected = False
        
        # Wait for async tasks to complete
        if self.loop and not self.loop.is_closed():
            # Schedule deinitialization
            try:
                asyncio.run_coroutine_threadsafe(
                    self.deinitialize_base(),
                    self.loop
                )
            except Exception as e:
                self.get_logger().error(f'Error during shutdown: {e}')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotBaseWebSocketBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

