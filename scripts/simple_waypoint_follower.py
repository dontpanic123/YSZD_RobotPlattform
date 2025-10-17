#!/usr/bin/env python3
"""
简单Waypoint Follower
直接控制机器人移动，不依赖Nav2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import json
import math
import time
import os
import signal
import sys
from std_srvs.srv import Empty
from std_msgs.msg import String

class SimpleWaypointFollower(Node):
    def __init__(self):
        super().__init__('simple_waypoint_follower')
        
        # 参数设置
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.6)  # 降低角速度
        self.declare_parameter('position_tolerance', 0.2)
        self.declare_parameter('orientation_tolerance', 0.15)  # 增加角度容差
        self.declare_parameter('smooth_control', True)
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('stop_during_turn', True)
        self.declare_parameter('turn_stop_angle_threshold', 1.047)  # 60度 = 1.047弧度
        
        # 获取参数
        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        self.smooth_control = self.get_parameter('smooth_control').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.stop_during_turn = self.get_parameter('stop_during_turn').value
        self.turn_stop_angle_threshold = self.get_parameter('turn_stop_angle_threshold').value
        
        # 状态变量
        self.waypoints = []
        self.current_waypoint_index = 0
        self.is_following = False
        self.current_pose = None
        self.shutdown_requested = False
        self.waypoint_start_time = None
        self.waypoint_timeout = 30.0  # 每个waypoint最多30秒
        self.last_angle_diff = None
        self.angle_stuck_count = 0
        self.angle_convergence_count = 0  # 角度收敛计数器
        self.min_angular_velocity = 0.05  # 最小角速度阈值
        
        # 创建发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.waypoint_markers_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        
        # 创建订阅器
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10
        )
        
        # 创建服务
        self.create_service(Empty, 'start_following', self.start_following_callback)
        self.create_service(Empty, 'stop_following', self.stop_following_callback)
        self.create_service(Empty, 'set_waypoints_file', self.set_waypoints_file_callback)
        # 使用参数设置服务来接收文件路径
        self.create_service(Empty, 'set_waypoints_file_path', self.set_waypoints_file_path_callback)
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('简单Waypoint Follower 已启动')
        
        # 如果指定了waypoints文件，自动加载
        if self.waypoints_file:
            self.load_waypoints(self.waypoints_file)
            # 发布waypoint可视化
            self.publish_waypoint_markers()
            
    def load_waypoints(self, filepath):
        """加载waypoints文件"""
        try:
            # 如果是相对路径，转换为绝对路径
            if not os.path.isabs(filepath):
                filepath = os.path.abspath(filepath)
            
            with open(filepath, 'r') as f:
                data = json.load(f)
                self.waypoints = data.get('waypoints', [])
                self.get_logger().info(f'加载了 {len(self.waypoints)} 个waypoints')
                
                # 打印前几个waypoints
                for i, wp in enumerate(self.waypoints[:3]):
                    pos = wp.get('position', {})
                    self.get_logger().info(f'Waypoint {i}: x={pos.get("x", 0):.2f}, y={pos.get("y", 0):.2f}')
                    
        except Exception as e:
            self.get_logger().error(f'加载waypoints文件失败: {e}')
            self.get_logger().error(f'文件路径: {filepath}')
            self.get_logger().error(f'当前工作目录: {os.getcwd()}')
            
    def odom_callback(self, msg):
        """里程计回调"""
        self.current_pose = msg.pose.pose
        
    def start_following_callback(self, request, response):
        """开始跟踪服务"""
        if not self.waypoints:
            self.get_logger().warn('没有加载waypoints，无法开始跟踪')
            return response
            
        self.is_following = True
        self.current_waypoint_index = 0
        self.get_logger().info('开始跟踪waypoints')
        # 更新可视化
        self.publish_waypoint_markers()
        return response
        
    def stop_following_callback(self, request, response):
        """停止跟踪服务"""
        self.is_following = False
        self.get_logger().info('停止跟踪waypoints')
        
        # 停止机器人
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        return response
        
    def set_waypoints_file_callback(self, request, response):
        """设置waypoints文件服务"""
        # 从参数中获取waypoints_file
        waypoints_file = self.get_parameter('waypoints_file').value
        if waypoints_file:
            self.get_logger().info(f'设置waypoints文件: {waypoints_file}')
            self.load_waypoints(waypoints_file)
            self.publish_waypoint_markers()
        else:
            self.get_logger().warn('未指定waypoints文件')
        return response
        
    def set_waypoints_file_path_callback(self, request, response):
        """设置waypoints文件路径服务"""
        # 从参数中获取文件路径
        filepath = self.get_parameter('waypoints_file').value
        if filepath:
            self.get_logger().info(f'设置waypoints文件路径: {filepath}')
            success = self.load_waypoints(filepath)
            if success:
                self.publish_waypoint_markers()
                self.get_logger().info(f'成功加载waypoints文件: {filepath}')
            else:
                self.get_logger().error(f'加载waypoints文件失败: {filepath}')
        else:
            self.get_logger().warn('未指定waypoints文件路径')
        return response
        
    def control_loop(self):
        """控制循环"""
        if self.shutdown_requested:
            return
            
        if not self.is_following or not self.waypoints or not self.current_pose:
            return
            
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('所有waypoints已完成')
            self.is_following = False
            
            # 发送停止命令，确保速度和旋转都归零
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.linear.y = 0.0
            stop_cmd.linear.z = 0.0
            stop_cmd.angular.x = 0.0
            stop_cmd.angular.y = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_cmd)
            
            self.get_logger().info('机器人已停止，所有运动命令已归零')
            return
            
        # 获取当前waypoint
        target_wp = self.waypoints[self.current_waypoint_index]
        target_pos = target_wp.get('position', {})
        target_orient = target_wp.get('orientation', {})
        
        # 检查waypoint超时
        current_time = time.time()
        if self.waypoint_start_time is None:
            self.waypoint_start_time = current_time
        elif current_time - self.waypoint_start_time > self.waypoint_timeout:
            self.get_logger().warn(f'Waypoint {self.current_waypoint_index} 超时，跳过')
            self.current_waypoint_index += 1
            self.waypoint_start_time = current_time
            self.publish_waypoint_markers()
            return
        
        # 计算目标位置
        target_x = target_pos.get('x', 0)
        target_y = target_pos.get('y', 0)
        
        # 获取当前位置
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # 计算距离和角度
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 计算目标角度
        target_angle = math.atan2(dy, dx)
        current_angle = self.quaternion_to_yaw(self.current_pose.orientation)
        angle_diff = self.normalize_angle(target_angle - current_angle)
        
        # 调试信息
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 20 == 0:  # 每20次输出一次调试信息
            self.get_logger().info(f'调试: 目标角度={target_angle:.2f}, 当前角度={current_angle:.2f}, 角度差={angle_diff:.2f}')
        
        # 检测角度是否卡住 - 更合理的检测条件
        # 只在以下情况检测：1) 距离很近 2) 角度差很小 3) 正在转向但角度不变
        if (self.last_angle_diff is not None and 
            distance < 0.5 and  # 距离小于0.5米
            abs(angle_diff) < 0.15 and  # 角度差小于0.15弧度(约8.6度)
            abs(angle_diff - self.last_angle_diff) < 0.005):  # 角度差变化很小
            
            self.angle_stuck_count += 1
            if self.angle_stuck_count > 200:  # 更保守的检测次数
                self.get_logger().warn(f'角度卡住检测: 近距离小角度差={angle_diff:.3f} 持续不变，强制跳过waypoint')
                self.current_waypoint_index += 1
                self.waypoint_start_time = None
                self.angle_stuck_count = 0
                self.angle_convergence_count = 0
                self.publish_waypoint_markers()
                return
        else:
            self.angle_stuck_count = 0
            
        # 检测角度收敛 - 防止无限旋转
        if abs(angle_diff) < self.orientation_tolerance:
            self.angle_convergence_count += 1
            if self.angle_convergence_count > 50:  # 角度已收敛50次循环
                self.get_logger().info(f'角度已收敛，当前角度差={angle_diff:.3f}，允许前进')
                # 强制设置角速度为0，允许前进
                if self.smooth_control:
                    cmd = self.smooth_control_logic(target_x, target_y, distance, 0.0)  # 传入0角度差
                else:
                    cmd = self.basic_control_logic(distance, 0.0)  # 传入0角度差
                self.cmd_vel_pub.publish(cmd)
                return
        else:
            self.angle_convergence_count = 0
            
        self.last_angle_diff = angle_diff
        
        # 平滑控制逻辑
        if self.smooth_control:
            cmd = self.smooth_control_logic(target_x, target_y, distance, angle_diff)
        else:
            cmd = self.basic_control_logic(distance, angle_diff)
            
        # 发布控制命令
        self.cmd_vel_pub.publish(cmd)
        
    def smooth_control_logic(self, target_x, target_y, distance, angle_diff):
        """平滑控制逻辑"""
        cmd = Twist()
        
        # 检查是否还有更多waypoint
        has_more_waypoints = self.current_waypoint_index < len(self.waypoints) - 1
        
        # 计算到下一个waypoint的距离（如果存在）
        next_distance = float('inf')
        if has_more_waypoints:
            next_wp = self.waypoints[self.current_waypoint_index + 1]
            next_pos = next_wp.get('position', {})
            next_x = next_pos.get('x', 0)
            next_y = next_pos.get('y', 0)
            next_dx = next_x - self.current_pose.position.x
            next_dy = next_y - self.current_pose.position.y
            next_distance = math.sqrt(next_dx*next_dx + next_dy*next_dy)
        
        # 判断是否应该减速
        should_slow_down = (not has_more_waypoints or 
                           (distance < self.position_tolerance * 2 and next_distance > self.lookahead_distance))
        
        # 计算目标速度
        if should_slow_down:
            # 接近目标时减速
            speed_factor = min(1.0, distance / (self.position_tolerance * 3))
            target_linear_speed = self.linear_speed * speed_factor
        else:
            # 保持正常速度
            target_linear_speed = self.linear_speed
        
        # 改进的角度控制 - 渐进式控制
        if abs(angle_diff) > self.orientation_tolerance:
            # 需要转向 - 使用渐进式角速度控制
            # 角度差大时使用较大角速度，角度差小时使用较小角速度
            if abs(angle_diff) > 0.5:  # 角度差大于28.6度
                angular_velocity = min(self.angular_speed, abs(angle_diff) * 0.8)
            elif abs(angle_diff) > 0.2:  # 角度差大于11.5度
                angular_velocity = min(self.angular_speed * 0.6, abs(angle_diff) * 1.2)
            else:  # 角度差小于11.5度
                angular_velocity = min(self.angular_speed * 0.3, abs(angle_diff) * 2.0)
            
            if angle_diff > 0:
                cmd.angular.z = angular_velocity
            else:
                cmd.angular.z = -angular_velocity
                
            # 转向时停止前进的逻辑 - 只有角度差大于阈值时才停止
            if self.stop_during_turn and abs(angle_diff) > self.turn_stop_angle_threshold:
                # 角度差大于60度时完全停止前进，避免转圈
                cmd.linear.x = 0.0
                self.get_logger().info(f'大角度转向({abs(angle_diff)*180/3.14159:.1f}度)，停止前进')
            elif self.stop_during_turn:
                # 小角度转向时适当减速
                cmd.linear.x = target_linear_speed * 0.3
                self.get_logger().info(f'小角度转向({abs(angle_diff)*180/3.14159:.1f}度)，减速前进')
            else:
                # 如果允许转向时前进，则大幅减速
                cmd.linear.x = target_linear_speed * 0.1
        else:
            # 朝向正确，正常前进
            cmd.linear.x = target_linear_speed
            cmd.angular.z = 0.0
        
        # 检查是否到达当前waypoint
        if distance <= self.position_tolerance:
            self.get_logger().info(f'到达waypoint {self.current_waypoint_index}')
            self.current_waypoint_index += 1
            self.waypoint_start_time = None
            self.publish_waypoint_markers()
            
            # 检查是否所有waypoint都已完成
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('所有waypoints已完成，发送停止命令')
                self.is_following = False
                
                # 发送停止命令
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.linear.y = 0.0
                stop_cmd.linear.z = 0.0
                stop_cmd.angular.x = 0.0
                stop_cmd.angular.y = 0.0
                stop_cmd.angular.z = 0.0
                return stop_cmd
        
        return cmd
    
    def basic_control_logic(self, distance, angle_diff):
        """基础控制逻辑（原始逻辑）"""
        cmd = Twist()
        
        if distance > self.position_tolerance:
            # 需要移动到目标位置
            if abs(angle_diff) > self.orientation_tolerance:
                # 先转向目标方向 - 使用渐进式控制
                if abs(angle_diff) > 0.5:  # 角度差大于28.6度
                    angular_velocity = min(self.angular_speed, abs(angle_diff) * 0.8)
                elif abs(angle_diff) > 0.2:  # 角度差大于11.5度
                    angular_velocity = min(self.angular_speed * 0.6, abs(angle_diff) * 1.2)
                else:  # 角度差小于11.5度
                    angular_velocity = min(self.angular_speed * 0.3, abs(angle_diff) * 2.0)
                
                if angle_diff > 0:
                    cmd.angular.z = angular_velocity
                else:
                    cmd.angular.z = -angular_velocity
                
                # 转向时停止前进的逻辑 - 只有角度差大于阈值时才停止
                if self.stop_during_turn and abs(angle_diff) > self.turn_stop_angle_threshold:
                    # 角度差大于60度时完全停止前进，避免转圈
                    cmd.linear.x = 0.0
                    self.get_logger().info(f'大角度转向({abs(angle_diff)*180/3.14159:.1f}度)，停止前进')
                elif self.stop_during_turn:
                    # 小角度转向时适当减速
                    cmd.linear.x = self.linear_speed * 0.3
                    self.get_logger().info(f'小角度转向({abs(angle_diff)*180/3.14159:.1f}度)，减速前进')
                else:
                    # 如果允许转向时前进，则大幅减速
                    cmd.linear.x = self.linear_speed * 0.1
            else:
                # 朝向正确，向前移动
                cmd.linear.x = self.linear_speed
        else:
            # 到达目标位置
            self.get_logger().info(f'到达waypoint {self.current_waypoint_index}')
            self.current_waypoint_index += 1
            self.waypoint_start_time = None
            self.publish_waypoint_markers()
            
            # 检查是否所有waypoint都已完成
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('所有waypoints已完成，发送停止命令')
                self.is_following = False
                
                # 发送停止命令
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.linear.y = 0.0
                stop_cmd.linear.z = 0.0
                stop_cmd.angular.x = 0.0
                stop_cmd.angular.y = 0.0
                stop_cmd.angular.z = 0.0
                return stop_cmd
        
        return cmd
        
    def quaternion_to_yaw(self, quaternion):
        """四元数转yaw角"""
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw
        
    def normalize_angle(self, angle):
        """角度归一化"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def publish_waypoint_markers(self):
        """发布waypoint可视化标记"""
        if not self.waypoints:
            return
            
        marker_array = MarkerArray()
        
        for i, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 设置位置
            pos = waypoint.get('position', {})
            marker.pose.position.x = pos.get('x', 0.0)
            marker.pose.position.y = pos.get('y', 0.0)
            marker.pose.position.z = pos.get('z', 0.0)
            
            # 设置方向
            orient = waypoint.get('orientation', {})
            marker.pose.orientation.x = orient.get('x', 0.0)
            marker.pose.orientation.y = orient.get('y', 0.0)
            marker.pose.orientation.z = orient.get('z', 0.0)
            marker.pose.orientation.w = orient.get('w', 1.0)
            
            # 设置大小
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # 根据是否已到达设置颜色
            if i < self.current_waypoint_index:
                # 已到达的waypoints - 绿色
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            elif i == self.current_waypoint_index:
                # 当前waypoint - 黄色
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
            else:
                # 未到达的waypoints - 红色
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            
            marker_array.markers.append(marker)
            
        # 发布标记
        self.waypoint_markers_pub.publish(marker_array)

def signal_handler(signum, frame):
    """信号处理器"""
    print("\n收到中断信号，正在停止...")
    sys.exit(0)

def main(args=None):
    # 设置信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    rclpy.init(args=args)
    node = SimpleWaypointFollower()
    
    try:
        # 使用spin_once来避免阻塞，允许键盘中断
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.05)  # 减少超时时间
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断，正在停止...')
        node.shutdown_requested = True
        # 停止机器人
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.get_logger().info('机器人已停止')
    except Exception as e:
        node.get_logger().error(f'发生错误: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
