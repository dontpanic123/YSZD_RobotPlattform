#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

class MecanumRobotNode(Node):
    def __init__(self):
        super().__init__('mecanum_robot_node')
        
        # 舵轮参数
        self.wheel_radius = 0.0625  # 轮子半径 (m) - 从直径125mm计算
        self.wheel_base = 0.383     # 轮距 (m) - 前后轮距离 (508mm - 2*62.5mm)
        self.wheel_track = 0.423     # 轴距 (m) - 左右轮距离 (548mm - 2*62.5mm)
        
        # 安全参数
        self.max_linear_speed = 2.0  # 最大线速度 (m/s) - 根据规格
        self.max_angular_speed = 1.0  # 最大角速度 (rad/s)
        self.max_steering_angle = math.pi / 2.0  # 最大转向角 ±90°
        self.command_timeout = 0.5  # 命令超时时间 (s)
        
        # 平滑参数
        self.steering_smooth_factor = 0.5  # 转向角平滑系数 (0-1, 越大响应越快) - 提高响应速度
        self.velocity_smooth_factor = 0.5  # 速度平滑系数
        
        # 主动轮状态 (左前轮和右后轮)
        # W1: 左前轮转向角, W2: 右后轮转向角
        # V1: 左前轮转速, V2: 右后轮转速
        self.steering_angles = [0.0, 0.0]  # [W1 (front_left), W2 (rear_right)]
        self.wheel_velocities = [0.0, 0.0]  # [V1 (front_left), V2 (rear_right)]
        self.steering_positions = [0.0, 0.0]  # 转向角位置
        self.wheel_positions = [0.0, 0.0]     # 轮子位置
        
        # 被动轮状态 (右前轮和左后轮)
        self.passive_steering_angles = [0.0, 0.0]  # [front_right, rear_left]
        self.passive_wheel_velocities = [0.0, 0.0]  # [front_right, rear_left]
        self.passive_steering_positions = [0.0, 0.0]
        self.passive_wheel_positions = [0.0, 0.0]
        
        # 机器人位置和姿态 (初始位置在原点)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # 命令超时管理
        self.last_command_time = self.get_clock().now()
        
        # 创建订阅者和发布者
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        
        self.joint_state_pub = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry, 
            '/odom', 
            10
        )
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.odom_timer = self.create_timer(0.2, self.publish_odometry)  # 减少频率从0.05到0.2
        self.tf_timer = self.create_timer(0.1, self.publish_tf)
        self.safety_timer = self.create_timer(0.1, self.check_safety)  # 安全检查定时器
        
        self.get_logger().info('舵轮机器人节点已启动')
        self.get_logger().info(f'初始位置: x={self.x}, y={self.y}, theta={self.theta}')
    
    def cmd_vel_callback(self, msg):
        """处理速度命令 - 简化的舵轮控制逻辑
        
        控制方案：
        - W键 (vx > 0): 主动轮正转（前进）
        - S键 (vx < 0): 主动轮反转（后退）
        - A键 (wz > 0): 前主动轮向左转45°，后主动轮向右转45°（左转弯）
        - D键 (wz < 0): 前主动轮向右转45°，后主动轮向左转45°（右转弯）
        """
        # 更新命令时间
        self.last_command_time = self.get_clock().now()
        
        # 1. 速度限制和安全检查
        vx = max(-self.max_linear_speed, min(self.max_linear_speed, msg.linear.x))
        vy = max(-self.max_linear_speed, min(self.max_linear_speed, msg.linear.y))
        wz = max(-self.max_angular_speed, min(self.max_angular_speed, msg.angular.z))
        
        R = self.wheel_radius
        
        # 2. 简化的舵轮控制逻辑
        # 转向角固定为45度
        STEERING_ANGLE = math.pi / 4.0  # 45度
        
        # 计算转向角
        # wz > 0: 左转 -> 前主动轮左转45°，后主动轮右转45°
        # wz < 0: 右转 -> 前主动轮右转45°，后主动轮左转45°
        # wz = 0: 直行 -> 两主动轮转角为0
        
        if abs(wz) > 0.05:  # 有转向命令
            # 根据wz的大小比例调整转向角（最大45度）
            steering_ratio = min(1.0, abs(wz) / self.max_angular_speed)
            actual_steering = STEERING_ANGLE * steering_ratio
            
            if wz > 0:  # 左转
                W1 = actual_steering   # 前主动轮向左转
                W2 = -actual_steering  # 后主动轮向右转
            else:  # 右转
                W1 = -actual_steering  # 前主动轮向右转
                W2 = actual_steering   # 后主动轮向左转
        else:  # 直行
            W1 = 0.0
            W2 = 0.0
        
        # 计算轮子转速
        # vx > 0: 前进，轮子正转
        # vx < 0: 后退，轮子反转
        V1 = vx / R
        V2 = vx / R
        
        # 被动轮跟随主动轮
        # 被动轮转向角与对角主动轮相同
        W3 = W2  # 右前轮跟随右后轮（对角）
        W4 = W1  # 左后轮跟随左前轮（对角）
        V3 = vx / R
        V4 = vx / R
        
        # 3. 平滑更新状态
        # 改进转向角平滑：当有转向时，使用更大的平滑系数使响应更快
        # 检测是否有转向意图
        has_turning = abs(wz) > 0.05 or abs(vy) > 0.05
        if has_turning:
            # 转向时使用更大的平滑系数，使角度变化更明显
            steering_factor = min(0.7, self.steering_smooth_factor * 1.4)
        else:
            steering_factor = self.steering_smooth_factor
        
        # 转向角平滑
        old_W1 = self.steering_angles[0]
        old_W2 = self.steering_angles[1]
        self.steering_angles[0] = self.steering_angles[0] + (W1 - self.steering_angles[0]) * steering_factor
        self.steering_angles[1] = self.steering_angles[1] + (W2 - self.steering_angles[1]) * steering_factor
        
        # 速度平滑
        self.wheel_velocities[0] = self.wheel_velocities[0] + (V1 - self.wheel_velocities[0]) * self.velocity_smooth_factor
        self.wheel_velocities[1] = self.wheel_velocities[1] + (V2 - self.wheel_velocities[1]) * self.velocity_smooth_factor
        
        # 被动轮平滑
        self.passive_steering_angles[0] = self.passive_steering_angles[0] + (W3 - self.passive_steering_angles[0]) * steering_factor
        self.passive_steering_angles[1] = self.passive_steering_angles[1] + (W4 - self.passive_steering_angles[1]) * steering_factor
        self.passive_wheel_velocities[0] = self.passive_wheel_velocities[0] + (V3 - self.passive_wheel_velocities[0]) * self.velocity_smooth_factor
        self.passive_wheel_velocities[1] = self.passive_wheel_velocities[1] + (V4 - self.passive_wheel_velocities[1]) * self.velocity_smooth_factor
        
        # 4. 更新位置（积分）
        dt = 0.1  # 与定时器频率一致
        self.steering_positions[0] = self.steering_angles[0]
        self.steering_positions[1] = self.steering_angles[1]
        self.wheel_positions[0] += self.wheel_velocities[0] * dt
        self.wheel_positions[1] += self.wheel_velocities[1] * dt
        
        self.passive_steering_positions[0] = self.passive_steering_angles[0]
        self.passive_steering_positions[1] = self.passive_steering_angles[1]
        self.passive_wheel_positions[0] += self.passive_wheel_velocities[0] * dt
        self.passive_wheel_positions[1] += self.passive_wheel_velocities[1] * dt
        
        # 5. 调试日志 - 增强转向角变化检测
        # 检测转向角是否发生变化
        steering_changed = abs(old_W1 - self.steering_angles[0]) > 0.01 or abs(old_W2 - self.steering_angles[1]) > 0.01
        
        if has_turning or steering_changed:
            self.get_logger().info(
                f'转向控制: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f} | '
                f'目标角度: W1={math.degrees(W1):.1f}°, W2={math.degrees(W2):.1f}° | '
                f'当前角度: W1={math.degrees(self.steering_angles[0]):.1f}°, W2={math.degrees(self.steering_angles[1]):.1f}° | '
                f'转速: V1={self.wheel_velocities[0]:.2f}, V2={self.wheel_velocities[1]:.2f}'
            )
        else:
            self.get_logger().debug(f'速度命令: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}')
    
    def publish_joint_states(self):
        """发布关节状态 - 8个关节：4个转向关节 + 4个驱动关节"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        # 关节名称：转向关节 + 驱动关节
        joint_state.name = [
            # 转向关节
            'front_left_steering_joint',   # 主动轮1
            'front_right_steering_joint',  # 被动轮1
            'rear_left_steering_joint',     # 被动轮2
            'rear_right_steering_joint',   # 主动轮2
            # 驱动关节
            'front_left_wheel_joint',      # 主动轮1
            'front_right_wheel_joint',      # 被动轮1
            'rear_left_wheel_joint',       # 被动轮2
            'rear_right_wheel_joint'       # 主动轮2
        ]
        
        # 关节位置：转向角 + 轮子位置
        joint_state.position = [
            self.steering_positions[0],      # front_left_steering (W1)
            self.passive_steering_positions[0],  # front_right_steering
            self.passive_steering_positions[1],  # rear_left_steering
            self.steering_positions[1],      # rear_right_steering (W2)
            self.wheel_positions[0],         # front_left_wheel (V1)
            self.passive_wheel_positions[0],    # front_right_wheel
            self.passive_wheel_positions[1],    # rear_left_wheel
            self.wheel_positions[1]          # rear_right_wheel (V2)
        ]
        
        # 关节速度：转向角速度 + 轮子角速度
        joint_state.velocity = [
            0.0,  # front_left_steering (转向角速度，通常为0或很小)
            0.0,  # front_right_steering
            0.0,  # rear_left_steering
            0.0,  # rear_right_steering
            self.wheel_velocities[0],        # front_left_wheel (V1)
            self.passive_wheel_velocities[0], # front_right_wheel
            self.passive_wheel_velocities[1], # rear_left_wheel
            self.wheel_velocities[1]         # rear_right_wheel (V2)
        ]
        
        self.joint_state_pub.publish(joint_state)
    
    def publish_odometry(self):
        """发布里程计信息 - 改进的舵轮运动学正解"""
        L = self.wheel_base / 2.0  # 前后轮距离的一半
        W = self.wheel_track / 2.0  # 左右轮距离的一半
        R = self.wheel_radius
        
        # 从主动轮状态计算机器人速度
        # 主动轮1（左前轮）：W1, V1
        # 主动轮2（右后轮）：W2, V2
        W1 = self.steering_angles[0]  # 使用平滑后的转向角
        W2 = self.steering_angles[1]
        V1 = self.wheel_velocities[0]  # 使用平滑后的速度
        V2 = self.wheel_velocities[1]
        
        # 计算每个主动轮在机器人坐标系中的速度分量
        # 左前轮位置: (L, W)
        vx1 = R * V1 * math.cos(W1)  # 左前轮在x方向的速度
        vy1 = R * V1 * math.sin(W1)  # 左前轮在y方向的速度
        
        # 右后轮位置: (-L, -W)
        vx2 = R * V2 * math.cos(W2)  # 右后轮在x方向的速度
        vy2 = R * V2 * math.sin(W2)  # 右后轮在y方向的速度
        
        # 改进的机器人速度计算
        # 方法：从两个主动轮的速度和位置关系计算机器人整体速度
        # 对于两主动轮结构，使用加权平均更准确
        vx = (vx1 + vx2) / 2.0
        vy = (vy1 + vy2) / 2.0
        
        # 改进的角速度计算
        # 使用两个主动轮的速度差和位置关系
        if abs(L) > 1e-6 and abs(W) > 1e-6:
            # 计算两个主动轮速度在机器人坐标系中的差异
            # 左前轮对旋转的贡献
            omega1 = (vx1 * math.sin(W1) - vy1 * math.cos(W1)) / (L + W)
            # 右后轮对旋转的贡献
            omega2 = (vx2 * math.sin(W2) - vy2 * math.cos(W2)) / (L + W)
            # 平均角速度
            wz = (omega1 + omega2) / 2.0
        else:
            wz = 0.0
        
        # 更新位置（积分）
        dt = 0.2  # 与定时器频率一致
        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.theta += wz * dt
        
        # 归一化角度到[-pi, pi]
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # 创建里程计消息
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # 位置
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # 姿态（四元数）
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # 速度
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        
        self.odom_pub.publish(odom)
    
    def check_safety(self):
        """安全检查：命令超时保护"""
        current_time = self.get_clock().now()
        time_since_last_command = (current_time - self.last_command_time).nanoseconds / 1e9
        
        # 如果命令超时，平滑停止
        if time_since_last_command > self.command_timeout:
            # 平滑减速到0
            self.steering_angles[0] = self.steering_angles[0] * 0.9
            self.steering_angles[1] = self.steering_angles[1] * 0.9
            self.wheel_velocities[0] = self.wheel_velocities[0] * 0.9
            self.wheel_velocities[1] = self.wheel_velocities[1] * 0.9
            
            # 如果速度很小，直接设为0
            if abs(self.wheel_velocities[0]) < 0.01:
                self.wheel_velocities[0] = 0.0
            if abs(self.wheel_velocities[1]) < 0.01:
                self.wheel_velocities[1] = 0.0
            if abs(self.steering_angles[0]) < 0.01:
                self.steering_angles[0] = 0.0
            if abs(self.steering_angles[1]) < 0.01:
                self.steering_angles[1] = 0.0
    
    def publish_tf(self):
        """发布TF变换"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # 位置
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # 姿态
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MecanumRobotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
