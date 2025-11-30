#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion
import time

class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector_node')
        
        # 创建发布者 - 为每个相机分别发布
        self.image_pub_front = self.create_publisher(Image, '/apriltag_detection_front', 10)
        self.image_pub_back = self.create_publisher(Image, '/apriltag_detection_back', 10)
        self.pose_pub_front = self.create_publisher(PoseStamped, '/apriltag_pose_front', 10)
        self.pose_pub_back = self.create_publisher(PoseStamped, '/apriltag_pose_back', 10)
        self.status_pub_front = self.create_publisher(String, '/apriltag_status_front', 10)
        self.status_pub_back = self.create_publisher(String, '/apriltag_status_back', 10)
        
        # 为了向后兼容，也保留通用话题
        self.image_pub = self.create_publisher(Image, '/apriltag_detection', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/apriltag_pose', 10)
        self.status_pub = self.create_publisher(String, '/apriltag_status', 10)
        
        # 订阅前置摄像头图像
        self.image_sub_front = self.create_subscription(
            Image,
            '/camera_front/image_raw',
            lambda msg: self.image_callback(msg, 'front'),
            10
        )
        
        # 订阅后置摄像头图像
        self.image_sub_back = self.create_subscription(
            Image,
            '/camera_back/image_raw',
            lambda msg: self.image_callback(msg, 'back'),
            10
        )
        
        # 订阅前置摄像头信息
        self.camera_info_sub_front = self.create_subscription(
            CameraInfo,
            '/camera_front/camera_info',
            lambda msg: self.camera_info_callback(msg, 'front'),
            10
        )
        
        # 订阅后置摄像头信息
        self.camera_info_sub_back = self.create_subscription(
            CameraInfo,
            '/camera_back/camera_info',
            lambda msg: self.camera_info_callback(msg, 'back'),
            10
        )
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # OpenCV桥接
        self.bridge = CvBridge()
        
        # 为每个相机分别存储摄像头参数
        self.camera_params = {
            'front': {
                'camera_matrix': None,
                'dist_coeffs': None,
                'info_received': False
            },
            'back': {
                'camera_matrix': None,
                'dist_coeffs': None,
                'info_received': False
            }
        }
        
        # 为每个相机分别存储检测结果
        self.detected_tags = {
            'front': {},
            'back': {}
        }
        
        # AprilTag检测器（参数优化）- 支持多种OpenCV版本
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        # Use DetectorParameters_create() for older OpenCV versions, or DetectorParameters() for newer ones
        try:
            detector_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            # Fallback for older OpenCV versions
            detector_params = cv2.aruco.DetectorParameters_create()
        # 角点细化提高定位稳定性
        if hasattr(detector_params, 'cornerRefinementMethod'):
            detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        if hasattr(detector_params, 'cornerRefinementWinSize'):
            detector_params.cornerRefinementWinSize = 5
        if hasattr(detector_params, 'cornerRefinementMaxIterations'):
            detector_params.cornerRefinementMaxIterations = 30
        if hasattr(detector_params, 'cornerRefinementMinAccuracy'):
            detector_params.cornerRefinementMinAccuracy = 0.01
        # 过滤极小/极大周长候选以降低误检
        if hasattr(detector_params, 'minMarkerPerimeterRate'):
            detector_params.minMarkerPerimeterRate = 0.02
        if hasattr(detector_params, 'maxMarkerPerimeterRate'):
            detector_params.maxMarkerPerimeterRate = 4.0
        
        # Try to use ArucoDetector (OpenCV 4.7+), fallback to detectMarkers (older versions)
        self.use_aruco_detector = False
        try:
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)
            self.use_aruco_detector = True
            self.get_logger().info("Using ArucoDetector API (OpenCV 4.7+)")
        except AttributeError:
            # Fallback to older API
            self.aruco_dict = aruco_dict
            self.detector_params = detector_params
            self.use_aruco_detector = False
            self.get_logger().info("Using detectMarkers API (OpenCV < 4.7)")
        
        # AprilTag物理尺寸（米）
        self.tag_size = 0.1  # 10cm x 10cm标签
        
        # 检测到的标签信息
        self.detected_tags = {}

        # 性能与日志节流
        self.image_scale = 0.75  # 0.5~1.0，越小越快
        self.last_log_time = 0.0
        self.log_interval_sec = 1.0
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info('AprilTag检测节点已启动 (OpenCV 4.10兼容版)')
        self.get_logger().info('等待摄像头图像...')
        self.get_logger().info('订阅前置相机: /camera_front/image_raw')
        self.get_logger().info('订阅后置相机: /camera_back/image_raw')
    
    def camera_info_callback(self, msg, camera_name):
        """处理摄像头信息"""
        # 验证相机参数是否有效
        if len(msg.k) < 9 or msg.k[0] <= 0 or msg.k[4] <= 0:
            self.get_logger().warn(f'⚠️ {camera_name}相机参数无效 (k矩阵未初始化)')
            return
        
        # 检查是否有NaN值
        k_array = np.array(msg.k)
        if np.any(np.isnan(k_array)) or np.any(np.isinf(k_array)):
            self.get_logger().warn(f'⚠️ {camera_name}相机参数包含NaN或Inf值')
            return
        
        if not self.camera_params[camera_name]['info_received']:
            # 提取摄像头内参
            self.camera_params[camera_name]['camera_matrix'] = k_array.reshape(3, 3)
            
            # 处理畸变系数
            if len(msg.d) > 0:
                d_array = np.array(msg.d)
                # 检查是否有NaN值，如果有则设为0
                d_array = np.nan_to_num(d_array, nan=0.0, posinf=0.0, neginf=0.0)
                self.camera_params[camera_name]['dist_coeffs'] = d_array
            else:
                # 如果没有畸变系数，使用零数组
                self.camera_params[camera_name]['dist_coeffs'] = np.zeros(5)
            
            self.camera_params[camera_name]['info_received'] = True
            self.get_logger().info(f'✅ {camera_name}相机参数已接收: fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f}, '
                                  f'cx={msg.k[2]:.2f}, cy={msg.k[5]:.2f}')
    
    def image_callback(self, msg, camera_name):
        """处理摄像头图像"""
        try:
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 检测AprilTag
            self.detect_apriltags(cv_image, msg.header.stamp, camera_name)
            
            # 发布处理后的图像
            self.publish_processed_image(cv_image, camera_name)
            
        except Exception as e:
            self.get_logger().error(f'❌ {camera_name}相机图像处理错误: {e}')
    
    def detect_apriltags(self, image, timestamp, camera_name):
        """检测AprilTag"""
        if not self.camera_params[camera_name]['info_received']:
            return
        
        # 可选下采样以提升性能
        if self.image_scale < 1.0:
            small_image = cv2.resize(image, None, fx=self.image_scale, fy=self.image_scale, interpolation=cv2.INTER_AREA)
        else:
            small_image = image
        
        # 转换为灰度图像
        gray = cv2.cvtColor(small_image, cv2.COLOR_BGR2GRAY)
        
        # 检测AprilTag - 使用适当的API
        if self.use_aruco_detector:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            # 使用旧的API
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
        
        if ids is not None and len(ids) > 0:
            # 如果进行了下采样，需要把角点坐标放大回原始尺度
            if self.image_scale < 1.0:
                scaled_corners = []
                scale = 1.0 / self.image_scale
                for c in corners:
                    scaled_corners.append((c * scale).astype(np.float32))
                corners = scaled_corners
            # 使用solvePnP进行位姿估计
            self.estimate_pose_with_solvePnP(corners, ids, timestamp, image, camera_name)
        else:
            # 清除该相机的旧检测结果
            self.detected_tags[camera_name].clear()
    
    def estimate_pose_with_solvePnP(self, corners, ids, timestamp, image, camera_name):
        """使用solvePnP进行位姿估计"""
        # 定义AprilTag的3D点（以标签中心为原点）
        object_points = np.array([
            [-self.tag_size/2, -self.tag_size/2, 0],
            [self.tag_size/2, -self.tag_size/2, 0],
            [self.tag_size/2, self.tag_size/2, 0],
            [-self.tag_size/2, self.tag_size/2, 0]
        ], dtype=np.float32)
        
        # 获取该相机的参数
        camera_matrix = self.camera_params[camera_name]['camera_matrix']
        dist_coeffs = self.camera_params[camera_name]['dist_coeffs']
        
        # 处理每个检测到的标签
        for i, tag_id in enumerate(ids.flatten()):
            try:
                # 获取2D角点
                image_points = corners[i][0].astype(np.float32)
                
                # 使用solvePnP进行位姿估计
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    camera_matrix,
                    dist_coeffs
                )
                
                if success:
                    # 计算距离
                    distance = np.linalg.norm(tvec)
                    
                    # 计算角度
                    angle_x = math.atan2(tvec[0], tvec[2])
                    angle_y = math.atan2(tvec[1], tvec[2])
                    
                    # 存储标签信息（包含相机名称）
                    tag_key = f"{camera_name}_{tag_id}"
                    self.detected_tags[camera_name][tag_id] = {
                        'position': tvec.flatten(),
                        'rotation': rvec.flatten(),
                        'distance': distance,
                        'angle_x': angle_x,
                        'angle_y': angle_y,
                        'timestamp': timestamp,
                        'camera': camera_name
                    }
                    
                    # 发布标签位姿（区分相机）
                    self.publish_tag_pose(tag_id, tvec.flatten(), rvec.flatten(), timestamp, camera_name)
                    
                    # 发布TF变换（区分相机）
                    self.publish_tag_tf(tag_id, tvec.flatten(), rvec.flatten(), timestamp, camera_name)
                    
                    # 在图像上绘制检测结果
                    self.draw_detection(image, corners[i], tag_id, tvec.flatten(), distance, camera_name)
                    
                    # 节流日志输出，避免刷屏
                    now = time.time()
                    if now - self.last_log_time > self.log_interval_sec:
                        self.last_log_time = now
                        tvec_flat = tvec.flatten()
                        self.get_logger().info(
                            f'🎯 [{camera_name}] 检测到AprilTag ID: {int(tag_id)}, '
                            f'距离: {distance:.2f}m, '
                            f'位置: ({tvec_flat[0]:.2f}, {tvec_flat[1]:.2f}, {tvec_flat[2]:.2f})'
                        )
                    
            except Exception as e:
                self.get_logger().error(f'[{camera_name}] 位姿估计错误: {e}')
    
    def publish_tag_pose(self, tag_id, position, rotation, timestamp, camera_name):
        """发布标签位姿"""
        pose = PoseStamped()
        pose.header.stamp = timestamp
        # 在frame_id中携带标签ID和相机名称，便于前端解析显示
        pose.header.frame_id = f'apriltag_{camera_name}_{int(tag_id)}'
        
        # 位置
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        
        # 旋转（从旋转向量转换为四元数）
        rotation_matrix, _ = cv2.Rodrigues(rotation)
        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
        
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        
        # 根据相机名称发布到相应话题
        if camera_name == 'front':
            self.pose_pub_front.publish(pose)
        else:
            self.pose_pub_back.publish(pose)
        
        # 为了向后兼容，也发布到通用话题（使用前置相机的数据）
        if camera_name == 'front':
            self.pose_pub.publish(pose)
    
    def publish_tag_tf(self, tag_id, position, rotation, timestamp, camera_name):
        """发布标签TF变换"""
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = f'camera_{camera_name}_link'
        t.child_frame_id = f'apriltag_{camera_name}_{tag_id}'
        
        # 位置
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]
        
        # 旋转
        rotation_matrix, _ = cv2.Rodrigues(rotation)
        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
        
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def draw_detection(self, image, corners, tag_id, position, distance, camera_name):
        """在图像上绘制检测结果"""
        # 根据相机选择不同的颜色
        if camera_name == 'front':
            color = (0, 255, 0)  # 绿色表示前置相机
            text_color = (0, 255, 0)
        else:
            color = (255, 0, 0)  # 蓝色表示后置相机
            text_color = (255, 0, 0)
        
        # 绘制标签轮廓 - 修复OpenCV 4.10兼容性
        try:
            cv2.aruco.drawDetectedMarkers(image, corners, np.array([tag_id]))
        except Exception as e:
            # 如果绘制失败，至少绘制角点
            cv2.polylines(image, [corners.astype(np.int32)], True, color, 2)
        
        # 绘制坐标轴 - 简化版本避免复杂投影
        try:
            # 获取角点中心作为原点
            center = np.mean(corners, axis=1).astype(np.int32)[0]
            
            # 绘制简单的坐标轴
            axis_length = 30  # 像素长度
            cv2.line(image, tuple(center), tuple(center + [axis_length, 0]), (0, 0, 255), 3)  # X轴 - 红色
            cv2.line(image, tuple(center), tuple(center + [0, axis_length]), (0, 255, 0), 3)  # Y轴 - 绿色
            
        except Exception as e:
            self.get_logger().warn(f'绘制坐标轴失败: {e}')
        
        # 在标签附近添加文本信息（ID、距离和相机名称）
        center_pt = np.mean(corners, axis=1).astype(np.int32)[0]
        text = f"[{camera_name.upper()}] ID:{int(tag_id)} {distance:.2f}m"
        text_origin = (int(center_pt[0]) + 5, int(center_pt[1]) - 5)
        cv2.putText(image, text, text_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
    
    def publish_processed_image(self, image, camera_name):
        """发布处理后的图像"""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            
            # 根据相机名称发布到相应话题
            if camera_name == 'front':
                self.image_pub_front.publish(msg)
            else:
                self.image_pub_back.publish(msg)
            
            # 为了向后兼容，也发布到通用话题（使用前置相机的数据）
            if camera_name == 'front':
                self.image_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'发布图像时出错: {e}')
    
    def publish_status(self):
        """发布检测状态"""
        # 为每个相机分别发布状态
        for camera_name in ['front', 'back']:
            status_msg = String()
            tags = self.detected_tags[camera_name]
            if tags:
                ids = [int(i) for i in tags.keys()]
                ids.sort()
                status_msg.data = f"[{camera_name.upper()}] 检测到 {len(ids)} 个标签: {ids}"
            else:
                status_msg.data = f"[{camera_name.upper()}] 未检测到标签"
            
            if camera_name == 'front':
                self.status_pub_front.publish(status_msg)
            else:
                self.status_pub_back.publish(status_msg)
        
        # 为了向后兼容，也发布合并状态到通用话题
        all_tags = {}
        for camera_name in ['front', 'back']:
            for tag_id, tag_info in self.detected_tags[camera_name].items():
                all_tags[f"{camera_name}_{tag_id}"] = tag_info
        
        status_msg = String()
        if all_tags:
            tag_list = [f"{info['camera']}_{tag_id}" for tag_id, info in all_tags.items()]
            status_msg.data = f"检测到 {len(tag_list)} 个标签: {tag_list}"
        else:
            status_msg.data = "未检测到标签"
        
        self.status_pub.publish(status_msg)
    
    def rotation_matrix_to_quaternion(self, R):
        """将旋转矩阵转换为四元数"""
        trace = np.trace(R)
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return [x, y, z, w]

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
