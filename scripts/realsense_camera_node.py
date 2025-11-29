#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs

class RealSenseCameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # 获取ROS参数
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('camera_serial', '')
        self.declare_parameter('width', 640)  # 使用标准分辨率（RealSense D435支持）
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)  # 标准帧率
        
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.camera_serial = self.get_parameter('camera_serial').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        
        # 使用camera_name创建话题名称
        topic_prefix = f'/camera_{self.camera_name}'
        
        # 创建发布者
        self.image_pub = self.create_publisher(Image, f'{topic_prefix}/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, f'{topic_prefix}/camera_info', 10)
        self.depth_pub = self.create_publisher(Image, f'{topic_prefix}/depth/image_raw', 10)
        
        # OpenCV桥接
        self.bridge = CvBridge()
        
        # RealSense相关
        self.pipeline = None
        self.config = None
        self.align = None
        self.camera_info_received = False
        self.color_intrinsics = None
        self.last_init_attempt = 0
        self.init_retry_interval = 5.0  # 重试间隔（秒）
        self.ctx = None  # Shared context for device management (multicam pattern)
        
        # 摄像头内参（将从RealSense获取）
        self.camera_matrix = None
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        
        # 创建定时器（根据配置的FPS调整）
        timer_period = 1.0 / self.fps if self.fps > 0 else 0.033
        self.timer = self.create_timer(timer_period, self.publish_image)
        
        # 初始化RealSense摄像头
        self.init_camera()
        
        self.get_logger().info(f'RealSense摄像头节点已启动 (名称: {self.camera_name}, 序列号: {self.camera_serial if self.camera_serial else "自动"})')
    
    def init_camera(self):
        """初始化RealSense摄像头 - 基于multicam示例的模式"""
        self.get_logger().info(f'正在初始化RealSense摄像头: {self.camera_name}...')
        
        try:
            # 创建共享的context（按照multicam示例的模式）
            if self.ctx is None:
                self.ctx = rs.context()
            
            # 在打开任何流之前捕获序列号（关键步骤，避免设备忙错误）
            device_list = self.ctx.query_devices()
            device_serials = []
            device_names = []
            
            # 安全地枚举设备并收集序列号
            # 使用迭代器方式访问设备，更健壮
            try:
                device_count = device_list.size()
                self.get_logger().info(f'检测到 {device_count} 个RealSense设备')
                
                # 尝试通过迭代器访问所有设备
                accessed_count = 0
                for i in range(device_count):
                    try:
                        dev = device_list[i]
                        # 尝试获取设备信息
                        try:
                            serial = dev.get_info(rs.camera_info.serial_number)
                            name = dev.get_info(rs.camera_info.name)
                            device_serials.append(serial)
                            device_names.append(name)
                            accessed_count += 1
                            self.get_logger().info(f'  ✓ 设备 {i}: 序列号={serial}, 名称={name}')
                        except RuntimeError as e:
                            error_msg = str(e).lower()
                            if 'bad optional access' in error_msg:
                                # 设备可能被占用，尝试通过临时pipeline访问
                                self.get_logger().debug(f'设备 {i} 直接访问失败，尝试其他方法...')
                                try:
                                    # 尝试创建临时pipeline来验证设备是否可用
                                    temp_pipeline = rs.pipeline(self.ctx)
                                    temp_config = rs.config()
                                    # 尝试通过设备对象本身获取序列号（如果可能）
                                    # 注意：如果设备被占用，这也会失败
                                    temp_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                                    # 不指定设备，让系统自动选择
                                    # 实际上，如果设备被占用，我们无法访问它
                                    # 所以这里我们只能记录警告
                                    self.get_logger().warn(f'设备 {i} 无法访问 (可能被其他进程占用): {e}')
                                except:
                                    self.get_logger().warn(f'设备 {i} 无法访问: {e}')
                            else:
                                self.get_logger().warn(f'设备 {i} 访问失败: {e}')
                            continue
                    except Exception as e:
                        self.get_logger().warn(f'枚举设备 {i} 时出错: {e} (跳过)')
                        continue
                
                if accessed_count < device_count:
                    self.get_logger().warn(f'只能访问 {accessed_count}/{device_count} 个设备')
                    self.get_logger().warn('提示: 如果设备被占用，请关闭其他使用RealSense的程序')
            except Exception as e:
                self.get_logger().error(f'枚举设备时出错: {e}')
                import traceback
                self.get_logger().error(f'堆栈跟踪: {traceback.format_exc()}')
                self.pipeline = None
                return
            
            if len(device_serials) == 0:
                self.get_logger().error('未找到可用的RealSense设备')
                self.pipeline = None
                return
            
            # 选择设备序列号
            selected_serial = None
            if self.camera_serial:
                # 如果指定了序列号，查找匹配的设备
                self.get_logger().info(f'查找序列号为 {self.camera_serial} 的摄像头...')
                if self.camera_serial in device_serials:
                    selected_serial = self.camera_serial
                    self.get_logger().info(f'找到匹配的设备: {self.camera_serial}')
                else:
                    self.get_logger().error(f'未找到序列号为 {self.camera_serial} 的设备')
                    self.get_logger().error(f'可用设备序列号: {device_serials}')
                    self.pipeline = None
                    return
            else:
                # 如果没有指定序列号，根据camera_name选择设备
                # front选择第一个，back选择第二个
                # 如果只有一个设备，允许back也使用它（单相机模式）
                if self.camera_name == 'front' and len(device_serials) > 0:
                    selected_serial = device_serials[0]
                    self.get_logger().info(f'自动选择第一个设备 (front): {selected_serial} ({device_names[0]})')
                elif self.camera_name == 'back':
                    if len(device_serials) > 1:
                        selected_serial = device_serials[1]
                        self.get_logger().info(f'自动选择第二个设备 (back): {selected_serial} ({device_names[1]})')
                    elif len(device_serials) == 1:
                        # 单相机模式：back也使用唯一的设备（但会与front冲突，所以禁用back）
                        self.get_logger().warn(f'只有一个设备可用，back摄像头将禁用以避免冲突')
                        self.get_logger().warn(f'如果只需要一个摄像头，请设置use_camera:=true并禁用back_camera_node')
                        self.pipeline = None
                        return
                    else:
                        self.get_logger().error(f'没有可用设备')
                        self.pipeline = None
                        return
                else:
                    self.get_logger().error(f'无法自动选择设备: camera_name={self.camera_name}, 可用设备数={len(device_serials)}')
                    self.get_logger().error(f'可用设备序列号: {device_serials}')
                    self.pipeline = None
                    return
            
            # 创建RealSense pipeline（使用共享context，按照multicam示例）
            self.pipeline = rs.pipeline(self.ctx)
            self.config = rs.config()
            
            # 配置特定设备（在打开流之前）
            self.config.enable_device(selected_serial)
            
            # 配置RGB和深度流（尝试多种分辨率组合以确保兼容性）
            # RealSense D435支持的标准分辨率: 640x480, 848x480, 1280x720等
            resolutions_to_try = [
                (self.width, self.height, self.fps),  # 首先尝试用户配置
                (640, 480, 30),  # 标准配置
                (640, 480, 15),  # 降低帧率
                (640, 360, 30),  # 更小分辨率
                (424, 240, 30),  # 最小分辨率
            ]
            
            pipeline_started = False
            pipeline_profile = None
            for width, height, fps in resolutions_to_try:
                try:
                    self.get_logger().info(f'尝试配置流: RGB={width}x{height}@{fps}fps, Depth={width}x{height}@{fps}fps')
                    # 确保pipeline已停止（如果之前尝试失败）
                    if pipeline_profile is not None:
                        try:
                            self.pipeline.stop()
                        except:
                            pass
                        pipeline_profile = None
                    
                    # 创建新config以避免冲突
                    test_config = rs.config()
                    test_config.enable_device(selected_serial)
                    test_config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
                    test_config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
                    
                    # 验证配置是否有效
                    pipeline_profile = self.pipeline.start(test_config)
                    self.config = test_config
                    self.width = width
                    self.height = height
                    self.fps = fps
                    pipeline_started = True
                    self.get_logger().info(f'✅ 成功配置: {width}x{height}@{fps}fps')
                    break
                except RuntimeError as e:
                    error_msg = str(e)
                    if 'Couldn\'t resolve requests' in error_msg or 'Requested stream format is not natively supported' in error_msg:
                        self.get_logger().debug(f'配置 {width}x{height}@{fps}fps 不支持，尝试下一个...')
                        continue
                    elif 'NvMapMemAllocInternalTagged' in error_msg or 'error 12' in error_msg:
                        self.get_logger().warn(f'GPU内存分配失败，尝试更低分辨率: {width}x{height}@{fps}fps')
                        continue
                    else:
                        self.get_logger().warn(f'配置失败: {error_msg}，尝试下一个...')
                        continue
                except Exception as e:
                    self.get_logger().warn(f'配置时发生错误: {e}，尝试下一个...')
                    continue
            
            if not pipeline_started or pipeline_profile is None:
                self.get_logger().error('所有分辨率配置都失败，无法启动pipeline')
                self.get_logger().error('请检查RealSense相机连接和驱动')
                if self.pipeline:
                    try:
                        self.pipeline.stop()
                    except:
                        pass
                self.pipeline = None
                return
            
            # Pipeline已在上面成功启动，pipeline_profile已设置
            
            # 验证设备序列号
            device = pipeline_profile.get_device()
            device_serial = device.get_info(rs.camera_info.serial_number)
            device_name = device.get_info(rs.camera_info.name)
            self.get_logger().info(f'已连接到摄像头: {device_serial} ({device_name}) - 名称: {self.camera_name}')
            
            # 获取颜色流的内参
            color_profile = pipeline_profile.get_stream(rs.stream.color)
            self.color_intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
            
            # 创建对齐对象（将深度对齐到RGB）
            self.align = rs.align(rs.stream.color)
            
            # 从RealSense内参构建相机矩阵
            self.camera_matrix = np.array([
                [self.color_intrinsics.fx, 0, self.color_intrinsics.ppx],
                [0, self.color_intrinsics.fy, self.color_intrinsics.ppy],
                [0, 0, 1]
            ], dtype=np.float32)
            
            # 获取畸变系数
            if hasattr(self.color_intrinsics, 'coeffs') and len(self.color_intrinsics.coeffs) >= 5:
                self.dist_coeffs = np.array(self.color_intrinsics.coeffs[:5], dtype=np.float32).reshape(5, 1)
            else:
                self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
            
            self.get_logger().info(f'RealSense摄像头初始化成功')
            self.get_logger().info(f'分辨率: {self.color_intrinsics.width}x{self.color_intrinsics.height}')
            self.get_logger().info(f'内参矩阵: fx={self.color_intrinsics.fx}, fy={self.color_intrinsics.fy}')
            self.get_logger().info(f'主点: cx={self.color_intrinsics.ppx}, cy={self.color_intrinsics.ppy}')
            
        except Exception as e:
            self.get_logger().error(f'RealSense摄像头初始化失败: {e}')
            import traceback
            self.get_logger().error(f'堆栈跟踪: {traceback.format_exc()}')
            self.pipeline = None
    
    def publish_image(self):
        """发布摄像头图像"""
        if self.pipeline is None:
            import time
            current_time = time.time()
            # 避免频繁重试
            if current_time - self.last_init_attempt > self.init_retry_interval:
                self.get_logger().warn('RealSense摄像头未初始化，尝试重新初始化...')
                self.last_init_attempt = current_time
                self.init_camera()
            return
        
        try:
            # 等待帧
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            
            # 对齐深度帧到颜色帧
            aligned_frames = self.align.process(frames)
            
            # 获取对齐后的颜色帧和深度帧
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame:
                self.get_logger().warn('无法获取颜色帧')
                return
            
            # 转换为numpy数组
            color_image = np.asanyarray(color_frame.get_data())
            
            # 检查图像尺寸
            if len(color_image.shape) != 3 or color_image.shape[2] != 3:
                self.get_logger().warn(f'图像格式不正确: {color_image.shape}')
                return
            
            # 转换为ROS图像消息
            ros_image = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = f'camera_{self.camera_name}_link'
            
            # 发布RGB图像
            self.image_pub.publish(ros_image)
            
            # 发布深度图像（如果可用）- 自定义距离着色: >1m蓝色, 0.3-1m黄色, <0.3m红色
            if depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # RealSense深度值以毫米为单位
                # 创建彩色图像
                depth_colormap = np.zeros((depth_image.shape[0], depth_image.shape[1], 3), dtype=np.uint8)
                
                # 定义距离阈值（毫米）
                NEAR_THRESHOLD = 300  # 0.3m
                FAR_THRESHOLD = 1000  # 1m
                
                # 创建掩码
                near_mask = depth_image < NEAR_THRESHOLD  # < 0.3m: 红色
                mid_mask = (depth_image >= NEAR_THRESHOLD) & (depth_image < FAR_THRESHOLD)  # 0.3-1m: 黄色
                far_mask = depth_image >= FAR_THRESHOLD  # > 1m: 蓝色
                zero_mask = depth_image == 0  # 无效深度: 黑色
                
                # 应用颜色
                # 红色区域 (< 0.3m): 根据距离调整红色强度
                if np.any(near_mask):
                    near_depth = depth_image[near_mask]
                    # 归一化到0-255，距离越近红色越强
                    near_intensity = ((NEAR_THRESHOLD - near_depth) / NEAR_THRESHOLD * 255).astype(np.uint8)
                    near_intensity = np.clip(near_intensity, 50, 255)  # 最小50确保可见
                    # BGR格式: [B, G, R]
                    depth_colormap[near_mask, 0] = 0  # B
                    depth_colormap[near_mask, 1] = 0  # G
                    depth_colormap[near_mask, 2] = near_intensity  # R (红色)
                
                # 黄色区域 (0.3-1m): 根据距离调整黄色强度
                if np.any(mid_mask):
                    mid_depth = depth_image[mid_mask]
                    # 归一化到0-255，中间距离为黄色
                    mid_normalized = ((mid_depth - NEAR_THRESHOLD) / (FAR_THRESHOLD - NEAR_THRESHOLD) * 255).astype(np.uint8)
                    mid_normalized = np.clip(mid_normalized, 0, 255)
                    # BGR格式: [B, G, R] - 黄色 = 红+绿
                    depth_colormap[mid_mask, 0] = 0  # B
                    depth_colormap[mid_mask, 1] = mid_normalized  # G
                    depth_colormap[mid_mask, 2] = mid_normalized  # R
                
                # 蓝色区域 (> 1m): 根据距离调整蓝色强度
                if np.any(far_mask):
                    far_depth = depth_image[far_mask]
                    # 归一化，距离越远蓝色越深
                    # 限制最大距离到5m以便更好的可视化
                    max_depth = 5000  # 5m
                    far_normalized = np.clip((far_depth - FAR_THRESHOLD) / (max_depth - FAR_THRESHOLD) * 255, 0, 255).astype(np.uint8)
                    # BGR格式: [B, G, R]
                    depth_colormap[far_mask, 0] = far_normalized  # B (蓝色)
                    depth_colormap[far_mask, 1] = 0  # G
                    depth_colormap[far_mask, 2] = 0  # R
                
                # 无效深度区域保持黑色
                depth_colormap[zero_mask] = [0, 0, 0]
                
                # 发布colormapped深度图像
                ros_depth = self.bridge.cv2_to_imgmsg(depth_colormap, 'bgr8')
                ros_depth.header.stamp = ros_image.header.stamp
                ros_depth.header.frame_id = f'camera_{self.camera_name}_link'
                self.depth_pub.publish(ros_depth)
            
            # 发布摄像头信息（只在第一次发布）
            if not self.camera_info_received:
                self.publish_camera_info()
                self.camera_info_received = True
                self.get_logger().info('摄像头信息已发布')
                
        except RuntimeError as e:
            if "Frame didn't arrive" in str(e):
                self.get_logger().warn('等待帧超时，继续尝试...')
            else:
                self.get_logger().error(f'RealSense运行时错误: {e}')
                # 尝试重新初始化
                self.reinit_camera()
        except Exception as e:
            self.get_logger().error(f'图像发布错误: {e}')
            import traceback
            self.get_logger().error(f'堆栈跟踪: {traceback.format_exc()}')
    
    def reinit_camera(self):
        """重新初始化摄像头"""
        import time
        current_time = time.time()
        # 避免频繁重试
        if current_time - self.last_init_attempt < self.init_retry_interval:
            return
        
        self.get_logger().info('尝试重新初始化RealSense摄像头...')
        try:
            if self.pipeline:
                self.pipeline.stop()
        except:
            pass
        self.pipeline = None
        self.camera_info_received = False
        self.last_init_attempt = current_time
        self.init_camera()
    
    def publish_camera_info(self):
        """发布摄像头信息"""
        if self.color_intrinsics is None or self.camera_matrix is None:
            self.get_logger().error('摄像头内参未初始化，无法发布camera_info')
            return
        
        try:
            camera_info = CameraInfo()
            camera_info.header.stamp = self.get_clock().now().to_msg()
            camera_info.header.frame_id = f'camera_{self.camera_name}_link'
            
            # 设置摄像头参数
            camera_info.width = self.color_intrinsics.width
            camera_info.height = self.color_intrinsics.height
            
            self.get_logger().info(f'摄像头分辨率: {camera_info.width}x{camera_info.height}')
            
            # 设置内参矩阵 (3x3矩阵，展平为9个元素的列表)
            camera_info.k = [
                float(self.camera_matrix[0, 0]), float(self.camera_matrix[0, 1]), float(self.camera_matrix[0, 2]),
                float(self.camera_matrix[1, 0]), float(self.camera_matrix[1, 1]), float(self.camera_matrix[1, 2]),
                float(self.camera_matrix[2, 0]), float(self.camera_matrix[2, 1]), float(self.camera_matrix[2, 2])
            ]
            
            # 设置畸变系数 (5个元素)
            try:
                if self.dist_coeffs.shape == (5, 1):
                    camera_info.d = [
                        float(self.dist_coeffs[0, 0]), float(self.dist_coeffs[1, 0]), 
                        float(self.dist_coeffs[2, 0]), float(self.dist_coeffs[3, 0]), 
                        float(self.dist_coeffs[4, 0])
                    ]
                else:
                    # 如果形状不正确，使用零值
                    camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
                    self.get_logger().warn(f'畸变系数形状不正确: {self.dist_coeffs.shape}，使用零值')
            except Exception as e:
                self.get_logger().error(f'设置畸变系数时出错: {e}')
                camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            
            # 设置投影矩阵 (3x4矩阵，展平为12个元素的列表)
            fx = float(self.camera_matrix[0, 0])
            fy = float(self.camera_matrix[1, 1])
            cx = float(self.camera_matrix[0, 2])
            cy = float(self.camera_matrix[1, 2])
            
            camera_info.p = [
                fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            
            # 设置旋转矩阵（单位矩阵）- 使用float类型
            camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            
            self.camera_info_pub.publish(camera_info)
            self.get_logger().info('摄像头信息已发布')
            
        except Exception as e:
            self.get_logger().error(f'发布摄像头信息时出错: {e}')
            self.get_logger().error(f'错误详情: {str(e)}')
            import traceback
            self.get_logger().error(f'堆栈跟踪: {traceback.format_exc()}')
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info('正在释放RealSense摄像头资源...')
        try:
            if self.pipeline is not None:
                self.pipeline.stop()
                self.pipeline = None
                self.get_logger().info('RealSense摄像头已释放')
        except Exception as e:
            self.get_logger().error(f'释放摄像头资源时出错: {e}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RealSenseCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

