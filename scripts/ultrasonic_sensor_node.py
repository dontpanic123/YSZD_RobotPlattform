#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header
import serial
import serial.tools.list_ports
import re
import threading
import time

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        
        # ROS参数
        self.declare_parameter('serial_port', '/dev/ttyTHS1')  # Jetson Orin Nano UART1
        self.declare_parameter('baudrate', 115200)  # Jetson UART1 baudrate
        self.declare_parameter('query_frequency', 5)  # 查询频率 0-5
        self.declare_parameter('auto_feedback', True)  # 是否自动反馈
        
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.query_frequency = self.get_parameter('query_frequency').get_parameter_value().integer_value
        self.auto_feedback = self.get_parameter('auto_feedback').get_parameter_value().bool_value
        
        # 串口对象
        self.serial_conn = None
        
        # 传感器数据存储 (8个传感器，索引0-7)
        self.sensor_readings = [None] * 8  # 存储距离值（毫米）
        self.last_update_time = [0.0] * 8
        
        # 创建发布者 - 为每个传感器创建一个Range消息发布者
        # 传感器编号从1开始显示（内部存储仍为0-7以匹配协议）
        self.sensor_pubs = []
        for i in range(8):
            pub = self.create_publisher(
                Range, 
                f'/ultrasonic/sensor_{i+1}',  # 显示编号从1开始
                10
            )
            self.sensor_pubs.append(pub)
        
        # 创建综合数据发布者（发布所有传感器数据）
        from std_msgs.msg import Float32MultiArray, MultiArrayDimension
        self.all_sensors_pub = self.create_publisher(
            Float32MultiArray,
            '/ultrasonic/all_sensors',
            10
        )
        
        # 初始化串口
        self.init_serial()
        
        # 配置传感器
        if self.auto_feedback:
            self.configure_sensor_frequency(self.query_frequency)
        else:
            self.configure_sensor_frequency(0)  # 查询反馈模式
        
        # 启动读取线程
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()
        
        # 如果使用查询模式，创建定时查询
        if not self.auto_feedback:
            self.create_timer(1.0, self.query_sensor_data)  # 每秒查询一次
        
        self.get_logger().info('超声波传感器节点已启动')
        self.get_logger().info(f'串口: {self.serial_port}, 波特率: {self.baudrate}')
        self.get_logger().info(f'查询频率: {self.query_frequency}, 自动反馈: {self.auto_feedback}')
        
        # Jetson UART提示
        if 'ttyTHS' in self.serial_port:
            self.get_logger().info('检测到Jetson UART设备')
            self.get_logger().info('GPIO映射: GPIO 6=GND, GPIO 8=UART1_TXD, GPIO 10=UART1_RXD')
            self.get_logger().info('如果遇到权限问题，请运行: sudo usermod -a -G dialout $USER')
    
    def init_serial(self):
        """初始化串口连接"""
        try:
            # 如果端口是自动检测，尝试查找（优先Jetson UART设备）
            if self.serial_port == 'auto':
                # 首先尝试Jetson UART设备
                jetson_uart_devices = ['/dev/ttyTHS1', '/dev/ttyTHS0', '/dev/ttyTHS2']
                for device in jetson_uart_devices:
                    try:
                        import os
                        if os.path.exists(device):
                            self.serial_port = device
                            self.get_logger().info(f'自动检测到Jetson UART设备: {self.serial_port}')
                            break
                    except:
                        continue
                else:
                    # 如果没有找到Jetson UART，尝试USB串口
                    ports = serial.tools.list_ports.comports()
                    for port in ports:
                        if 'USB' in port.device or 'ttyUSB' in port.device or 'ttyACM' in port.device:
                            self.serial_port = port.device
                            self.get_logger().info(f'自动检测到USB串口: {self.serial_port}')
                            break
                    else:
                        self.get_logger().error('未找到可用串口')
                        return
            
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            if self.serial_conn.is_open:
                self.get_logger().info(f'串口已打开: {self.serial_port}')
                time.sleep(0.5)  # 等待串口稳定
            else:
                self.get_logger().error('串口打开失败')
                
        except serial.SerialException as e:
            self.get_logger().error(f'串口初始化失败: {e}')
            if 'ttyTHS' in self.serial_port:
                self.get_logger().error('Jetson UART权限提示: 请确保用户有权限访问UART设备')
                self.get_logger().error('运行: sudo usermod -a -G dialout $USER 然后重新登录')
            self.serial_conn = None
        except PermissionError as e:
            self.get_logger().error(f'串口权限错误: {e}')
            if 'ttyTHS' in self.serial_port:
                self.get_logger().error('Jetson UART权限提示: 请运行: sudo usermod -a -G dialout $USER')
            self.serial_conn = None
        except Exception as e:
            self.get_logger().error(f'初始化串口时出错: {e}')
            self.serial_conn = None
    
    def calculate_checksum(self, message):
        """计算异或校验和（当前固定返回XX用于调试）"""
        # 实际应该计算异或和，但协议说明调试时固定为*XX
        return 'XX'
    
    def send_command(self, command):
        """发送命令到传感器模块"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().warn('串口未打开，无法发送命令')
            return False
        
        try:
            # 构建完整命令：$命令*XX\r\n
            checksum = self.calculate_checksum(command)
            full_command = f'${command}*{checksum}\r\n'
            
            self.serial_conn.write(full_command.encode('ascii'))
            self.get_logger().debug(f'发送命令: {full_command.strip()}')
            return True
        except Exception as e:
            self.get_logger().error(f'发送命令失败: {e}')
            return False
    
    def configure_sensor_frequency(self, frequency):
        """配置传感器查询频率"""
        # $UDM1;[频率]*XX
        command = f'UDM1;{frequency}'
        return self.send_command(command)
    
    def query_sensor_data(self):
        """查询传感器数据（用于查询反馈模式）"""
        # $UDM2;CHK*XX
        command = 'UDM2;CHK'
        self.send_command(command)
    
    def parse_sensor_message(self, message):
        """解析传感器反馈消息
        
        格式: $udm1;[n];[传感器信息1];[传感器信息2];[传感器信息n]*XX
        传感器信息: 5个字符，第1位是索引(0-7)，后4位是距离(毫米)
        """
        try:
            # 移除$和*XX\r\n
            if not message.startswith('$udm1;'):
                return False
            
            # 提取消息体（去掉$和*XX\r\n）
            if '*XX' in message:
                msg_body = message.split('*XX')[0]
            else:
                msg_body = message.rstrip('\r\n')
            
            # 移除$udm1;前缀
            if msg_body.startswith('$udm1;'):
                msg_body = msg_body[6:]
            
            # 按分号分割
            parts = msg_body.split(';')
            if len(parts) < 2:
                self.get_logger().warn(f'消息格式错误: {message}')
                return False
            
            # 第一个部分是传感器数量
            try:
                sensor_count = int(parts[0])
            except ValueError:
                self.get_logger().warn(f'无法解析传感器数量: {parts[0]}')
                return False
            
            # 解析每个传感器的数据
            current_time = time.time()
            updated_sensors = []
            
            for i in range(1, min(len(parts), sensor_count + 1)):
                sensor_info = parts[i].strip()
                if len(sensor_info) != 5:
                    self.get_logger().warn(f'传感器信息格式错误: {sensor_info}')
                    continue
                
                try:
                    # 第1位是传感器索引
                    sensor_index = int(sensor_info[0])
                    # 后4位是距离（毫米）
                    distance_mm = int(sensor_info[1:5])
                    
                    if 0 <= sensor_index <= 7:
                        self.sensor_readings[sensor_index] = distance_mm
                        self.last_update_time[sensor_index] = current_time
                        updated_sensors.append(sensor_index)
                    else:
                        self.get_logger().warn(f'传感器索引超出范围: {sensor_index}')
                except ValueError as e:
                    self.get_logger().warn(f'解析传感器数据失败: {sensor_info}, 错误: {e}')
                    continue
            
            if updated_sensors:
                self.get_logger().debug(f'更新了 {len(updated_sensors)} 个传感器: {updated_sensors}')
                # 发布更新的传感器数据
                self.publish_sensor_data(updated_sensors)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'解析传感器消息失败: {e}')
            import traceback
            self.get_logger().error(f'堆栈跟踪: {traceback.format_exc()}')
            return False
    
    def publish_sensor_data(self, sensor_indices=None):
        """发布传感器数据"""
        if sensor_indices is None:
            sensor_indices = range(8)
        
        current_time = self.get_clock().now()
        
        # 发布每个传感器的Range消息
        for i in sensor_indices:
            if self.sensor_readings[i] is not None:
                range_msg = Range()
                range_msg.header.stamp = current_time.to_msg()
                range_msg.header.frame_id = f'ultrasonic_sensor_{i+1}'  # 显示编号从1开始
                range_msg.radiation_type = Range.ULTRASOUND
                range_msg.field_of_view = 0.1  # 约5.7度（典型超声波传感器）
                range_msg.min_range = 0.02  # 2cm最小距离
                range_msg.max_range = 4.0   # 4m最大距离
                range_msg.range = self.sensor_readings[i] / 1000.0  # 转换为米
                
                self.sensor_pubs[i].publish(range_msg)
        
        # 发布所有传感器数据的数组
        from std_msgs.msg import Float32MultiArray, MultiArrayDimension
        array_msg = Float32MultiArray()
        
        # 设置维度信息
        dim = MultiArrayDimension()
        dim.label = "sensors"
        dim.size = 8
        dim.stride = 8
        array_msg.layout.dim = [dim]
        
        # 填充数据（None转换为-1表示无效）
        array_msg.data = [
            (self.sensor_readings[i] / 1000.0) if self.sensor_readings[i] is not None else -1.0
            for i in range(8)
        ]
        
        self.all_sensors_pub.publish(array_msg)
    
    def read_serial_loop(self):
        """串口读取循环（在独立线程中运行）"""
        buffer = ''
        
        while self.running:
            if self.serial_conn is None or not self.serial_conn.is_open:
                time.sleep(1.0)
                # 尝试重新初始化
                if self.serial_conn is None:
                    self.init_serial()
                continue
            
            try:
                # 读取数据
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting).decode('ascii', errors='ignore')
                    buffer += data
                    
                    # 查找完整的消息（以\r\n结尾）
                    while '\r\n' in buffer:
                        line_end = buffer.find('\r\n')
                        message = buffer[:line_end + 2]
                        buffer = buffer[line_end + 2:]
                        
                        # 解析消息
                        if message.startswith('$udm1;'):
                            self.parse_sensor_message(message)
                        else:
                            self.get_logger().debug(f'收到其他消息: {message.strip()}')
                else:
                    time.sleep(0.01)  # 短暂休眠避免CPU占用过高
                    
            except serial.SerialException as e:
                self.get_logger().error(f'串口读取错误: {e}')
                self.serial_conn.close()
                self.serial_conn = None
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f'读取串口数据时出错: {e}')
                time.sleep(0.1)
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info('正在关闭超声波传感器节点...')
        self.running = False
        
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('串口已关闭')
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = UltrasonicSensorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

