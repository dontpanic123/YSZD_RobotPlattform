#!/usr/bin/env python3
"""
RealSense设备诊断脚本
用于检查连接的RealSense设备状态和序列号
"""

import pyrealsense2 as rs
import sys

def check_devices():
    print("=" * 60)
    print("RealSense设备诊断")
    print("=" * 60)
    
    ctx = rs.context()
    devices = ctx.query_devices()
    device_count = devices.size()
    
    print(f"\n检测到 {device_count} 个RealSense设备\n")
    
    accessible_devices = []
    inaccessible_devices = []
    
    for i in range(device_count):
        print(f"设备 {i}:")
        try:
            dev = devices[i]
            try:
                serial = dev.get_info(rs.camera_info.serial_number)
                name = dev.get_info(rs.camera_info.name)
                usb_type = dev.get_info(rs.camera_info.usb_type_descriptor) if hasattr(rs.camera_info, 'usb_type_descriptor') else "N/A"
                
                print(f"  ✓ 可访问")
                print(f"  序列号: {serial}")
                print(f"  名称: {name}")
                print(f"  USB类型: {usb_type}")
                
                # 尝试创建pipeline测试
                try:
                    pipeline = rs.pipeline(ctx)
                    config = rs.config()
                    config.enable_device(serial)
                    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                    profile = pipeline.start(config)
                    print(f"  ✓ Pipeline测试: 成功")
                    pipeline.stop()
                except Exception as e:
                    print(f"  ⚠ Pipeline测试失败: {e}")
                
                accessible_devices.append({
                    'index': i,
                    'serial': serial,
                    'name': name
                })
                print()
            except RuntimeError as e:
                error_msg = str(e)
                print(f"  ✗ 无法访问设备信息")
                print(f"  错误: {error_msg}")
                if 'bad optional access' in error_msg.lower():
                    print(f"  可能原因: 设备被其他进程占用或需要权限")
                inaccessible_devices.append(i)
                print()
        except Exception as e:
            print(f"  ✗ 枚举失败: {e}")
            inaccessible_devices.append(i)
            print()
    
    print("=" * 60)
    print("总结:")
    print(f"  总设备数: {device_count}")
    print(f"  可访问: {len(accessible_devices)}")
    print(f"  不可访问: {len(inaccessible_devices)}")
    
    if accessible_devices:
        print("\n可访问的设备序列号:")
        for idx, dev_info in enumerate(accessible_devices):
            print(f"  [{idx}] {dev_info['serial']} ({dev_info['name']})")
    
    if inaccessible_devices:
        print(f"\n不可访问的设备索引: {inaccessible_devices}")
        print("\n建议:")
        print("  1. 检查是否有其他程序正在使用这些设备")
        print("  2. 运行: sudo fuser /dev/video* 查看占用进程")
        print("  3. 尝试重新插拔USB设备")
        print("  4. 检查USB权限: ls -l /dev/video*")
    
    print("=" * 60)
    
    return accessible_devices, inaccessible_devices

if __name__ == '__main__':
    try:
        accessible, inaccessible = check_devices()
        if len(accessible) == 0:
            print("\n❌ 没有可访问的设备!")
            sys.exit(1)
        elif len(accessible) < 2 and len(inaccessible) > 0:
            print("\n⚠️  检测到多个设备，但部分无法访问")
            print("   如果需要在launch文件中指定序列号，请使用上面的序列号")
        else:
            print("\n✅ 设备检查完成")
    except Exception as e:
        print(f"\n❌ 检查失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


