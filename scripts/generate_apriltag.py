#!/usr/bin/env python3

import cv2
import numpy as np
import argparse
import os

def generate_apriltag(tag_id, size, output_dir="apriltags"):
    """生成AprilTag标记"""
    
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    # 获取AprilTag字典
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    
    # 生成AprilTag
    tag_image = cv2.aruco.generateImageMarker(dictionary, tag_id, size)
    
    # 保存图像
    filename = f"apriltag_{tag_id:03d}_{size}x{size}.png"
    filepath = os.path.join(output_dir, filename)
    cv2.imwrite(filepath, tag_image)
    
    print(f"✅ 生成AprilTag ID: {tag_id}")
    print(f"📐 尺寸: {size}x{size} 像素")
    print(f"💾 保存到: {filepath}")
    
    return filepath

def generate_multiple_tags(start_id, count, size, output_dir="apriltags"):
    """生成多个AprilTag标记"""
    
    print(f"🎯 生成 {count} 个AprilTag标记 (ID: {start_id} - {start_id + count - 1})")
    print("=" * 50)
    
    generated_files = []
    
    for i in range(count):
        tag_id = start_id + i
        filepath = generate_apriltag(tag_id, size, output_dir)
        generated_files.append(filepath)
    
    print(f"\n✅ 成功生成 {count} 个AprilTag标记")
    print(f"📁 保存目录: {output_dir}")
    
    return generated_files

def create_printable_sheet(tag_ids, size, output_file="apriltag_sheet.png"):
    """创建可打印的AprilTag表格"""
    
    # 计算表格尺寸
    cols = 4  # 每行4个标签
    rows = (len(tag_ids) + cols - 1) // cols
    
    # 标签间距
    margin = 20
    spacing = 10
    
    # 计算总尺寸
    total_width = cols * size + (cols - 1) * spacing + 2 * margin
    total_height = rows * size + (rows - 1) * spacing + 2 * margin
    
    # 创建白色背景
    sheet = np.ones((total_height, total_width), dtype=np.uint8) * 255
    
    # 获取字典
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    
    # 放置每个标签
    for i, tag_id in enumerate(tag_ids):
        row = i // cols
        col = i % cols
        
        # 计算位置
        x = margin + col * (size + spacing)
        y = margin + row * (size + spacing)
        
        # 生成标签
        tag_image = cv2.aruco.generateImageMarker(dictionary, tag_id, size)
        
        # 放置到表格中
        sheet[y:y+size, x:x+size] = tag_image
        
        # 添加ID标签
        cv2.putText(sheet, f"ID: {tag_id}", (x, y-5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, 0, 1)
    
    # 保存表格
    cv2.imwrite(output_file, sheet)
    print(f"📄 可打印表格已保存: {output_file}")
    print(f"📐 表格尺寸: {total_width}x{total_height} 像素")
    
    return output_file

def main():
    parser = argparse.ArgumentParser(description='生成AprilTag标记')
    parser.add_argument('--id', type=int, default=0, help='AprilTag ID')
    parser.add_argument('--size', type=int, default=200, help='标签尺寸（像素）')
    parser.add_argument('--count', type=int, default=1, help='生成数量')
    parser.add_argument('--start-id', type=int, default=0, help='起始ID')
    parser.add_argument('--output-dir', type=str, default='apriltags', help='输出目录')
    parser.add_argument('--sheet', action='store_true', help='生成可打印表格')
    
    args = parser.parse_args()
    
    print("🎯 AprilTag标记生成器")
    print("=" * 30)
    
    if args.count > 1:
        # 生成多个标签
        tag_ids = list(range(args.start_id, args.start_id + args.count))
        generate_multiple_tags(args.start_id, args.count, args.size, args.output_dir)
        
        if args.sheet:
            # 生成可打印表格
            create_printable_sheet(tag_ids, args.size, f"{args.output_dir}/apriltag_sheet.png")
    else:
        # 生成单个标签
        generate_apriltag(args.id, args.size, args.output_dir)

if __name__ == '__main__':
    main()

















