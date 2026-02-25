#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from sensor_msgs.msg import Image

def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = StorageOptions(uri=path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)
    return storage_options, converter_options

def process_bag(bag_path, topic_name, output_dir):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")

    reader = SequentialReader()
    storage_options, converter_options = get_rosbag_options(bag_path)
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag file: {e}")
        # Try mcap if sqlite3 fails
        try:
            storage_options.storage_id = 'mcap'
            reader.open(storage_options, converter_options)
            print("Opened as MCAP format.")
        except Exception as e_mcap:
            print(f"Error opening bag file as MCAP: {e_mcap}")
            return

    # Filter by topic
    storage_filter = StorageFilter(topics=[topic_name])
    reader.set_filter(storage_filter)

    bridge = CvBridge()
    count = 1
    
    print(f"Processing bag: {bag_path}")
    print(f"Target topic: {topic_name}")
    print(f"Output directory: {output_dir}")

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        
        if topic == topic_name:
            try:
                msg = deserialize_message(data, Image)
                cv_image = None

                # Handle 16-bit 240x100 raw images
                if msg.width == 240 and msg.height == 100 and (msg.encoding == 'mono16' or msg.encoding == '16UC1'):
                    # Save raw binary data
                    raw_filename = os.path.join(output_dir, f"{count}.raw")
                    with open(raw_filename, 'wb') as f:
                        f.write(msg.data)
                    
                    if count % 10 == 0:
                        print(f"Saved {count} images (raw)...", end='\r')
                    count += 1
                    continue

                # Handle NV12 format manually
                if msg.encoding == 'nv12':
                    # NV12 is YUV 4:2:0, 1.5 bytes per pixel
                    # Height is 1.5 * height of the image
                    # Data is stored as Y plane followed by interleaved UV plane
                    
                    # Convert bytes to numpy array
                    # Note: msg.data is a list of bytes in Python ROS2, need to convert to bytearray/bytes first if needed
                    # But numpy.frombuffer works with bytes-like objects
                    
                    # Ensure data is correct type
                    img_data = np.frombuffer(msg.data, dtype=np.uint8)
                    
                    height = msg.height
                    width = msg.width
                    
                    # Reshape to (height * 1.5, width) for cv2.cvtColor
                    # NV12 size check
                    expected_size = int(width * height * 1.5)
                    if img_data.size != expected_size:
                        print(f"Warning: NV12 data size mismatch. Expected {expected_size}, got {img_data.size}. Skipping frame {count}.")
                        continue
                        
                    yuv_mat = img_data.reshape((int(height * 1.5), width))
                    cv_image = cv2.cvtColor(yuv_mat, cv2.COLOR_YUV2BGR_NV12)
                    
                else:
                    # Handle other formats (Bayer, Mono, RGB, etc.) using CvBridge
                    # desired_encoding='bgr8' will automatically convert Bayer/Mono to BGR
                    try:
                        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    except CvBridgeError as e:
                        print(f"CvBridge Error: {e}")
                        continue

                if cv_image is not None:
                    output_filename = os.path.join(output_dir, f"{count}.jpg")
                    cv2.imwrite(output_filename, cv_image)
                    if count % 10 == 0:
                        print(f"Saved {count} images...", end='\r')
                    count += 1
                    
            except Exception as e:
                print(f"Error processing message: {e}")
                continue

    print(f"\nDone. Saved {count - 1} images to {output_dir}")

def main():
    parser = argparse.ArgumentParser(description="Extract images from a ROS2 bag file.")
    # 设置默认参数：default='默认值'
    parser.add_argument("--bag_path", default="/home/zzh/cam_extrinsic_calibtool/bag/rosbag2_2025_06_04-23_49_25", help="Path to the ROS2 bag file (folder or .mcap/.db3 file).")
    parser.add_argument("--topic", default="/ir_flood_tof1", help="Name of the image topic to extract.")
    parser.add_argument("--output_dir", default="/home/zzh/cam_extrinsic_calibtool/images/ir_tof1_flood", help="Directory to save the extracted images.")

    args = parser.parse_args()

    bag_path = args.bag_path
    topic = args.topic
    output_dir = args.output_dir

    print(f"Using configuration:")
    print(f"  Bag Path:   {bag_path}")
    print(f"  Topic:      {topic}")
    print(f"  Output Dir: {output_dir}")

    # Validate inputs
    if not bag_path or not topic or not output_dir:
        print("Error: All parameters (bag_path, topic, output_dir) are required.")
        return

    process_bag(bag_path, topic, output_dir)

if __name__ == "__main__":
    main()
