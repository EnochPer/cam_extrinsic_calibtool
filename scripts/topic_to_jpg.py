#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import cv2
import numpy as np
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def convert_raw_to_jpg(raw_path, output_dir, output_filename, raw_to_bmp_executable):
    """
    Convert RAW file to JPG via intermediate BMP.
    Copied from scripts/raw_to_calib_jpg.py
    """
    try:
        # 1. Construct paths
        filename = os.path.basename(raw_path)
        name_without_ext = os.path.splitext(filename)[0]
        
        # Temporary BMP path
        temp_bmp_path = os.path.join(output_dir, f"{name_without_ext}_temp.bmp")
        
        # Final JPG path
        jpg_path = os.path.join(output_dir, output_filename)

        # 2. Call C++ executable to convert RAW to BMP
        # Usage: ./RawToBmpDemo <input_raw> <output_bmp>
        cmd = [raw_to_bmp_executable, raw_path, temp_bmp_path]
        
        # print(f"Running: {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode != 0:
            print(f"[ERROR] RawToBmpDemo failed for {filename}:")
            print(result.stderr)
            print(result.stdout)
            return False

        # Check if BMP was actually created
        if not os.path.exists(temp_bmp_path):
             # Fallback check: maybe it appended .bmp?
            if os.path.exists(temp_bmp_path + ".bmp"):
                temp_bmp_path += ".bmp"
            else:
                print(f"[ERROR] BMP file not found after execution: {temp_bmp_path}")
                return False

        # 3. Read BMP with OpenCV
        image = cv2.imread(temp_bmp_path)
        if image is None:
            print(f"[ERROR] Failed to load intermediate BMP: {temp_bmp_path}")
            # Clean up even if failed
            if os.path.exists(temp_bmp_path):
                os.remove(temp_bmp_path)
            return False

        # 4. Rotate image 180 degrees
        image = cv2.rotate(image, cv2.ROTATE_180)

        # 5. Save as JPG
        cv2.imwrite(jpg_path, image)
        # print(f"[OK] Converted: {filename} -> {output_filename}")

        # 6. Clean up temporary BMP
        if os.path.exists(temp_bmp_path):
            os.remove(temp_bmp_path)
            
        return True

    except Exception as e:
        print(f"[ERROR] Failed to convert {raw_path}: {e}")
        return False

class ImageSaverNode(Node):
    def __init__(self, topic1, topic2, output_dir, raw_to_bmp_exec):
        super().__init__('image_saver_node')
        self.output_dir = output_dir
        self.raw_to_bmp_exec = raw_to_bmp_exec
        self.bridge = CvBridge()
        
        # Track if we have saved images for each topic
        self.saved_tof1 = False
        self.saved_tof2 = False
        
        # Counters for received frames
        self.count_tof1 = 0
        self.count_tof2 = 0

        # Subscribe to topics
        self.sub1 = self.create_subscription(
            Image, topic1, lambda msg: self.image_callback(msg, "tof1"), 10)
        self.sub2 = self.create_subscription(
            Image, topic2, lambda msg: self.image_callback(msg, "tof2"), 10)
        
        self.get_logger().info(f"Subscribed to {topic1} and {topic2}")
        self.get_logger().info(f"Saving images to {output_dir}")
        self.get_logger().info(f"Using RawToBmp executable: {raw_to_bmp_exec}")

    def image_callback(self, msg, prefix):
        # Check if we already saved this topic
        if prefix == "tof1":
            if self.saved_tof1:
                return
            self.count_tof1 += 1
            if self.count_tof1 != 5:
                return
        elif prefix == "tof2":
            if self.saved_tof2:
                return
            self.count_tof2 += 1
            if self.count_tof2 != 5:
                return

        try:
            # Handle 16-bit 240x181 raw images
            # Note: msg.height might be 181, msg.width might be 240
            # The user mentioned "240*181*2", which implies 16-bit (2 bytes) per pixel
            print("count: ", self.count_tof1, self.count_tof2)
            
            if msg.width == 240 and msg.height == 362:
                # Save raw binary data
                raw_filename = f"{prefix}.raw"
                raw_path = os.path.join(self.output_dir, raw_filename)
                
                # msg.data is array.array or bytes in ROS2 python
                # We need to ensure it's bytes for writing
                data_bytes = msg.data
                if not isinstance(data_bytes, bytes):
                     # If it's array.array (e.g. from some bridges), convert to bytes
                     try:
                         data_bytes = data_bytes.tobytes()
                     except:
                         pass

                with open(raw_path, 'wb') as f:
                    f.write(data_bytes)
                
                # Convert to JPG
                jpg_filename = f"{prefix}.jpg"
                if convert_raw_to_jpg(raw_path, self.output_dir, jpg_filename, self.raw_to_bmp_exec):
                    self.get_logger().info(f"Successfully saved and converted {prefix}.jpg")
                    
                    # Mark as saved
                    if prefix == "tof1":
                        self.saved_tof1 = True
                    elif prefix == "tof2":
                        self.saved_tof2 = True
                    
                    # Check if both are done
                    if self.saved_tof1 and self.saved_tof2:
                        self.get_logger().info("Both images saved. Exiting...")
                        # Raise SystemExit to stop the node
                        raise SystemExit
                else:
                    self.get_logger().error(f"Failed to convert image ({prefix})")

            else:
                # Optional: Handle other formats if needed, or just log info
                self.get_logger().warn(f"[{prefix}] Received non-matching image format: {msg.encoding} {msg.width}x{msg.height}")
                pass
                
        except SystemExit:
            raise
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

def main():
    parser = argparse.ArgumentParser(description="Subscribe to ROS2 topics and save raw images as JPG.")
    
    default_out = "/root/3dtof_calib/images/output"
    default_exec = "/root/3dtof_calib/bin/rawToBmpDemo_exec"

    parser.add_argument("--topic1", default="/ir_tof1_raw", help="Name of the first image topic (tof1).")
    parser.add_argument("--topic2", default="/ir_tof2_raw", help="Name of the second image topic (tof2).")
    parser.add_argument("--output_dir", default=default_out, help="Directory to save the extracted images.")
    parser.add_argument("--exec", default=default_exec, help="Path to the RawToBmpDemo executable.")

    args = parser.parse_args()

    topic1 = args.topic1
    topic2 = args.topic2
    output_dir = args.output_dir
    raw_to_bmp_exec = args.exec

    # Validate inputs
    if not output_dir:
        print("Error: output_dir is required.")
        return

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")
        
    if not os.path.isfile(raw_to_bmp_exec):
        print(f"Error: RawToBmpDemo executable not found at: {raw_to_bmp_exec}")
        return

    rclpy.init()
    node = ImageSaverNode(topic1, topic2, output_dir, raw_to_bmp_exec)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
