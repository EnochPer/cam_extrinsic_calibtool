#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import cv2
import subprocess
import glob

def convert_raw_to_jpg(raw_path, output_dir, output_filename, raw_to_bmp_executable):
    """
    Convert RAW file to JPG via intermediate BMP.
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
        
        print(f"Running: {' '.join(cmd)}")
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
        print(f"[OK] Converted: {filename} -> {output_filename}")

        # 6. Clean up temporary BMP
        if os.path.exists(temp_bmp_path):
            os.remove(temp_bmp_path)
            
        return True

    except Exception as e:
        print(f"[ERROR] Failed to convert {raw_path}: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Convert two specific RAW images to tof1.jpg and tof2.jpg for calibration.")
    parser.add_argument("--input1", default="/root/tof1_image/RAWDATA_10.raw", help="Path to the first .raw file (will be converted to tof1.jpg).")
    parser.add_argument("--input2", default="/root/tof2_image/RAWDATA_10.raw",  help="Path to the second .raw file (will be converted to tof2.jpg).")
    parser.add_argument("--output", default="/root/3dtof_calib/images/", help="Output directory for .jpg files. Default is ./output")
    parser.add_argument("--exec", default="/root/3dtof_calib/src/3dtof_calib/RawToBmpDemo_240719/build/rawToBmpDemo", help="Path to the compiled RawToBmpDemo executable.")

    args = parser.parse_args()

    input1_path = args.input1
    input2_path = args.input2
    output_dir = args.output
    raw_to_bmp_exec = os.path.abspath(args.exec)

    # Check executable
    if not os.path.isfile(raw_to_bmp_exec):
        print(f"Error: RawToBmpDemo executable not found at: {raw_to_bmp_exec}")
        print("Please compile the C++ tool first.")
        sys.exit(1)
    
    # Ensure executable has execute permissions
    if not os.access(raw_to_bmp_exec, os.X_OK):
        print(f"Warning: {raw_to_bmp_exec} is not executable. Trying to add execute permission...")
        try:
            os.chmod(raw_to_bmp_exec, 0o755)
        except OSError as e:
            print(f"Error setting execute permission: {e}")
            sys.exit(1)

    # Create output directory
    if not os.path.exists(output_dir):
        try:
            os.makedirs(output_dir)
            print(f"Created output directory: {output_dir}")
        except OSError as e:
            print(f"Error creating output directory: {e}")
            sys.exit(1)

    # Process files
    success_count = 0
    
    # Process Input 1 -> tof1.jpg
    if os.path.isfile(input1_path):
        print(f"Processing Input 1: {input1_path} -> tof1.jpg")
        if convert_raw_to_jpg(input1_path, output_dir, "tof1.jpg", raw_to_bmp_exec):
            success_count += 1
    else:
        print(f"[ERROR] Input 1 file not found: {input1_path}")

    # Process Input 2 -> tof2.jpg
    if os.path.isfile(input2_path):
        print(f"Processing Input 2: {input2_path} -> tof2.jpg")
        if convert_raw_to_jpg(input2_path, output_dir, "tof2.jpg", raw_to_bmp_exec):
            success_count += 1
    else:
        print(f"[ERROR] Input 2 file not found: {input2_path}")

    print(f"\nConversion complete. {success_count}/2 files converted successfully.")

if __name__ == "__main__":
    main()
