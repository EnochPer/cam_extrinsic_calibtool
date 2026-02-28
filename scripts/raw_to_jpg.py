#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import cv2

def convert_bmp_to_jpg(bmp_path, output_dir):
    """
    Convert BMP file to JPG.
    """
    try:
        # Read image
        image = cv2.imread(bmp_path)
        if image is None:
            print(f"[ERROR] Failed to load image: {bmp_path}")
            return False

        # # Rotate image 180 degrees
        # image = cv2.rotate(image, cv2.ROTATE_180)

        # Construct output path
        filename = os.path.basename(bmp_path)
        name_without_ext = os.path.splitext(filename)[0]
        jpg_path = os.path.join(output_dir, f"{name_without_ext}.jpg")
        
        # Save as JPG
        cv2.imwrite(jpg_path, image)
        print(f"[OK] Converted: {filename} -> {os.path.basename(jpg_path)}")
        return True

    except Exception as e:
        print(f"[ERROR] Failed to convert {bmp_path}: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Convert BMP images to JPG format.")
    parser.add_argument("--input", default="/home/zzh/cam_extrinsic_calibtool/images/tof2", help="Input file path or directory containing .bmp files. Default is current directory.")
    parser.add_argument("--output", default="/home/zzh/cam_extrinsic_calibtool/images/tof2/jpg", help="Output directory for .jpg files. Default is current directory.")

    args = parser.parse_args()

    input_path = args.input
    output_dir = args.output

    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        try:
            os.makedirs(output_dir)
            print(f"Created output directory: {output_dir}")
        except OSError as e:
            print(f"Error creating output directory: {e}")
            sys.exit(1)

    # Collect files to process
    files_to_process = []
    if os.path.isfile(input_path):
        if input_path.lower().endswith('.bmp'):
            files_to_process.append(input_path)
        else:
            print("Error: Input file is not a .bmp file.")
            sys.exit(1)
    elif os.path.isdir(input_path):
        # Non-recursive search for .bmp files
        try:
            for file in os.listdir(input_path):
                file_path = os.path.join(input_path, file)
                if os.path.isfile(file_path) and file.lower().endswith('.bmp'):
                    files_to_process.append(file_path)
        except OSError as e:
            print(f"Error reading directory: {e}")
            sys.exit(1)
    else:
        print(f"Error: Input path '{input_path}' not found.")
        sys.exit(1)

    if not files_to_process:
        print(f"No .bmp files found in '{input_path}'.")
        sys.exit(0)

    print(f"Found {len(files_to_process)} files. Starting conversion...")

    success_count = 0
    for bmp_file in files_to_process:
        if convert_bmp_to_jpg(bmp_file, output_dir):
            success_count += 1

    print(f"\nConversion complete. {success_count}/{len(files_to_process)} files converted successfully.")

if __name__ == "__main__":
    main()
