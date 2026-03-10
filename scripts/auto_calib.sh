#!/bin/bash

echo "步骤 4: 执行 topic_to_jpg.py"
python3 "topic_to_jpg.py" 

echo "步骤 5: 执行 ros2 run cam_extrinsic_calibtool calib_node"
source ../../../install/setup.bash
ros2 run cam_extrinsic_calibtool calib_node
