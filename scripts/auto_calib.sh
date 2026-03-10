#!/bin/bash

echo "步骤 1: 执行 topic_to_jpg.py"
python3 "topic_to_jpg.py" 

echo "步骤 2: 执行 ros2 run 3dtof_calib calib_node"
source ../../../install/setup.bash
ros2 run 3dtof_calib calib_node
