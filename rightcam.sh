#!/usr/bin/bash
echo "Right Camera"
source /bigdata/thao/.env/bin/activate
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=right_camera -p serial_no:=_239222302270 -p align_depth.enable:=true

