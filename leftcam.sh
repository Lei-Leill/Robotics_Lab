#!/usr/bin/bash
echo "Left Camera"
source /bigdata/thao/.env/bin/activate
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=left_camera -p serial_no:=_239222303493 -p align_depth.enable:=true
