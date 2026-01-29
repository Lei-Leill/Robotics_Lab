#!/usr/bin/bash
echo "Right Camera"
source /opt/ros/humble/setup.bash
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=right_camera -p serial_no:=_239222302270 -p align_depth.enable:=true

