#!/usr/bin/bash
echo "Client"
cd ros2_ws
colcon build
source install/setup.bash
ros2 run server_client client
