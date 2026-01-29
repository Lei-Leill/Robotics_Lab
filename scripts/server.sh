#!/usr/bin/bash
echo "Server"
cd ros2_ws
colcon build
source install/setup.bash
source /bigdata/thao/.env/bin/activate
ros2 run server_client server
