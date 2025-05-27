# Robotics_Lab

## Commands
```
source /bigdata/thao/.env/bin/activate

ros2 run realsense2_camera realsense2_camera_node --ros-args -p align_depth.enable:=true
```

### To build the ros2 package
Go to the ros2_ws repository
```
colcon build
source intall/setup.bash
ros2 run rgbd_process recorder
```