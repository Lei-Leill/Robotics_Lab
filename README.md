# Robotics_Lab

## Commands

### To run the realsense_ros
```
source /bigdata/thao/.env/bin/activate
ros2 run realsense2_camera realsense2_camera_node --ros-args -p align_depth.enable:=true
```

### To build the ros2 package
Make sure to have the realsense2_camer running simultaneously

Go to the ros2_ws repository
```
colcon build
source intall/setup.bash
ros2 run rgbd_process recorder
```
Then the depth and rgb images would be saved into rgbd_data