# Robotics_Lab

## Commands
```
source /bigdata/thao/.env/bin/activate
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=right_camera -r __ns:=robot
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=right -p  __align_depth.enable:=true
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=D455_1 -r __ns:=robot1

ros2 run realsense2_camera realsense2_camera_node --ros-args -p align_depth.enable:=true

```


```
/camera/camera/accel/imu_info
/camera/camera/accel/metadata
/camera/camera/accel/sample
/camera/camera/aligned_depth_to_color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/aligned_depth_to_infra1/camera_info
/camera/camera/aligned_depth_to_infra1/image_raw
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/camera/camera/color/metadata
/camera/camera/depth/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/depth/metadata
/camera/camera/extrinsics/depth_to_accel
/camera/camera/extrinsics/depth_to_color
/camera/camera/extrinsics/depth_to_gyro
/camera/camera/extrinsics/depth_to_infra1
/camera/camera/extrinsics/depth_to_infra2
/camera/camera/gyro/imu_info
/camera/camera/gyro/metadata
/camera/camera/gyro/sample
/camera/camera/infra1/camera_info
/camera/camera/infra1/image_rect_raw
/camera/camera/infra1/metadata
/camera/camera/infra2/camera_info
/camera/camera/infra2/image_rect_raw
/camera/camera/infra2/metadata
/parameter_events
/rosout
/tf_static
```