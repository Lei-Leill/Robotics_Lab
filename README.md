# Robotics_Lab

## To run the object detection with Dino

The sample inputs containing rbg, depth, labels, camera_info are in the sample_input folder

```
python3 dino_3d_location.py --rgb sample_input/rgb.png --depth sample_input/depth.png --label sample_input/labels.txt --info sample_input/camera_info.json
```

## To get information from the Realsense-ROS

**Do following two steps**

### To run the realsense_camera
```
source /bigdata/thao/.env/bin/activate
#ros2 run realsense2_camera realsense2_camera_node --ros-args -p align_depth.enable:=true

## Launch separately in two terminals 
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=left_camera -p serial_no:=_239222303493 -p align_depth.enable:=true
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=right_camera -p serial_no:=_239222302270 -p align_depth.enable:=true

```

### To build the ros2 package
Make sure to have the realsense2_camera running simultaneously

Go to the ros2_ws repository
```
colcon build
source intall/setup.bash
ros2 run rgbd_process recorder
```
Then the depth and rgb images would be saved into rgbd_data




### We realised the could could make the code more efficient using services
