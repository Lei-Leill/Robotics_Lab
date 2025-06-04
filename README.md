# Robotics_Lab

## Initialize the ROS2 camera topics
If need the real-time image, must launch ros for each camera to begin
if you are not in the (.env) virtual environment, do this:
```source /bigdata/thao/.env/bin/activate```

Launch two cameras separately in two terminals 
```
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=left_camera -p serial_no:=_239222303493 -p align_depth.enable:=true
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=right_camera -p serial_no:=_239222302270 -p align_depth.enable:=true
```
If you want to testify whether they are launched successfully, do
```ros2 topic list```

After this, you can proceed with the server and client packages 

## Server (Kiki) and Client (Chihiro) package 
They are in ros2_ws/src/server_client, server will call dino_3d_location.py and the rgbd_process pkg to take real-time images

### Steps to run it
Go to the ros2_ws repository

Do the following steps to build the packages, source to the environment that ros2 run, and run the server

```
colcon build
source intall/setup.bash
ros2 run server_client server
```

After starting the server, open another terminal, or open a terminal in Chihiro. Do the following:
```
colcon build
source intall/setup.bash
ros2 run server_client client
```

### What it does?
After doing the command ```ros2 run server_client server```, the server would be initiated and waiting for request from the server.

After doing the command ```ros2 run server_client server```, the client is requesting the server to do two subprocess: (1) capture rgb and depth images from left and right cameras, and (2) run the grounding dino and send the (x,y,z) location information back to the client. 

You can see the information sent back in the terminal after the server has finished with all the process.


## Just run the object detection with Dino

suscribed_data folder contains camera information, rgb image, depth image for left and right camera

you can make changes the labels.txt to control what object you want to detect and get location info.
```
python3 dino_3d_location.py --folder suscribed_data --label labels.txt
```