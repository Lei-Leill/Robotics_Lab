# Robotics_Lab

## Initialize the ROS2 camera topics
If need the real-time image, must launch ros for each camera to begin

Be sure you are in the (.env) virtual environment that has ros2 installed, do this:
```source /bigdata/thao/.env/bin/activate```

Make sure to connect the wires of cameras to computer and launch two cameras separately in two terminals 
```
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=left_camera -p serial_no:=_239222303493 -p align_depth.enable:=true
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=right_camera -p serial_no:=_239222302270 -p align_depth.enable:=true
```
If you want to testify whether they are launched successfully, do
```ros2 topic list```

After this, you can proceed with the server and client packages 

## Server (Kiki) and Client (Chihiro) package 
They are in ```Robotics_Lab/ros2_ws/src/server_client/server_client```, ```server.py``` will call dino_3d_location.py and the rgbd_process pkg to take real-time images, ```client.py``` send request to server

### Before running it:
Check that the folder structure and location of files match with your machine

In server.py in directory ```Robotics_Lab/ros2_ws/server_client/server_client/server.py```

Change the paths that accords with where the files are in your machine
```
self.label_file = '/homes/tlei/Robotics_Lab/labels.txt'       # label file path
self.grounding_dino_script = '/homes/tlei/Robotics_Lab/dino_3d_location.py'  # path to your detection script
```

Once you are set up, you don't need to do these next time, just run it with following steps!

### Steps to run it
1. Go to the ```ros2_ws``` repository

2. Do the following steps to build the packages, source to the environment that ros2 run, and run the server

```
colcon build
source install/setup.bash
ros2 run server_client server
```

Server would be initiated and waiting for request from the client.

3. Open another terminal (Kiki or Chihiro) and the ```ros2_ws``` folder. Do the following:
```
colcon build
source install/setup.bash
ros2 run server_client client
```
Client is requesting the server to do two subprocess: (1) capture rgb and depth images from left and right cameras, and (2) run the grounding dino and send the (x,y,z) location information back to the client. 

You can see the information sent back in the terminal after the server has finished with all the process.

### Important things to keep in mind
1. Must initialize (run) the server before sending request by running the client
2. You can control what object you want to detect and get location info by making changes the labels.txt 


## How the Object Detection is done

You can find the script in ```dino_3d_location.py```

The model takes in labels, folder that contains camera information (focal length etc), rgb image, and depth image. The labels tell the Object Detection model (DINO) what to detect and spot. 

**Object Detection**

After the mode spots the object in the rgb image, it will return x_min, x_max, y_min, y_max, which can form a bounding box around it. Like this: 

**Transform 2D to 3D:**

We first compute the center of the detected object in the 2D image by averaging the minimum and maximum x and y pixel coordinates. Next, we retrieve the corresponding depth value (i.e., the distance from the camera) from the depth image.
However, the x and y values obtained from object detection are in pixel units, while we need meters. To convert pixel coordinates to metric units relative to the camera, we use the depth value z (in meters) and the camera's focal length f (also in meters) to perform a perspective projection transformation.

**Camera View to Real-world Robot view**

The coordinates at this stage are still relative to the individual (left/right) camera. To express them in a global frame relative to the robot or world coordinate system, we apply the camera extrinsic parameters (rotation and translation) obtained from camera calibration. This transformation uses the camera calibration matrix to align the camera's view with the robot’s coordinate frame.


## Just run the object detection with Dino

subscribed_data folder contains camera information, rgb image, depth image for left and right camera

you can make changes the labels.txt to control what object you want to detect and get location info.
```
python3 dino_3d_location.py --folder suscribed_data --label labels.txt
```
