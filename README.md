# Robotics_Lab

## Object detection with DINO

Activate the virtual environment that has DINO installed by running:
```
source /bigdata/thao/.env/bin/activate
```

The `subscribed_data` folder contains camera information, rgb images, depth images for the left and right cameras. You can make changes in the `labels.txt` file to control what object you want to detect, then run:
```
python3 dino_3d_location.py --folder subscribed_data --label labels.txt
```

## Initialize the ROS2 camera topics
If you need real-time images, launch ros for each camera.

Make sure you are on the kiki computer, which has realsense-ros installed. Connect the cameras wires to the computer and launch two camera nodes in two separate terminals:
```
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=left_camera -p serial_no:=_239222303493 -p align_depth.enable:=true
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=right_camera -p serial_no:=_239222302270 -p align_depth.enable:=true
```
If you want to test whether the camera nodes launched successfully, run the following in a new terminal:
```
ros2 topic list
```

After this, you can proceed with the server and client packages.

## Server (Kiki) and Client (Chihiro) package 
Relevant files are in the `Robotics_Lab/ros2_ws/src/server_client/server_client` folder. `server.py` will call `dino_3d_location.py` and the `rgbd_process` pkg to take and process real-time images, `client.py` sends requests to the server.

### Before running it:
Check that the folder structure and location of the files match with your machine.

In ```Robotics_Lab/ros2_ws/server_client/server_client/server.py```, change the paths according to where the files are on your machine:
```
self.label_file = '/homes/tlei/Robotics_Lab/labels.txt'       # label file path
self.grounding_dino_script = '/homes/tlei/Robotics_Lab/dino_3d_location.py'  # path to your detection script
```

Once you are set up, you don't need to do these next time, just run it with following steps!

### Steps to run it:
1. Go to the ```ros2_ws``` repository

2. Do the following steps to build the packages, source the ros2 environment, and run the server:

  ```
  colcon build
  source install/setup.bash
  ros2 run server_client server
  ```

The server would be initiated and waiting for requests from the client.

3. Open another terminal (Kiki or Chihiro) and the ```ros2_ws``` folder. Do the following:
  ```
  colcon build
  source install/setup.bash
  ros2 run server_client client
  ```
The client is requesting the server to do two subprocess: (1) capture rgb and depth images from the left and right cameras, and (2) run the grounding dino script to detect the object that matches the provided label and send the (x,y,z) location information back to the client. 

You can see the information sent back in the terminal after the server has finished with all the processes.

### Important things to keep in mind:
1. Must initialize (run) the server before sending requests by running the client
2. You can control what object you want to detect by making changes in labels.txt 


## How the Object Detection is done

You can find the script in ```dino_3d_location.py```

The model takes in labels, folder that contains camera information (focal length, etc.), rgb image, and depth image. The labels tell the Object Detection model (DINO) what to detect and spot. 

**Object Detection**

After the mode spots the object in the rgb image, it will return the x_min, x_max, y_min, y_max of the bounding box around the object.

**Transform 2D to 3D:**

We first compute the center of the detected object in the 2D image by averaging the minimum and maximum x and y pixel coordinates. Next, we retrieve the corresponding depth value (i.e., the distance from the camera) from the depth image.
However, the x and y values obtained from object detection are in pixel units, while we need meters. To convert pixel coordinates to metric units relative to the camera, we use the depth value z (in meters) and the camera's focal length f (in pixel units) to perform a perspective projection transformation.

**Camera View to Real-world Robot view**

The coordinates at this stage are still relative to the individual (left/right) camera. To express them in a global frame relative to the robot or world coordinate system, we apply the camera extrinsic parameters (rotation and translation) obtained from camera calibration. This transformation uses the camera calibration matrix to align the camera's view with the robot’s coordinate frame.
