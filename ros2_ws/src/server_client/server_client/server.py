#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # simple service with no input, just success + message
import subprocess
import json
import os

class RGBDDetectionServer(Node):
    def __init__(self):
        super().__init__('rgbd_detection_server')
        self.srv = self.create_service(Trigger, 'detect_object', self.handle_detection_request)
        
        # Configs - change these paths so that they match with your setup!
        self.ros2_package = 'rgbd_process'    # ros2 package name with rgbd_recorder node
        self.ros2_node = 'recorder'          # the executable node for RGBD capture
        self.data_folder = '~/subscribed_data'  # where images are saved
        script_dir = os.path.dirname(os.path.realpath(__file__))
        #self.get_logger().info(f"current directory path is {script_dir}")
        self.label_file = '/homes/tlei/Robotics_Lab/labels.txt'       # label file path
        self.grounding_dino_script = '/homes/tlei/Robotics_Lab/dino_3d_location.py'  # path to your detection script

        self.get_logger().info("RGBD Detection Server is ready.")

    def handle_detection_request(self, request, response):
        self.get_logger().info("Received detection request.")
        
        # Step 1: Run ROS2 node to capture one RGBD frame
        self.get_logger().info("Starting RGB-D capture...")
        proc = subprocess.run(
            ["ros2", "run", self.ros2_package, self.ros2_node],
            capture_output=True,
            text=True
        )
        if proc.returncode != 0:
            response.success = False
            response.message = f"RGB-D capture failed: {proc.stderr}"
            self.get_logger().error(response.message)
            return response

        self.get_logger().info("RGB-D capture complete.")

        # Step 2: Run Grounding DINO detection
        self.get_logger().info("Running Grounding DINO detection...")
        proc = subprocess.run(
            ["python3", self.grounding_dino_script,
             "--folder", self.data_folder,
             "--label", self.label_file],
            capture_output=True,
            text=True
        )
        if proc.returncode != 0:
            response.success = False
            response.message = f"Grounding DINO detection failed: {proc.stderr}"
            self.get_logger().error(response.message)
            return response

        self.get_logger().info("Detection complete.")

        # Step 3: Read the detection JSON result from output folders
        # Assuming output folder format: output/{left_camera,right_camera}/location.json
        left_json_path = os.path.join("output", "left_camera", "location.json")
        right_json_path = os.path.join("output", "right_camera", "location.json")

        results = {}
        try:
            with open(left_json_path, "r") as f:
                results['left_camera'] = json.load(f)
            with open(right_json_path, "r") as f:
                results['right_camera'] = json.load(f)
        except Exception as e:
            response.success = False
            response.message = f"Failed to load detection results: {str(e)}"
            self.get_logger().error(response.message)
            return response

        # Step 4: Return results as JSON string
        response.success = True
        response.message = json.dumps(results)
        self.get_logger().info("Sending detection results to client.")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RGBDDetectionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
