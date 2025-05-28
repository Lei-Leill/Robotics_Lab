import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import json

class RGBDRecorder(Node):
    def __init__(self):
        super().__init__('rgbd_recorder')
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None
        
        camera_name = ['left_camera', 'right_camera']
        for i in range(2):
            self.save_dir = f'/homes/tlei/Robotic/suscribed_data/{camera_name[i]}'
            os.makedirs(self.save_dir, exist_ok=True)
            self.create_subscription(CameraInfo, f'/camera/{camera_name[i]}/color/camera_info', self.cam_info_callback, 10)
            self.create_subscription(Image, f'/camera/{camera_name[i]}/color/image_raw', self.rgb_callback, 10)
            self.create_subscription(Image, f'/camera/{camera_name[i]}/aligned_depth_to_color/image_raw', self.depth_callback, 10)
            self.subscriptions.append(cam_info_sub)
        self.frame_limit = 1
        self.frame_count = 0

    def cam_info_callback(self, msg):
        self.get_logger().info("Camera Info received.")
        # Log relevant camera info (avoid logging the entire msg object)
        self.get_logger().info(f"Height: {msg.height}, Width: {msg.width}")
        self.get_logger().info(f"Focal Length X: {msg.k[0]}, Focal Length Y: {msg.k[4]}")
        self.get_logger().info(f"Principal Point X: {msg.k[2]}, Principal Point Y: {msg.k[5]}")

        # Extract relevant camera info fields
        camera_info = {
            'focal_length_x': msg.k[0],  # fx
            'focal_length_y': msg.k[4],  # fy
            'principal_point_x': msg.k[2],  # cx
            'principal_point_y': msg.k[5],  # cy
        }
        
        # Convert the camera info to a JSON string
        camera_info_json = json.dumps(camera_info, indent=4)

        # Save the JSON to a file
        json_filename = os.path.join(self.save_dir, 'camera_info.json')
        
        with open(json_filename, 'w') as json_file:
            json_file.write(camera_info_json)
        
        self.get_logger().info(f"Saved Camera Info to {json_filename}")
        for sub in self.subscriptions:
            sub.destroy()  # Unsubscribe

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.save_images()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Depth image is 16UC1 (unsigned short, mm)

    def save_images(self):
        print('start the work')
        if self.rgb_image is not None and self.depth_image is not None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S%f')

            rgb_path = os.path.join(self.save_dir, f'rgb_{timestamp}.png')
            depth_path = os.path.join(self.save_dir, f'depth_{timestamp}.png')

            cv2.imwrite(rgb_path, self.rgb_image)
            cv2.imwrite(depth_path, self.depth_image)  # Saved as 16-bit PNG

            self.get_logger().info(f"Saved RGB to {rgb_path}")
            self.get_logger().info(f"Saved Depth to {depth_path}")

            if self.frame_count >= self.frame_limit:
                self.get_logger().info("Frame limit reached. Shutting down.")
                self.frame_count += 1
                rclpy.shutdown()

            # Optionally reset to avoid multiple saves per frame
            self.rgb_image = None
            self.depth_image = None
    
def main(args=None):
    rclpy.init(args=args)
    recorder = RGBDRecorder()
    rclpy.spin(recorder)
    recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
