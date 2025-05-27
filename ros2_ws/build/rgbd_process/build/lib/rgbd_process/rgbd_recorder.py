import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class RGBDRecorder(Node):
    def __init__(self):
        super().__init__('rgbd_recorder')
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None

        self.create_subscription(Image, '/camera/camera/color/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.cam_info_callback, 10)

        self.frame_limit = 5
        self.frame_count = 0
        self.save_dir = 'rgbd_data'
        os.makedirs(self.save_dir, exist_ok=True)

    def cam_info_callback(self, msg):
        self.get_logger().info("Camera Info received.")
        # Only log once
        self.destroy_subscription(self.cam_info_callback)

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.save_images()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Depth image is 16UC1 (unsigned short, mm)

    def save_images(self):
        if self.rgb_image is not None and self.depth_image is not None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S%f')

            rgb_path = os.path.join(self.save_dir, f'rgb_{timestamp}.png')
            depth_path = os.path.join(self.save_dir, f'depth_{timestamp}.png')

            cv2.imwrite(rgb_path, self.rgb_image)
            cv2.imwrite(depth_path, self.depth_image)  # Saved as 16-bit PNG

            self.get_logger().info(f"Saved RGB to {rgb_path}")
            self.get_logger().info(f"Saved Depth to {depth_path}")

            self.frame_count += 1
            if self.frame_count >= self.frame_limit:
                self.get_logger().info("Frame limit reached. Shutting down.")
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
