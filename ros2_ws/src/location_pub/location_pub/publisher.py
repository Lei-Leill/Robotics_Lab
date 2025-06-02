import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import time

class JSONPublisher(Node):
    def __init__(self):
        super().__init__('json_publisher')

        self.publisher_ = self.create_publisher(String, 'detection_json', 10)

        self.json_paths = [
            '/homes/tlei/Robotic/Robotics_Lab/output/left_camera/location.json',
            '/homes/tlei/Robotic/Robotics_Lab/output/right_camera/location.json'
        ]

        self.index = 0  # To track which file to publish next

        # Create a timer to publish each file 1 second apart
        self.timer = self.create_timer(1.0, self.publish_next)

    def publish_next(self):
        if self.index >= len(self.json_paths):
            self.get_logger().info("All files published. Shutting down.")
            self.timer.cancel()
            #rclpy.shutdown()
            self.get_logger().info("Keeping node alive for debugging.")

            return

        json_path = self.json_paths[self.index]
        self.index += 1

        if not os.path.exists(json_path):
            self.get_logger().error(f"JSON file not found: {json_path}")
            return

        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
            msg = String()
            msg.data = json.dumps(data)
            self.get_logger().info(f"Message published is: {msg.data}")
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published JSON from: {json_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish {json_path}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JSONPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
