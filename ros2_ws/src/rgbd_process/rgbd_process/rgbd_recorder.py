#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2, os, json
from datetime import datetime

class RGBDRecorder(Node):
    """
    Save exactly one RGB-Depth pair (+ CameraInfo) for the given RealSense
    camera, then set self.done = True so the outer loop can clean up.
    """
    def __init__(self, camera_name: str):
        super().__init__('rgbd_recorder_' + camera_name)
        self.camera_name = camera_name
        self.bridge      = CvBridge()
        self.done        = False           # ← watched by the executor loop

        # buffers
        self.rgb_image   = None
        self.depth_image = None

        # output directory
        self.save_dir = f'~/subscribed_data/{camera_name}'
        os.makedirs(self.save_dir, exist_ok=True)
        if not os.path.exists(self.save_dir):
            raise RuntimeError(f"Directory creation failed: {self.save_dir}")
        self.get_logger().info(f"[{camera_name}] Saving to {self.save_dir}")

        # subscriptions (keep the handles!)
        self.info_sub  = self.create_subscription(
            CameraInfo,
            f'/camera/{camera_name}/color/camera_info',
            self.cam_info_cb,
            10
        )
        self.rgb_sub   = self.create_subscription(
            Image,
            f'/camera/{camera_name}/color/image_raw',
            self.rgb_cb,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            f'/camera/{camera_name}/aligned_depth_to_color/image_raw',
            self.depth_cb,
            10
        )

    # --------------------- callbacks ----------------------------------------
    def cam_info_cb(self, msg: CameraInfo):
        info = {
            "height": msg.height, "width": msg.width,
            "fx": msg.k[0], "fy": msg.k[4],
            "cx": msg.k[2], "cy": msg.k[5],
        }
        with open(os.path.join(self.save_dir, 'camera_info.json'), 'w') as f:
            json.dump(info, f, indent=4)
        self.get_logger().info(f"[{self.camera_name}] camera_info.json written")

        # CameraInfo done → no need to listen any longer
        self.destroy_subscription(self.info_sub)

    def rgb_cb(self, msg: Image):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.try_save_pair()

    def depth_cb(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        self.try_save_pair()

    # --------------------- helper -------------------------------------------
    def try_save_pair(self):
        if self.rgb_image is None or self.depth_image is None:
            return                                      # still waiting

        ts = datetime.now().strftime('%Y%m%d_%H%M%S%f')
        cv2.imwrite(os.path.join(self.save_dir, f'rgb_{ts}.png'),   self.rgb_image)
        cv2.imwrite(os.path.join(self.save_dir, f'depth_{ts}.png'), self.depth_image)
        self.get_logger().info(f"[{self.camera_name}] images saved")

        # stop listening so no more callbacks arrive
        self.destroy_subscription(self.rgb_sub)
        self.destroy_subscription(self.depth_sub)

        self.done = True                                # ← signal “all finished”


def capture_one(camera_name: str):
    node = RGBDRecorder(camera_name)

    exec_ = rclpy.executors.SingleThreadedExecutor()
    exec_.add_node(node)

    while rclpy.ok() and not node.done:
        exec_.spin_once(timeout_sec=0.2)

    # node needs to be destroyed to avoid hanging 
    exec_.remove_node(node)
    node.destroy_node() 
    print(f"{camera_name} is done")

def main(args=None):
    rclpy.init(args=args)

    for cam in ('right_camera', 'left_camera'):
        capture_one(cam)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
