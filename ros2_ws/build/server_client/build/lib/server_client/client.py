import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('detection_client')
    client = node.create_client(Trigger, 'detect_object')

    if not client.wait_for_service(timeout_sec=5.0):
        print("Service not available")
        return

    req = Trigger.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        if future.result().success:
            print("Detection result:")
            print(future.result().message)  # JSON string of detection data
        else:
            print("Detection failed:", future.result().message)
    else:
        print("Service call failed")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
