import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
from datetime import datetime

class ImageDataCollector(Node):
    def __init__(self):
        super().__init__('image_data_collector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.latest_frame = None

        # Base dataset path
        self.base_path = "/home/ricard/ros2_ws_puzzlebot/src/mobile_robotics_cv_line_tracker/dataset_LightsV2"
        os.makedirs(self.base_path, exist_ok=True)

        self.get_logger().info("Image data collector node started!")

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.takeSS(self.latest_frame)
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

    def takeSS(self, frame):
        cv2.imshow("Original", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            self.get_logger().info("Shutting down node.")
            self.destroy_node()
            rclpy.shutdown()

        elif key in [ord('r'), ord('g'), ord('y'), ord('d'), ord('s')]:
            label_map = {
                ord('r'): 'red',
                ord('g'): 'green',
                ord('y'): 'yellow',
                ord('d'): 'directions',
                ord('s'): 'stops'
            }

            label = label_map[key]
            label_path = os.path.join(self.base_path, label)
            os.makedirs(label_path, exist_ok=True)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(label_path, f"{label}_image_{timestamp}.jpg")
            cv2.imwrite(filename, frame)
            self.get_logger().info(f"Screenshot saved to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageDataCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
