import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

class HSVTuner(Node):
    def __init__(self):
        super().__init__('hsv_tuner')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.image = None

        # Create OpenCV window and trackbars
        cv2.namedWindow("HSV Controls", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("HSV Controls", 400, 300)

        cv2.createTrackbar("H Min", "HSV Controls", 0, 179, lambda x: None)
        cv2.createTrackbar("H Max", "HSV Controls", 179, 179, lambda x: None)
        cv2.createTrackbar("S Min", "HSV Controls", 0, 255, lambda x: None)
        cv2.createTrackbar("S Max", "HSV Controls", 255, 255, lambda x: None)
        cv2.createTrackbar("V Min", "HSV Controls", 0, 255, lambda x: None)
        cv2.createTrackbar("V Max", "HSV Controls", 255, 255, lambda x: None)

        self.timer = self.create_timer(0.05, self.update_display)
        self.get_logger().info("HSV Tuner node started!")

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

    def update_display(self):
        if self.image is None:
            return

        # Get slider values
        h_min = cv2.getTrackbarPos("H Min", "HSV Controls")
        h_max = cv2.getTrackbarPos("H Max", "HSV Controls")
        s_min = cv2.getTrackbarPos("S Min", "HSV Controls")
        s_max = cv2.getTrackbarPos("S Max", "HSV Controls")
        v_min = cv2.getTrackbarPos("V Min", "HSV Controls")
        v_max = cv2.getTrackbarPos("V Max", "HSV Controls")

        # Convert to HSV and apply mask
        self.image = cv2.resize(self.image, (800, 600))
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(self.image, self.image, mask=mask)

        # Show original and mask
        cv2.imshow("Original", self.image)
        cv2.imshow("Mask", mask)
        cv2.imshow("Filtered", result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = HSVTuner()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
