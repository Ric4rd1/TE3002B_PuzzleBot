"""
Author: Gerardo Sanchez A.k.A SirDuck
File: line_follower.py
Description: ROS2 node that follows a black line using five virtual
 sensors by dividing the lower region of the camera frame into
 five zones and applying a proportional controller for smooth turning.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineFollower(Node):
    """
    ROS2 node for line following based on image processing.
    Splits the bottom of the frame into five zones, detects line,
    and publishes velocity commands proportional to detected error.
    """

    def __init__(self):
        """
        Initialize the node, parameters, subscriptions, and publishers.
        """
        super().__init__('line_follower')

        # Declare and read control parameters
        self.declare_parameter('base_speed', 0.13)
        self.declare_parameter('k_angular', 0.5)
        self.base_speed = self.get_parameter('base_speed').value
        self.k_angular = self.get_parameter('k_angular').value

        # Bridge for ROS Image <-> OpenCV
        self.bridge = CvBridge()

        # Subscribe to raw camera images
        self.create_subscription(
            Image, 'camera', self.camera_callback, 10
        )

        # Publishers: error, visualization image, and velocity commands
        self.error_pub = self.create_publisher(Float32, 'line_error', 10)
        self.image_pub = self.create_publisher(Image, 'line_detection', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Frame data
        self.current_frame = None
        self.image_received = False

        # Timer to process frames at 20Hz
        self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('Line follower node ready')

    def camera_callback(self, msg: Image):
        """
        Callback to receive images from the camera topic.

        Converts ROS Image to OpenCV BGR format.
        """
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def process_frame(
        self, frame: np.ndarray
    ) -> (Twist, np.ndarray, float):  # noqa: C901
        """
        Process a single frame: detect line, compute error, and generate
        velocity command and visualization image.

        Returns:
            twist (Twist): Proportional velocity command.
            vis (np.ndarray): Visualization image with zones and error text.
            error (float): Normalized line position error.
        """
        # Convert to grayscale and threshold to binary
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(
            gray, 127, 255, cv2.THRESH_BINARY_INV
        )

        height, width = thresh.shape
        sensor_y = height - 50
        zone_width = width // 5

        # Split bottom strip into 5 zones
        zones = [
            thresh[sensor_y:height,
                   i * zone_width:(i + 1) * zone_width]
            for i in range(5)
        ]

        # Weights for each zone center: -2, -1, 0, 1, 2
        weights = np.array([-2, -1, 0, 1, 2], dtype=float)

        # Detect if line present in each zone
        detections = np.array(
            [cv2.countNonZero(z) > 50 for z in zones],
            dtype=float
        )

        # Compute normalized error
        if detections.sum() > 0:
            error = float(np.dot(weights, detections) / detections.sum())
        else:
            error = 0.0

        # Build Twist message
        twist = Twist()
        twist.linear.x = self.base_speed
        twist.angular.z = -self.k_angular * error

        # Slow down when turning sharply
        max_weight = np.max(np.abs(weights))
        factor = max(0.2, 1 - abs(error) / max_weight)
        twist.linear.x *= factor

        # Visualization: draw zone rectangles
        vis = frame.copy()
        for idx, active in enumerate(detections):
            color = (0, 255, 0) if active else (0, 0, 255)
            x1 = idx * zone_width
            x2 = (idx + 1) * zone_width
            cv2.rectangle(vis, (x1, sensor_y), (x2, height), color, 2)

        # Overlay error text
        cv2.putText(
            vis,
            f'Err={error:.2f}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2
        )

        return twist, vis, error

    def timer_callback(self):
        """
        Timer callback at fixed rate to process the latest frame.

        Publishes velocity, error, and visualization image.
        """
        if not self.image_received or self.current_frame is None:
            return

        twist, vis_img, error = self.process_frame(self.current_frame)

        # Publish velocity command
        self.cmd_pub.publish(twist)

        # Publish line error
        err_msg = Float32()
        err_msg.data = error
        self.error_pub.publish(err_msg)

        # Publish processed image
        self.image_pub.publish(
            self.bridge.cv2_to_imgmsg(vis_img, 'bgr8')
        )

        # Reset flag until next image arrives
        self.image_received = False


def main(args=None):
    """
    Main entry point. Initializes ROS2 and spins the node.
    """
    rclpy.init(args=args)
    node = LineFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  
    main()

