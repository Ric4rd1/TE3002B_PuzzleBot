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
        self.latest_frame = None
        self.frozen_frame = None  # Imagen congelada

        for i in range(1, 4):
            window_name = f"HSV Controls {i}"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 400, 300)
            cv2.createTrackbar(f"H Min {i}", window_name, 0, 179, lambda x: None)
            cv2.createTrackbar(f"H Max {i}", window_name, 179, 179, lambda x: None)
            cv2.createTrackbar(f"S Min {i}", window_name, 0, 255, lambda x: None)
            cv2.createTrackbar(f"S Max {i}", window_name, 255, 255, lambda x: None)
            cv2.createTrackbar(f"V Min {i}", window_name, 0, 255, lambda x: None)
            cv2.createTrackbar(f"V Max {i}", window_name, 255, 255, lambda x: None)

        self.timer = self.create_timer(0.05, self.update_display)
        self.get_logger().info("HSV Tuner with 3 masks and capture mode started!")

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

    def get_hsv_bounds(self, mask_id):
        window = f"HSV Controls {mask_id}"
        h_min = cv2.getTrackbarPos(f"H Min {mask_id}", window)
        h_max = cv2.getTrackbarPos(f"H Max {mask_id}", window)
        s_min = cv2.getTrackbarPos(f"S Min {mask_id}", window)
        s_max = cv2.getTrackbarPos(f"S Max {mask_id}", window)
        v_min = cv2.getTrackbarPos(f"V Min {mask_id}", window)
        v_max = cv2.getTrackbarPos(f"V Max {mask_id}", window)
        return np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max])

    def update_display(self):
        # Usamos la imagen congelada si existe
        if self.frozen_frame is None:
            if self.latest_frame is None:
                return
            frame = cv2.resize(self.latest_frame, (800, 600))
        else:
            frame = self.frozen_frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv[...,2] = cv2.equalizeHist(hsv[...,2])
        frame_eq = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        cl = clahe.apply(l)
        limg = cv2.merge((cl,a,b))
        frame_clahe = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

        cv2.imshow("Original", frame)
        cv2.imshow("Equalized", frame_eq)
        cv2.imshow("CLAHE", frame_clahe)

        for i in range(1, 4):
            lower, upper = self.get_hsv_bounds(i)
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(frame, frame, mask=mask)
            cv2.imshow(f"Mask {i}", mask)
            cv2.imshow(f"Filtered {i}", result)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.destroy_node()
            rclpy.shutdown()
        elif key == ord('c'):
            if self.latest_frame is not None:
                self.frozen_frame = cv2.resize(self.latest_frame, (800, 600))
                self.get_logger().info("Captured new frame for tuning.")

def main(args=None):
    rclpy.init(args=args)
    node = HSVTuner()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
