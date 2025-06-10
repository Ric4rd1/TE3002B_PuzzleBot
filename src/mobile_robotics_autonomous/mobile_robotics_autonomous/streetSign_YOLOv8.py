import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
from rclpy.qos import qos_profile_sensor_data


class YoloInference(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.model = YOLO('/home/ricard/ros2_ws_puzzlebot/src/mobile_robotics_autonomous/models/SignsV2.pt')  # Load the YOLOv8 model
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10) # For Puzzlebot
        self.pub = self.create_publisher(String, 'traffic_signs', 10) 
        self.pub2 = self.create_publisher(String, 'traffic_signs2', 10) 
        self.yolo_img_pub =  self.create_publisher(Image, 'processed_img_signs', qos_profile=qos_profile_sensor_data) 

        self.img = None
        timer_period = 0.08
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.valid_img = True
        except:
            self.get_logger().info('Failed to get an image')

    def timer_callback(self):
        if self.img is None:
            return

        results = self.model(self.img)

        # Annotate image
        frame = results[0].plot()

        # Publish annotated image
        #self.yolo_img_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

        # Publish class name
        for r in results:
            for box in r.boxes:
                class_id = int(box.cls)
                class_name = self.model.names[class_id]
                msg = String()
                msg.data = class_name

                if class_name == 'stop' and box.conf < 0.85:
                    #self.get_logger().info(f'Low confidence for class {class_name}: {box.conf}')
                    continue
                elif class_name in ["give_way", "construction"] and box.conf < 0.7:
                    continue
                elif box.conf < 0.5:
                    #self.get_logger().info(f'Low confidence for class {class_name}: {box.conf}')
                    continue

                if class_name in ["construction", "give_way","stop"]:
                    self.pub2.publish(String(data=class_name))
                
                self.pub.publish(msg)
                break  # Only publish one class (first detection)




def main(args=None):
    rclpy.init(args=args)
    y_i = YoloInference()
    rclpy.spin(y_i)
    y_i.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()