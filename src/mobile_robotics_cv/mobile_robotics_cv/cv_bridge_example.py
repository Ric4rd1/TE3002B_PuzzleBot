import rclpy 
from rclpy.node import Node 
import cv2 
import numpy as np 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
  
class CVExample(Node): 
    def __init__(self): 
        super().__init__('ros_color_tracker') 
 
        self.bridge = CvBridge() 
  
        
        # self.sub = self.create_subscription(Image, 'robot/camera1/image_raw', self.camera_callback, 10) # Real robot
        self.sub = self.create_subscription(Image, '/camera', self.camera_callback, 10) # Simulated robot
        self.pub = self.create_publisher(Image, 'processed_img', 10) 
         
        self.image_received_flag = False #This flag is to ensure we received at least one image  
        dt = 0.1 
        self.timer = self.create_timer(dt, self.timer_callback) 
        self.get_logger().info('ros_color_tracker Node started') 
  

    def camera_callback(self, msg): 
        try:  
            # We select bgr8 because its the OpenCV encoding by default  
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            # Add a text to the image
            cv2.putText(self.cv_img, 'Hello', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            self.image_received_flag = True  
        except: 
            self.get_logger().info('Failed to get an image') 

  

    def timer_callback(self): 
        if self.image_received_flag: 
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.cv_img,'bgr8')) 
      

def main(args=None): 
    rclpy.init(args=args) 
    cv_e = CVExample() 
    rclpy.spin(cv_e) 
    cv_e.destroy_node() 
    rclpy.shutdown() 
  

if __name__ == '__main__': 
    main()