""" This program publishes the radius and center of the detected ball   
    The radius will be zero if there is no detected object  
    published topics:  
        /processed_img [Image] 
    subscribed topics: 
        /camera    [Image]  
"""  
import rclpy 
from rclpy.node import Node 
import cv2 
import numpy as np 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from rcl_interfaces.msg import SetParametersResult
  
class CVExample(Node): 
    def __init__(self): 
        super().__init__('color_detector') 
 
        self.bridge = CvBridge() 

        self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10) 
        #self.sub = self.create_subscription(Image, 'camera', self.camera_callback, 10) 
        self.pub = self.create_publisher(Image, 'processed_img', 10) 

        # Parameters for the color detection, HSV color space
        self.declare_parameter('h_min', 40)
        self.declare_parameter('h_max', 80)
        self.declare_parameter('s_min', 0)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 0)
        self.declare_parameter('v_max', 255)

        self.h_min = self.get_parameter('h_min').value
        self.h_max = self.get_parameter('h_max').value
        self.s_min = self.get_parameter('s_min').value
        self.s_max = self.get_parameter('s_max').value
        self.v_min = self.get_parameter('v_min').value
        self.v_max = self.get_parameter('v_max').value

        self.lower_bound = np.array([self.h_min, self.s_min, self.v_min])
        self.upper_bound = np.array([self.h_max, self.s_max, self.v_max])

        self.add_on_set_parameters_callback(self.parameter_callback)
         
        self.image_received_flag = False #This flag is to ensure we received at least one image  
        dt = 0.5 
        self.timer = self.create_timer(dt, self.timer_callback) 
        self.get_logger().info('ros_color_tracker Node started') 

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'h_min':
                self.h_min = param.value
            elif param.name == 'h_max':
                self.h_max = param.value
            elif param.name == 's_min':
                self.s_min = param.value
            elif param.name == 's_max':
                self.s_max = param.value
            elif param.name == 'v_min':
                self.v_min = param.value
            elif param.name == 'v_max':
                self.v_max = param.value

        # Update the lower and upper bounds for color detection
        self.lower_bound = np.array([self.h_min, self.s_min, self.v_min])
        self.upper_bound = np.array([self.h_max, self.s_max, self.v_max])
        self.get_logger().info(f'Updated parameters: h_min={self.h_min}, h_max={self.h_max}, s_min={self.s_min}, s_max={self.s_max}, v_min={self.v_min}, v_max={self.v_max}')

        return SetParametersResult(successful=True)
  
    def camera_callback(self, msg): 
        try:  
            # We select bgr8 because its the OpenCV encoding by default  
            self.cv_img= self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            self.image_received_flag = True  
        except: 
            self.get_logger().info('Failed to get an image') 
  
  
    def timer_callback(self): 
        if self.image_received_flag: 
            self.image_received_flag=False 
            # Resize the image to 160x120 
            resized_image = cv2.resize(self.cv_img, (160,120)) #(width, height) 
            # Add some text to the image 
            cv2.putText(resized_image, "Holo", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2) 

            # Convert the image to HSV color space
            hsv_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)
            blurred_image = cv2.GaussianBlur(hsv_image, (5, 5), 0)
            mask = cv2.inRange(blurred_image, self.lower_bound, self.upper_bound)
            # Morphological operations to remove noise
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # Apply the mask to the original image
            result = cv2.bitwise_and(resized_image, resized_image, mask=mask)

            self.pub.publish(self.bridge.cv2_to_imgmsg(result,'bgr8')) 
 

      

def main(args=None): 
    rclpy.init(args=args) 
    cv_e = CVExample() 
    rclpy.spin(cv_e) 
    cv_e.destroy_node() 
    rclpy.shutdown() 
  

if __name__ == '__main__': 
    main() 