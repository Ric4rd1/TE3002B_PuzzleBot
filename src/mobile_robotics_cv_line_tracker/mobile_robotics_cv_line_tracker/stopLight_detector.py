import rclpy 
from rclpy.node import Node 
import cv2 
import numpy as np 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
  
class TrafficDetector(Node): 
    def __init__(self): 
        super().__init__('stopLight_detector') 
 
        self.bridge = CvBridge() 

        # Subscriptions
        self.declare_parameter('simulated_camera', True) # Set to True if using simulated robot
        self.simulated = self.get_parameter('simulated_camera').value
        if self.simulated:
            self.get_logger().info('Using simulated robot')
            self.sub = self.create_subscription(Image, 'camera', self.camera_callback, 10) # Simulated robot
        else:
            self.get_logger().info('Using real robot')
            self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10) # For Puzzlebot
        
        #self.sub = self.create_subscription(Image, 'image_raw', self.camera_callback, 10) # Webcam
        #self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10) # For Puzzlebot
        #self.sub = self.create_subscription(Image, 'camera', self.camera_callback, 10) # Simulated Puzzlebot

        # Publishers
        self.pub = self.create_publisher(Image, 'processed_img_traffic', qos_profile=qos_profile_sensor_data) 
        self.trafic_light_pub = self.create_publisher(String, 'traffic_light', 10)

        # Parameters for the color detection, HSV color space
        # First 3 are lower bounds, last 3 are upper bounds
        self.declare_parameter('Red_HSV', [0, 133, 105, 34, 255, 255]) 
        self.declare_parameter('Green_HSV', [35, 133, 105, 80, 255, 255])
        self.declare_parameter('Yellow_HSV', [22, 133, 105, 52, 255, 255])

        red_hsv = self.get_parameter('Red_HSV').value
        green_hsv = self.get_parameter('Green_HSV').value
        yellow_hsv = self.get_parameter('Yellow_HSV').value

        self.red_lower_bound = np.array(red_hsv[:3])
        self.red_upper_bound = np.array(red_hsv[3:])
        self.green_lower_bound = np.array(green_hsv[:3])
        self.green_upper_bound = np.array(green_hsv[3:])
        self.yellow_lower_bound = np.array(yellow_hsv[:3])
        self.yellow_upper_bound = np.array(yellow_hsv[3:])

        self.get_logger().info(f'Initial parameters: Red_HSV={red_hsv}, Green_HSV={green_hsv}, Yellow_HSV={yellow_hsv}')

        self.add_on_set_parameters_callback(self.parameter_callback)

        # This size is preconfigured on the Puzzlebot
        # Under /home/puzzlebot/ros2_packages_ws/src/ros_deep_learning/launch/video_source.ros2.launch
        self.width = 160
        self.height = 120
        self.area_min = (self.width * self.height) * 0.55 # 0.5 for simulation, 0.015 for real robot

         
        self.image_received_flag = False #This flag is to ensure we received at least one image  
        dt = 0.05 # 20Hz
        self.timer = self.create_timer(dt, self.timer_callback) 
        self.get_logger().info('Node started!!!') 

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
            #resized_image = cv2.resize(self.cv_img, (160,120)) #(width, height) 
            # Add some text to the image 
            #cv2.putText(resized_image, "Holo", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2) 

            # Convert the image to HSV color space
            hsv_image = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2HSV)
            # Apply Gaussian blur to the image
            blurred_image = cv2.GaussianBlur(hsv_image, (5, 5), 0)
            # Create a mask for each color
            red_mask = cv2.inRange(blurred_image, self.red_lower_bound, self.red_upper_bound)
            green_mask = cv2.inRange(blurred_image, self.green_lower_bound, self.green_upper_bound)
            yellow_mask = cv2.inRange(blurred_image, self.yellow_lower_bound, self.yellow_upper_bound)
            # Apply morphological operations to remove noise
            for mask in [red_mask, green_mask, yellow_mask]:
                mask = cv2.erode(mask, None, iterations=4)
                mask = cv2.dilate(mask, None, iterations=4)
            # Combine the masks
            combined_mask = cv2.bitwise_or(red_mask, green_mask)
            combined_mask = cv2.bitwise_or(combined_mask, yellow_mask)
            
            result = cv2.bitwise_and(self.cv_img, self.cv_img, mask=combined_mask)

            # Find contours in each color mask
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            contours = [red_contours, green_contours, yellow_contours]
            
            for color, cnts in zip(['red', 'green', 'yellow'], contours):
                for cnt in cnts:
                    area = cv2.contourArea(cnt)
                    if area > self.area_min:
                        # Draw the bounding box around the detected light
                        x, y, w, h = cv2.boundingRect(cnt)
                        cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        center = (x + w // 2, y + h // 2)
                        cv2.circle(result, center, 5, (0, 255, 0), -1)
                        cv2.putText(result, f'{color} light', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        # Publish the detected color
                        self.trafic_light_pub.publish(String(data=color))

            self.pub.publish(self.bridge.cv2_to_imgmsg(result,'bgr8')) 
 

      

def main(args=None): 
    rclpy.init(args=args) 
    cv_e = TrafficDetector() 
    rclpy.spin(cv_e) 
    cv_e.destroy_node() 
    rclpy.shutdown() 
  

if __name__ == '__main__': 
    main() 