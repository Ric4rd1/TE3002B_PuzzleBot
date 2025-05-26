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
        self.declare_parameter('simulated_camera', False) # Set to True if using simulated robot
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
        self.declare_parameter('Red_HSV', [0, 36, 138, 20, 255, 255]) 
        self.declare_parameter('Green_HSV', [60, 55, 104, 81, 255, 255])
        self.declare_parameter('Yellow_HSV', [22, 96, 71, 31, 255, 255])

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
        dt = 0.2 
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
            self.image_received_flag = False 

            # Convert image to HSV and equalize V channel
            hsv_image = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2HSV)
            hsv_image[..., 2] = cv2.equalizeHist(hsv_image[..., 2])

            # Gaussian blur
            blurred_image = cv2.GaussianBlur(hsv_image, (5, 5), 0)

            # Create and clean masks
            red_mask = cv2.inRange(blurred_image, self.red_lower_bound, self.red_upper_bound)
            green_mask = cv2.inRange(blurred_image, self.green_lower_bound, self.green_upper_bound)
            yellow_mask = cv2.inRange(blurred_image, self.yellow_lower_bound, self.yellow_upper_bound)

            for mask in [red_mask, green_mask, yellow_mask]:
                mask[:] = cv2.erode(mask, None, iterations=2)
                mask[:] = cv2.dilate(mask, None, iterations=2)

            masks = {'red': red_mask, 'green': green_mask, 'yellow': yellow_mask}
            result = self.cv_img.copy()

            # Show masks
            #cv2.imshow()

            for color, mask in masks.items():
                # Detect circles
                circles = cv2.HoughCircles(
                    mask,
                    cv2.HOUGH_GRADIENT,
                    dp=1.2,
                    minDist=5,
                    param1=50,
                    param2=10,
                    minRadius=3,
                    maxRadius=20
                )

                # If circles are found, pick the largest one
                if circles is not None:
                    circles = np.uint16(np.around(circles))
                    largest_circle = max(circles[0, :], key=lambda c: c[2])  # c[2] is radius
                    x, y, r = largest_circle
                    if r > 0:
                        cv2.circle(result, (x, y), r, (0, 255, 0), 2)
                        cv2.circle(result, (x, y), 2, (0, 0, 255), 3)
                        cv2.putText(result, f'{color} circle', (x - 10, y - r - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        self.trafic_light_pub.publish(String(data=color))

            #self.pub.publish(self.bridge.cv2_to_imgmsg(result, 'bgr8'))

      

def main(args=None): 
    rclpy.init(args=args) 
    cv_e = TrafficDetector() 
    rclpy.spin(cv_e) 
    cv_e.destroy_node() 
    rclpy.shutdown() 
  

if __name__ == '__main__': 
    main() 