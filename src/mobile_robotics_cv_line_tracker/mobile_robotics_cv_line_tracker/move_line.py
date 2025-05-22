import rclpy 
from rclpy.node import Node 
import cv2 
import numpy as np 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
from rclpy import qos
import signal
import sys

class MoveLine(Node):
    
    def __init__(self):
        super().__init__('move_line')
        

        # Subsriptions
        #self.sub = self.create_subscription(Image, 'camera', self.camera_callback, 10) # Simulated robot
        self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10) # Real robot
        
        # Publishers
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_img = self.create_publisher(Image, 'processed_img_line', qos.qos_profile_sensor_data)

        # Timer
        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)

        # Global variables
        self.bridge = CvBridge()
        self.image_received_flag = False
        self.cmd_vel = Twist()

        # Handle shutdown gracefully 
        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function 

        self.get_logger().info('MoveLine Node started')

    def shutdown_function(self, signum, frame): 
        # Handle shutdown gracefully 
        # This function will be called when Ctrl+C is pressed 
        # It will stop the robot and shutdown the node 
        self.get_logger().info("Shutting down. Stopping robot...") 
        stop_twist = Twist()  # All zeros to stop the robot 
        self.pub.publish(stop_twist) # publish it to stop the robot before shutting down 
        rclpy.shutdown() # Shutdown the node 
        sys.exit(0) # Exit the program 

    def camera_callback(self, msg): 
        try:  
            # We select bgr8 because its the OpenCV encoding by default  
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            self.image_received_flag = True  
        except: 
            self.get_logger().info('Failed to get an image')

    def timer_callback(self):
        if self.image_received_flag:
            # Process the image and get the line position
            proccessed_img, center = self.process_image(self.cv_img) # Returns the processed image and the coordinaate of the center of the line
            
            # Calculate the control command
            linear, angular = self.control(center[0])

            self.cmd_vel.linear.x = linear
            self.cmd_vel.angular.z = angular

            # Publish the processed image
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(proccessed_img, 'bgr8'))
            # Publish the control command
            self.pub.publish(self.cmd_vel)

    def control(self, center_x):
        # Get the center of the image
        height, width = self.cv_img.shape[:2]
        center_img = width // 2

        # Calculate the error
        error = center_img - center_x

        # Proportional control
        k_p = 0.013
        linear_speed = 0.15
        angular_speed = k_p * error

        # Limit the angular speed
        max_angular_speed = 0.8
        min_angular_speed = 0.2
        if abs(angular_speed) > max_angular_speed:
            angular_speed = max_angular_speed * np.sign(angular_speed)
        elif abs(angular_speed) < min_angular_speed:
            angular_speed = min_angular_speed * np.sign(angular_speed)

        return linear_speed, angular_speed

    def process_image(self, img):
        # Convert the image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Crop the image to get the bottom part of the image
        height, width = gray.shape[:2]
        # Get bottom 1/3 vertically
        start_row = int(height * (2/3))  # lower third
        end_row = height
        # Get right 1/2 horizontally
        start_col = 0
        end_col = width
        gray = gray[start_row:end_row, start_col:end_col]

        # Apply Gaussian blur to the image
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply a threshold 
        _, thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV)

        # Apply morphological operations to remove noise
        thresh = cv2.erode(thresh, None, iterations=2)  
        thresh = cv2.dilate(thresh, None, iterations=2) 
        
        # Find contours in the thresholded image
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea) 
            x, y, w, h = cv2.boundingRect(c)
            y += start_row # Adjust y coordinate to match original image
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            center = (x + w // 2, y + h // 2)
            cv2.circle(img, center, 5, (0, 255, 0), -1)
        else:
            center = (width // 2, height // 2)

        return img, center
    
def main(args=None):
    rclpy.init(args=args)
    move_line = MoveLine()

    try:
        rclpy.spin(move_line)
    except KeyboardInterrupt:
        pass
    finally:
        move_line.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
