import rclpy 
from rclpy.node import Node 
import cv2 
import numpy as np 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from rclpy import qos
import signal
import sys
import time

class MoveLine(Node):
    
    def __init__(self):
        super().__init__('move_line_traffic')
        

        # Subsriptions
        self.declare_parameter('simulated_camera', False) # Set to True if using simulated robot
        self.simulated = self.get_parameter('simulated_camera').value
        if self.simulated:
            self.get_logger().info('Using simulated robot')
            self.sub = self.create_subscription(Image, 'camera', self.camera_callback, 10) # Simulated robot
        else:
            self.get_logger().info('Using real robot')
            self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10) # For Puzzlebot
        self.stop_light_sub = self.create_subscription(String, "traffic_light", self.stop_light_callback, 10)

        
        # Publishers
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_img = self.create_publisher(Image, 'processed_img_line', qos.qos_profile_sensor_data)
        self.linear_speed_pub = self.create_publisher(Float32, 'linear_speed', 10)
        self.angular_speed_pub = self.create_publisher(Float32, 'angular_speed', 10)

        # Timer
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # Global variables
        self.bridge = CvBridge()
        self.image_received_flag = False
        self.cmd_vel = Twist()
        self.state = 'stop' # Initial state
        self.traffic_light = 'green' # Initial traffic light state

        # PID controller variables (Using difference equation implemenation)
        #self.kp = 0.05
        #self.ki = 0.000
        #self.kd = 0.01
        # ------- Turning parameters -------
        self.kp = 1.0
        self.ki = 0.00
        self.kd = 0.001
        self.error = [0.0, 0.0, 0.0]  # e[0] actual error, e[1] last error, e[2] error before last
        self.u = [0.0 , 0.0] # u[0] actual output, u[1] last output
        self.K1 = self.kp + self.dt*self.ki + self.kd/self.dt
        self.K2 = -self.kp - 2.0*self.kd/self.dt
        self.K3 = self.kd/self.dt; 
        self.alpha = 0.8  # smoothing factor (0 < alpha < 1), lower = smoother
        self.filtered_error = 0.0
        self.prev_error = 0
        # ------- Straight parameters -------
        self.kp_straight = 0.9  # Proportional gain for straight control
        self.ki_straight = 0.0  # Integral gain for straight control
        self.kd_straight = 0.001  # Derivative gain for straight control
        self.error_straight = [0.0, 0.0, 0.0]  # e[0] actual error, e[1] last error, e[2] error before last
        self.u_straight = [0.0 , 0.0] # u[0] actual output, u[1] last output
        self.K1_straight = self.kp_straight + self.dt*self.ki_straight + self.kd_straight/self.dt
        self.K2_straight = -self.kp_straight - 2.0*self.kd_straight/self.dt
        self.K3_straight = self.kd_straight/self.dt
        self.alpha_straight = 0.8  # smoothing factor for straight control
        self.filtered_error_straight = 0.0
        self.prev_error_straight = 0
        

        self.original_img_width = 160
        self.original_img_height = 120
        self.cv_window_size = (self.original_img_width*3, self.original_img_height*3)


        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 1.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # Handle shutdown gracefully 
        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function 

        self.get_logger().info('MoveLine Node started')

    def control(self, center_x):
        # Get the center of the image
        height, width = self.cv_img.shape[:2]
        center_img = width // 2

        # Raw error calculation
        #raw_error = center_img - center_x

        max_expected_error = 80.0
        raw_error = (center_img - center_x)
        raw_error = np.clip(raw_error, -max_expected_error, max_expected_error)
        raw_error /= max_expected_error  # Now in [-1, 1]

        # Apply low-pass filter to the error
        self.filtered_error = self.alpha * raw_error + (1 - self.alpha) * self.filtered_error
        self.error[0] = self.filtered_error

        # discrete-time PID difference equation
        self.u[0] = self.K1 * self.error[0] + self.K2 * self.error[1] + self.K3 * self.error[2] + self.u[1]
        

        # Limit the angular speed
        max_angular_speed = self.angular_speed
        min_angular_speed = 0.0
        if abs(self.u[0]) > max_angular_speed:
            self.u[0] = max_angular_speed * np.sign(self.u[0])
        elif abs(self.u[0]) < min_angular_speed:
            self.u[0] = min_angular_speed * np.sign(self.u[0])

        # Shift values
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.u[1] = self.u[0]


        return self.linear_speed, self.u[0]  # Return linear speed and angular speed

    def control_straight(self, center_x):
        # Get the center of the image
        height, width = self.cv_img.shape[:2]
        center_img = width // 2

        # Raw error calculation
        raw_error = center_img - center_x
        max_expected_error = 80.0
        raw_error = np.clip(raw_error, -max_expected_error, max_expected_error)
        raw_error /= max_expected_error  # Now in [-1, 1]

        # Apply low-pass filter to the error
        self.filtered_error_straight = self.alpha_straight * raw_error + (1 - self.alpha_straight) * self.filtered_error_straight
        self.error_straight[0] = self.filtered_error_straight

        # discrete-time PID difference equation
        self.u_straight[0] = self.K1_straight * self.error_straight[0] + self.K2_straight * self.error_straight[1] + self.K3_straight * self.error_straight[2] + self.u_straight[1]
        # Limit the angular speed
        max_angular_speed = self.angular_speed
        min_angular_speed = 0.0
        if abs(self.u_straight[0]) > max_angular_speed:
            self.u_straight[0] = max_angular_speed * np.sign(self.u_straight[0])
        elif abs(self.u_straight[0]) < min_angular_speed:
            self.u_straight[0] = min_angular_speed * np.sign(self.u_straight[0])
        # Shift values
        self.error_straight[2] = self.error_straight[1]
        self.error_straight[1] = self.error_straight[0]
        self.u_straight[1] = self.u_straight[0]

        return 0.19, self.u_straight[0]  # Return linear speed and angular speed

    def stop_light_callback(self, msg):
        # Update traffic light state
        if self.traffic_light == msg.data:
            return
        self.traffic_light = msg.data

        # Check if the traffic light is red
        if self.traffic_light == 'red':
            self.state = 'red_light'
            self.get_logger().info("Red light detected, stopping robot")
        elif self.traffic_light == 'green' and self.state == 'red_light':
            self.state = 'green_light'
            self.get_logger().info("Green light detected, moving robot")
        elif self.traffic_light == 'yellow' and self.state != 'red_light':
            self.state = 'yellow_light' 
            self.get_logger().info("Yellow light detected, slowing down robot")


    def shutdown_function(self, signum, frame): 
        # Handle shutdown gracefully 
        # This function will be called when Ctrl+C is pressed 
        # It will stop the robot and shutdown the node 
        cv2.destroyAllWindows()
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
        # Main loop, state machine

        if self.state == 'stop':
            self.cmd_vel.linear.x = 0.0 # m/s 
            self.cmd_vel.angular.z = 0.0 # rad/s 
            self.pub.publish(self.cmd_vel) #publish the message 
            time.sleep(1)

            if self.image_received_flag:
                self.state = 'green_light' # Change state to green light
        
        elif self.state == 'green_light':
            if self.image_received_flag:
                # Process the image and get the line position
                proccessed_img, center, direction = self.process_image(self.cv_img) # Returns the processed image and the coordinaate of the center of the line
                
                # Calculate the control command
                if direction == "Left" or direction == "Right":
                    linear, angular = self.control(center[0])
                elif direction == "Straight":
                    linear, angular = self.control_straight(center[0])
                #linear, angular = self.control(center[0]) # Use the x coordinate of the center of the line

                self.cmd_vel.linear.x = linear
                self.cmd_vel.angular.z = angular

                # Publish the processed image
                #self.pub_img.publish(self.bridge.cv2_to_imgmsg(proccessed_img, 'bgr8'))
                proccessed_img = cv2.resize(proccessed_img, (800, 600))
                cv2.imshow("Proccessed img",proccessed_img)
                cv2.waitKey(1)
                # Publish the control command
                self.pub.publish(self.cmd_vel)

        elif self.state == 'yellow_light':
            if self.image_received_flag:
                # Process the image and get the line position
                proccessed_img, center, direction = self.process_image(self.cv_img) # Returns the processed image and the coordinaate of the center of the line
                
                # Calculate the control command
                linear, angular = self.control(center[0])

                self.cmd_vel.linear.x = linear - 0.05  # Slow down
                self.cmd_vel.angular.z = angular

                # Publish the processed image
                #self.pub_img.publish(self.bridge.cv2_to_imgmsg(proccessed_img, 'bgr8'))
                proccessed_img = cv2.resize(proccessed_img, (800, 600))
                cv2.imshow("Proccessed img",proccessed_img)
                cv2.waitKey(1)
                # Publish the control command
                self.pub.publish(self.cmd_vel)

        elif self.state == 'red_light':
            self.cmd_vel.linear.x = 0.0 # m/s 
            self.cmd_vel.angular.z = 0.0 # rad/s 
            self.pub.publish(self.cmd_vel) #publish the message 

        # Publish the linear and angular speeds
        self.linear_speed_pub.publish(Float32(data=self.cmd_vel.linear.x))
        self.angular_speed_pub.publish(Float32(data=self.cmd_vel.angular.z))

    
    def process_image(self, img):
        # Get x and y coordinates of the center of the line in the lower part of the image
        x,y,w,h = self.detect_lower_bound(img)
        # Get x and y coordinates of the center of the line in the upper part of the image
        x2,y2,w2,h2 = self.detect_upper_bound(img, x)

        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        center_lower = (x + w // 2, y + h // 2)
        cv2.circle(img, center_lower, 2, (0, 255, 0), -1)

        cv2.rectangle(img, (x2, y2), (x2 + w2, y2 + h2), (0, 255, 0), 2)
        center_upper = (x2 + w2 // 2, y2 + h2 // 2)
        cv2.circle(img, center_upper, 2, (0, 255, 0), -1)

        # Draw a line between the two centers
        cv2.line(img, center_lower, center_upper, (0, 255, 255), 1)

        # Draw lines delimiting the upper and lower bounds
        cv2.line(img, (0, center_upper[1]), (img.shape[1], center_upper[1]), (255, 0, 0), 1)
        cv2.line(img, (0, center_lower[1]), (img.shape[1], center_lower[1]), (255, 0, 0), 1)

        # Calculate the angle between the two centers
        angle = np.arctan2(center_upper[1] - center_lower[1], center_upper[0] - center_lower[0])
        angle = np.degrees(angle) + 90.0  # Convert to degrees

        direction = "Straight"  # Default direction
        if angle > 10:
            cv2.putText(img, "Turn Right", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            direction = "Right"
        elif angle < -10:
            cv2.putText(img, "Turn Left", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            direction = "Left"
        elif (-10 <= angle <= 10) and (img.shape[1] // 2 - 20 <= center_lower[0] <= img.shape[1] // 2 + 20):
            cv2.putText(img, "Go Straight", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            direction = "Straight"
        else:
            cv2.putText(img, "------", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)



        cv2.putText(img, f"Angle: {angle:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        

        return img, center_lower, direction
    
    def detect_upper_bound(self, original_img, x):
        grayimg = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)
        # Crop the image to get the upper part of the image
        height, width = grayimg.shape[:2]
        # Get top bound 
        start_row = int(height * (5/10))  
        end_row = int(height * (6/10))
        # Get horizontally the whole image
        start_col = np.clip(x - 20, 0, width)
        end_col = np.clip(x + 35, 0, width)
        grayimg = grayimg[start_row:end_row, start_col:end_col]

        cv2.imshow("Upper Bound", grayimg)

        # Apply Gaussian blur to the image
        blurred = cv2.GaussianBlur(grayimg, (5, 5), 0)
        # Apply a threshold
        _, thresh = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)

        # Apply morphological operations to remove noise
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=2)

        cv2.imshow("Thresholded Upper Bound", thresh)

        # Find contours in the thresholded image
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea) 
            x, y, w, h = cv2.boundingRect(c)
            y += start_row
            # Adjust x coordinate to match original image
            x += start_col
            #cv2.rectangle(original_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            #center = (x + w // 2, y + h // 2)
            #cv2.circle(original_img, center, 2, (0, 255, 0), -1)
        else:
            if x > width // 2:
                x,y,w,h = (width, height // 2, 0, 0)
            elif x <= width // 2:
                x,y,w,h = (0, height // 2, 0, 0)


        return x,y,w,h

    
    def detect_lower_bound(self, original_img):
        grayimg = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)
        # Crop the image to get the bottom part of the image
        height, width = grayimg.shape[:2]
        # Get bottom 1/3 vertically
        start_row = int(height * (4/5))  # lower third
        end_row = height
        # Get horizontally the whole image
        start_col = 0
        end_col = width
        grayimg = grayimg[start_row:end_row, start_col:end_col]

        cv2.imshow("Lower Bound", grayimg)

        # Apply Gaussian blur to the image
        blurred = cv2.GaussianBlur(grayimg, (5, 5), 0)

        # Apply a threshold 
        _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)

        # Apply morphological operations to remove noise
        thresh = cv2.erode(thresh, None, iterations=2)  
        thresh = cv2.dilate(thresh, None, iterations=2) 

        cv2.imshow("Thresholded Lower Bound", thresh)
        
        # Find contours in the thresholded image
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea) 
            x, y, w, h = cv2.boundingRect(c)
            y += start_row # Adjust y coordinate to match original image
            #cv2.rectangle(original_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            #center = (x + w // 2, y + h // 2)
            #cv2.circle(original_img, center, 2, (0, 255, 0), -1)
        else:
            x,y,w,h = (width // 2, height // 2, 0, 0)

        return x, y, w, h
    
def main(args=None):
    rclpy.init(args=args)
    move_line = MoveLine()

    try:
        rclpy.spin(move_line)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        move_line.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
