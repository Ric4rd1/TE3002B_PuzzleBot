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
        self.stop_light_sub = self.create_subscription(String, "traffic_light", self.stop_light_callback, 10)
        self.x_center_sub = self.create_subscription(Float32, 'x_center', self.x_center_callback, qos_profile=qos.qos_profile_sensor_data) 
        self.line_direction_sub = self.create_subscription(String, 'line_direction', self.line_direction_callback, qos_profile=qos.qos_profile_sensor_data)

        
        # Publishers
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed_pub = self.create_publisher(Float32, 'linear_speed', 10)
        self.angular_speed_pub = self.create_publisher(Float32, 'angular_speed', 10)

        # Timer
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # Global variables
        self.x_center_recieved_flag = False
        self.x_center = 0.0
        self.line_direction = "Straight" 
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

        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 1.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # Handle shutdown gracefully 
        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function 

        self.get_logger().info('MoveLine Node started')

    def x_center_callback(self, msg):
        # Update the x center value
        self.x_center = msg.data
        self.x_center_recieved_flag = True

    def line_direction_callback(self, msg):
        # Update the line direction
        self.line_direction = msg.data

    def control(self, center_x):
        # Get the center of the image
        center_img = self.original_img_width // 2

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
        center_img = self.original_img_width // 2

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
        self.get_logger().info("Shutting down. Stopping robot...") 
        stop_twist = Twist()  # All zeros to stop the robot 
        self.pub.publish(stop_twist) # publish it to stop the robot before shutting down 
        rclpy.shutdown() # Shutdown the node 
        sys.exit(0) # Exit the program 

    def timer_callback(self):
        # Main loop, state machine

        if self.state == 'stop':
            self.cmd_vel.linear.x = 0.0 # m/s 
            self.cmd_vel.angular.z = 0.0 # rad/s 
            self.pub.publish(self.cmd_vel) #publish the message 
            time.sleep(1)

            if self.x_center_recieved_flag:
                self.state = 'green_light' # Change state to green light
        
        elif self.state == 'green_light':
            if self.x_center_recieved_flag:
                
                # Calculate the control command
                if self.line_direction == "Left" or self.line_direction == "Right":
                    linear, angular = self.control(self.x_center)
                elif self.line_direction == "Straight":
                    linear, angular = self.control_straight(self.x_center)
                #linear, angular = self.control(center[0]) # Use the x coordinate of the center of the line

                self.cmd_vel.linear.x = linear
                self.cmd_vel.angular.z = angular

                # Publish the control command
                self.pub.publish(self.cmd_vel)

        elif self.state == 'yellow_light':
            if self.x_center_recieved_flag:
                
                # Calculate the control command
                linear, angular = self.control(self.x_center)

                self.cmd_vel.linear.x = linear - 0.05  # Slow down
                self.cmd_vel.angular.z = angular
                
                # Publish the control command
                self.pub.publish(self.cmd_vel)

        elif self.state == 'red_light':
            self.cmd_vel.linear.x = 0.0 # m/s 
            self.cmd_vel.angular.z = 0.0 # rad/s 
            self.pub.publish(self.cmd_vel) #publish the message 

        # Publish the linear and angular speeds
        self.linear_speed_pub.publish(Float32(data=self.cmd_vel.linear.x))
        self.angular_speed_pub.publish(Float32(data=self.cmd_vel.angular.z))

    
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
