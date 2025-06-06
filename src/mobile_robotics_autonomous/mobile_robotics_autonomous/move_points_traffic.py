
import rclpy 
from rclpy.node import Node 
import rclpy.duration
from rclpy import qos
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String, Bool
import numpy as np
import signal
import sys
import time
 
# Move point to point, 4 points total (closed loop) points are given topic /point

class MovePoints(Node): 
    def __init__(self): 
        super().__init__('move_points_traffic')

        # Handle shutdown gracefully 
        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function 

        # Parameters
        self.declare_parameter('linear_vel', 0.19) # (m/s)
        self.declare_parameter('angular_vel', 0.8) # (rad/s)

        # Publishers
        # Publisher for sending velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10) 
        # Publisher for sending confirmation to get next point
        self.confirmation_pub = self.create_publisher(String, "confirmation", 10)
        # Publisher for activating odom node
        self.odom_switch_pub = self.create_publisher(Bool, "odom_switch", 10)

        # Subscribers
        # Subscriber for receiving actual pose of the robot
        self.pose2D_sub = self.create_subscription(Pose2D, "pose", self.pose2D_callback, qos.qos_profile_sensor_data)
        # Subscriber for starting rutine
        self.intersection_detection_sub = self.create_subscription(String, "intersection_detection", self.intersection_detection_callback, 10) 
        # Subscriber for receiving traffic signs
        self.traffic_signs_sub = self.create_subscription(String, "traffic_signs", self.traffic_signs_callback, 10)
        # Subscriber for detecting stop light
        self.stop_light_sub = self.create_subscription(String, "traffic_light", self.stop_light_callback, 10)

        # Timer
        timer_period = 0.05 #Time in seconds to call the timer_callback function 
        self.timer = self.create_timer(timer_period, self.timer_callback) 

        # Variables
        self.state = 'wait' # Initial state
        self.start = False
        self.traffic_light = 'red' # Initial traffic light state
        self.traffic_sign = 'right'
        self.recieved_initial_pose = False # Flag to check if the initial pose is received
        # Constants
        self.vel_linear = self.get_parameter('linear_vel').value # (rad/s)
        self.vel_angular = self.get_parameter('angular_vel').value # (rad/s)
        # Control
        self.kp_linear = 0.9
        self.kp_angular = 0.9
        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.setpoint = [0.0, 0.0]

        # Message
        self.cmd_vel = Twist() 

        self.get_logger().info("Node initialized!!!") 

    def shutdown_function(self, signum, frame): 
        # Handle shutdown gracefully 
        # This function will be called when Ctrl+C is pressed 
        # It will stop the robot and shutdown the node 
        self.get_logger().info("Shutting down. Stopping robot...") 
        stop_twist = Twist()  # All zeros to stop the robot 
        self.cmd_vel_pub.publish(stop_twist) # publish it to stop the robot before shutting down 
        rclpy.shutdown() # Shutdown the node 
        sys.exit(0) # Exit the program 

    def stop_light_callback(self, msg):
        # Update traffic light state
        if self.traffic_light == msg.data:
            return
        self.traffic_light = msg.data

    def intersection_detection_callback(self, msg):
        if msg.data == "intersection":
            self.get_logger().info("Starting intersection manuver")
            self.start = True

    def pose2D_callback(self, msg):
        if not self.recieved_initial_pose:
            self.recieved_initial_pose = True
        # Update current position
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        #self.get_logger().info(f'Values recieved: {self.x}x, {self.y}y, {self.yaw}rad\n')

    def traffic_signs_callback(self, msg):
        if msg.data in ["straight", "left", "right"]:
            self.traffic_sign = msg.data
        elif msg.data == "stop":
            pass
        
    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    
    def pControl(self, x, y, theta):
        # Calculate errors
        angle2goal = np.arctan2(self.setpoint[1] - y, self.setpoint[0] - x)
        self.yaw_error = self.normalize_angle(angle2goal - theta)
        self.distance_error = np.sqrt((self.setpoint[0] - x)**2 + (self.setpoint[1] - y)**2)

        # Control signals
        control_ang = self.kp_angular * self.yaw_error
        control_lin = self.kp_linear * self.distance_error
        
        # Limit velocities
        min_vel_lin = 0.1
        max_vel_lin = self.vel_linear
        if abs(control_lin) > max_vel_lin:
            control_lin = np.sign(control_lin) * max_vel_lin
        elif abs(control_lin) < min_vel_lin:
            control_lin = np.sign(control_lin) * min_vel_lin

        min_vel_ang = 0.1
        max_vel_ang = self.vel_angular
        if abs(control_ang) > max_vel_ang:
            control_ang = np.sign(control_ang) * max_vel_ang
        elif abs(control_ang) < min_vel_ang:
            control_ang = np.sign(control_ang) * min_vel_ang

        return control_lin, control_ang
    
    def pControl_ang(self, x, y, theta):
        angle2goal = np.arctan2(self.setpoint[1] - y, self.setpoint[0] - x)
        self.yaw_error = self.normalize_angle(angle2goal - theta)
        control_ang = self.kp_angular * self.yaw_error
        min_vel = 0.35
        max_vel = self.vel_angular
        if abs(control_ang) > max_vel:
            control_ang = np.sign(control_ang) * max_vel
        elif abs(control_ang) < min_vel:
            control_ang = np.sign(control_ang) * min_vel
        return control_ang
    

    def timer_callback(self): 
        # Main loop, state machine

        # Initial state
        if self.state == "wait": 
            # Wait for initial localization (odometry) pose
            if self.start:
                self.odom_switch_pub.publish(Bool(data=True)) # Turn odometry on
                self.state = "correct"
                self.start = False
                self.setpoint = [0.1, 0.0]
                self.get_logger().info("Correcting...")

        # Move to the edge of the intersection and start to listen to traffic lights and signs topics
        elif self.state == "correct":
            if self.recieved_initial_pose:
                lin_vel, ang_vel = self.pControl(self.x, self.y, self.theta)
                # Publish control signals
                self.cmd_vel.linear.x = lin_vel
                self.cmd_vel.angular.z = ang_vel
                self.cmd_vel_pub.publish(self.cmd_vel)

                if self.distance_error < 0.02 and abs(self.yaw_error) < 0.1: #Check if the robot is close to the setpoint
                    # Log current position
                    self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.theta}rad\n')
                    # Stop the robot
                    stop=Twist()
                    self.cmd_vel_pub.publish(stop)
                    self.state = "evaluate"
                    self.get_logger().info("Evaluating...")

        # Evaluate the conditions (traffic light and direction) 
        elif self.state == "evaluate":

            if self.traffic_light == "green":
                if self.traffic_sign == "straight":
                    self.setpoint = [0.6, 0]
                    self.state = "move"
                    self.get_logger().info("Going straight...")
                elif self.traffic_sign == "left":
                    self.setpoint = [0.3, 0.35]
                    self.state = "move"
                    self.get_logger().info("Going to the left...")
                elif self.traffic_sign == "right":
                    self.setpoint = [0.3, -0.35]
                    self.state = "move"
                    self.get_logger().info("Going to the right...")

            elif self.traffic_light in ["red", "yellow"]:
                pass

        # Move to the point setpoint
        elif self.state == "move": 
            lin_vel, ang_vel = self.pControl(self.x, self.y, self.theta)
            # Publish control signals
            self.cmd_vel.linear.x = lin_vel
            self.cmd_vel.angular.z = ang_vel
            self.cmd_vel_pub.publish(self.cmd_vel)

            # Log current position
            #self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.yaw}rad\n')

            # Check if the robot is close to the setpoint
            if self.distance_error < 0.15: #Check if the robot is close to the setpoint
                # Log current position
                self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.theta}rad\n')
                # Stop the robot
                stop=Twist()
                self.cmd_vel_pub.publish(stop)
                self.get_logger().info("Moving finished")
                # Send confirmation message and wait for next point
                self.state = "wait" 
                self.recieved_initial_pose = False
                self.confirmation_pub.publish(String(data='OK')) # Publish confirmation message
                self.get_logger().info("Confirmation message sent")
            
        
                
def main(args=None): 
    rclpy.init(args=args) 
    move_points = MovePoints() 
    try:
        rclpy.spin(move_points)
    except KeyboardInterrupt:
        pass
    finally:
        move_points.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__': 
    main() 