
import rclpy 
from rclpy.node import Node 
import rclpy.duration
from rclpy import qos
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String
import numpy as np
import signal
import sys
 
# Move point to point, 4 points total (closed loop) points are given topic /point

class MovePoints(Node): 
    def __init__(self): 
        super().__init__('move_points')

        # Wait for localization and point generator to be ready
        sleep_duration = rclpy.duration.Duration(seconds=2.0)
        self.get_clock().sleep_for(sleep_duration) 

        # Handle shutdown gracefully 
        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function 

        # Parameters
        self.declare_parameter('linear_vel', 0.2) # (m/s)
        self.declare_parameter('angular_vel', 0.6) # (rad/s)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10) 
        self.confirmation_pub = self.create_publisher(String, "confirmation", 10)

        # Subscribers
        self.pose2D_sub = self.create_subscription(Pose2D, "pose2D", self.pose2D_callback, qos.qos_profile_sensor_data)
        self.point_sub = self.create_subscription(Pose2D, "point", self.point_callback, 10)

        # Timer
        timer_period = 0.05 #Time in seconds to call the timer_callback function 
        self.timer = self.create_timer(timer_period, self.timer_callback) 

        # Variables
        self.state = 'stop' # Initial state
        self.first_time = True # flag
        self.recieved_initial_pose = False # flag
        self.recieved_point = False # flag
        self.send_confirmation = False # flag
        self.counter = 0 
        # Constants
        self.vel_linear = self.get_parameter('linear_vel').value # (m/s)
        self.vel_angular = self.get_parameter('angular_vel').value # (rad/s)
        # Control
        self.kp_linear = 0.9
        self.kp_angular = 5.0
        self.x_err = 0.0
        self.y_err = 0.0
        self.dist_err = 0.0
        self.yaw_err = 0.0
        # Postition
        # Actual position
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # Desired position
        self.x_set = 0.0
        self.y_set = 0.0
        self.yaw_set = 0.0 # rad

        # Message
        self.vel = Twist() 
        self.pose2D = Pose2D()

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

    def pose2D_callback(self, msg):
        if not self.recieved_initial_pose:
            self.recieved_initial_pose = True
        # Update current position
        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.theta
        #self.get_logger().info(f'Values recieved: {self.x}x, {self.y}y, {self.yaw}rad\n')

    def point_callback(self, msg):
        # Update setpoint
        self.recieved_point = True
        self.x_set = msg.x
        self.y_set = msg.y
        # Calculate yaw setpoint
        self.yaw_set = np.arctan2(self.y_set - self.y, self.x_set - self.x)
        # Normalize yaw setpoint
        self.yaw_set = self.normalize_angle(self.yaw_set)
        self.get_logger().info(f'Point recieved: {self.x_set}x, {self.y_set}y\n')

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def stabilize_position(self):
        # Stop the robot and wait for 1 second
        sleep_time = 1.0 # seconds
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel)
        self.get_logger().info("Stabilizing position")
        # Create a Duration object and sleep
        sleep_duration = rclpy.duration.Duration(seconds=sleep_time)
        self.get_clock().sleep_for(sleep_duration)
        self.get_logger().info("Stabilization complete")
    
    def control_combined(self):
        # Control for linear and angular velocities
        # Calculate errors
        self.x_err = self.x_set - self.x
        self.y_err = self.y_set - self.y
        self.dist_err = np.sqrt(self.x_err**2 + self.y_err**2)
        self.yaw_err = self.normalize_angle(self.yaw_set - self.yaw) 
        # Log setpoint
        self.get_logger().info(f'Setpoint: {self.x_set}x, {self.y_set}y, {self.yaw_set}rad\n')
        # Calculate control signals
        self.vel.linear.x = self.kp_linear * self.dist_err
        self.vel.angular.z = self.kp_angular * self.yaw_err
        # Limit linear velocities
        if self.vel.linear.x > self.vel_linear:
            self.vel.linear.x = self.vel_linear
        # Limit angular velocities 
        # Positive and negative limits
        if self.vel.angular.z > 0:
            if self.vel.angular.z > self.vel_angular:
                self.vel.angular.z = self.vel_angular
        elif self.vel.angular.z < 0:
            if self.vel.angular.z < -self.vel_angular:
                self.vel.angular.z = -self.vel_angular
        # Publish the message
        self.cmd_vel_pub.publish(self.vel)

        #self.get_logger().info(f'Control: [{self.vel.linear.x}] linear vel, [{self.vel.angular.z}] angular vel\n')
    
    def control_linear(self):
        # Control only for linear velocity
        self.x_err = self.x_set - self.x
        self.y_err = self.y_set - self.y
        self.dist_err = np.sqrt(self.x_err**2 + self.y_err**2)
        # Control
        self.vel.linear.x = self.kp_linear * self.dist_err
        self.vel.angular.z = 0.0
        # Limit the velocity
        if self.vel.linear.x > self.vel_linear:
            self.vel.linear.x = self.vel_linear
        # Publish the message
        self.cmd_vel_pub.publish(self.vel)

        #self.get_logger().info(f'Control: [{self.vel.linear.x}] linear vel, [{self.vel.angular.z}] angular vel\n')

    def control_angular(self):
        # Control only for angular velocity
        self.x_err = self.x_set - self.x
        self.y_err = self.y_set - self.y
        self.dist_err = np.sqrt(self.x_err**2 + self.y_err**2)
        self.yaw_err = self.normalize_angle(self.yaw_set - self.yaw)
        # Control
        self.vel.linear.x = 0.0
        self.vel.angular.z = self.kp_angular * self.yaw_err
        # Limit the velocity
        if self.vel.angular.z > 0:
            if self.vel.angular.z > self.vel_angular:
                self.vel.angular.z = self.vel_angular
        elif self.vel.angular.z < 0:
            if self.vel.angular.z < -self.vel_angular:
                self.vel.angular.z = -self.vel_angular
        # Publish the message
        self.cmd_vel_pub.publish(self.vel)

        #self.get_logger().info(f'Control: [{self.vel.linear.x}] linear vel, [{self.vel.angular.z}] angular vel\n')
        

    def timer_callback(self): 
        # Main loop, state machine

        # Initial state
        if self.state == "stop": 
            self.vel.linear.x = 0.0 # m/s 
            self.vel.angular.z = 0.0 # rad/s 
            self.cmd_vel_pub.publish(self.vel) #publish the message 

            # Wait for initial localization (odometry) pose
            if self.first_time and self.recieved_initial_pose: 
                self.first_time = False
                self.confirmation_pub.publish(String(data='ok')) # Publish confirmation message
                self.get_logger().info("First confirmation message sent")
                return
            elif self.recieved_point: # Check if the point has been received
                self.recieved_point = False
                self.state = "turn" #Change the state to move forward 
                # Log current position
                self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.yaw}rad\n')
                self.get_logger().info("Moving to point")
                return
        
        # Turn to the yaw setpoint
        elif self.state == "turn":
            self.control_angular() # Apply control and pubilsh vel

            # Check if the robot is close to the setpoint
            if abs(self.yaw_err) < 0.05:
                self.state = "move"
                self.get_logger().info("Turning complete")
                self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.yaw}rad\n')
                self.stabilize_position() # Send velocities to 0 and wait 1 sec

        # Move forward to the  point setpoint
        elif self.state == "move": 
            self.control_linear() # Apply control and pubilsh vel

            # Log current position
            #self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.yaw}rad\n')

            # Check if the robot is close to the setpoint
            if self.dist_err < 0.15: #Check if the robot is close to the setpoint
                self.state = "wait" 
                self.send_confirmation = True # Set the flag to wait for confirmation
                # Log current position
                self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.yaw}rad\n')

                self.stabilize_position() # Send velocities to 0 and wait 1 sec
                self.get_logger().info("Waiting for next point")

        # Send confirmation message and wait for next point
        elif self.state == "wait":
            if self.send_confirmation:
                self.send_confirmation = False # Reset the flag
                self.confirmation_pub.publish(String(data='ok')) # Publish confirmation message
                self.get_logger().info("Confirmation message sent")
            
            if self.recieved_point: # Check if the point has been received
                self.state = "turn" # Change the state to turn
                self.recieved_point = False
                self.get_logger().info("Moving to point")
            
        
                
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