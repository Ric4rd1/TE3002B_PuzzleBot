
import rclpy 
from rclpy.node import Node 
import rclpy.duration
from rclpy import qos
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String
import numpy as np
 
# Move point to point, 4 points total (closed loop) points are given topic /point

class MovePoints(Node): 
    def __init__(self): 
        super().__init__('move_points')
        sleep_duration = rclpy.duration.Duration(seconds=2.0)
        self.get_clock().sleep_for(sleep_duration) 

        self.start_time = self.get_clock().now() #Indicate the time when the robot starts moving.   
        # Parameters
        self.declare_parameter('linear_vel', 0.2) # (m/s)
        self.declare_parameter('angular_vel', 0.6) # (rad/s)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10) 
        self.confirmation_pub = self.create_publisher(String, "confirmation", 10)

        # Subscriber
        self.pose2D_sub = self.create_subscription(Pose2D, "pose2D", self.pose2D_callback, qos.qos_profile_sensor_data)
        self.point_sub = self.create_subscription(Pose2D, "point", self.point_callback, 10)

        # Timer
        timer_period = 0.05 #Time in seconds to call the timer_callback function 
        self.timer = self.create_timer(timer_period, self.timer_callback) 

        # Variables
        self.state = 'stop' # Initial state
        self.first_time = True # flag
        self.recieved_initial_pose = False
        self.recieved_point = False
        self.send_confirmation = False
        self.counter = 0 
        # Constants
        self.vel_linear = self.get_parameter('linear_vel').value # (m/s)
        self.vel_angular = self.get_parameter('angular_vel').value # (rad/s)
        self.calibration_factor_l = 0.95 # Calibration factor for the robot
        # Control
        self.kp_linear = 1.0
        self.kp_angular = 4.0
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

    def pose2D_callback(self, msg):
        if not self.recieved_initial_pose:
            self.recieved_initial_pose = True
        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.theta
        #self.get_logger().info(f'Values recieved: {self.x}x, {self.y}y, {self.yaw}rad\n')

    def point_callback(self, msg):
        self.recieved_point = True
        self.x_set = msg.x
        self.y_set = msg.y
        self.yaw_set = np.arctan2(self.y_set - self.y, self.x_set - self.x)
        self.yaw_set = self.normalize_angle(self.yaw_set)
        self.get_logger().info(f'Point recieved: {self.x_set}x, {self.y_set}y\n')

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def stabilize_position(self):
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
        self.x_err = self.x_set - self.x
        self.y_err = self.y_set - self.y
        self.dist_err = np.sqrt(self.x_err**2 + self.y_err**2)
        self.yaw_err = self.normalize_angle(self.yaw_set - self.yaw)
        # Log setpoint
        self.get_logger().info(f'Setpoint: {self.x_set}x, {self.y_set}y, {self.yaw_set}rad\n')
        # Control
        self.vel.linear.x = self.kp_linear * self.dist_err
        self.vel.angular.z = self.kp_angular * self.yaw_err
        # Limit the velocity
        if self.vel.linear.x > self.vel_linear:
            self.vel.linear.x = self.vel_linear
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

        if self.state == "stop": 
            self.vel.linear.x = 0.0 # m/s 
            self.vel.angular.z = 0.0 # rad/s 
            self.cmd_vel_pub.publish(self.vel) #publish the message 


            if self.first_time and self.recieved_initial_pose: 
                self.first_time = False
                self.confirmation_pub.publish(String(data='ok')) # Publish confirmation message
                self.get_logger().info("First confirmation message sent")
                return
            elif self.recieved_point: # Check if the point has been received
                self.recieved_point = False
                self.state = "move" #Change the state to move forward 
                # Log current position
                self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.yaw}rad\n')
                self.get_logger().info("Moving to point")
                return


        elif self.state == "move": 
            self.control_combined() # Apply control and pubilsh vel
            # Log current position
            #self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.yaw}rad\n')
            if self.dist_err < 0.1: #Check if the robot is close to the setpoint
                self.state = "wait" 
                self.send_confirmation = True # Set the flag to wait for confirmation
                # Log current position
                self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.yaw}rad\n')

                self.stabilize_position() # Call the stabilize position function
                self.get_logger().info("Waiting for next point")

        elif self.state == "wait":
            if self.send_confirmation:
                self.send_confirmation = False # Reset the flag
                self.confirmation_pub.publish(String(data='ok')) # Publish confirmation message
                self.get_logger().info("Confirmation message sent")
            
            if self.recieved_point: # Check if the point has been received
                self.state = "move" # Change the state to turn
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