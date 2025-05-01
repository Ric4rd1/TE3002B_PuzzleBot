
import rclpy 
from rclpy.node import Node 
import rclpy.duration
from rclpy import qos
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
import numpy as np
import signal
import sys
 
# Move in a square of 2m length (closed loop)

class MoveSquare(Node): 
    def __init__(self): 

        super().__init__('move_square') #Init the node with the name "move_forward" 
        # Handle shutdown gracefully 
        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function 

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10) 

        # Subscriber
        self.pose2D_sub = self.create_subscription(Pose2D, "pose2D", self.pose2D_callback, qos.qos_profile_sensor_data)

        # Timer
        timer_period = 0.05 #Time in seconds to call the timer_callback function 
        self.timer = self.create_timer(timer_period, self.timer_callback) 

        # Variables
        self.state = 'stop' # Initial state
        self.first_time = True # flag
        self.recieved_initial_pose = False
        self.counter = 0 
        # Constants
        self.length = 2.0 # square length (m)
        self.vel_linear = 0.2 # (m/s)
        self.vel_angular = 0.6 # (rad/s)
        # Control
        self.kp_linear = 0.9
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
        # Control for linear and angular velocity
        # Calculate the error
        self.x_err = self.x_set - self.x
        self.y_err = self.y_set - self.y
        self.dist_err = np.sqrt(self.x_err**2 + self.y_err**2)
        self.yaw_err = self.normalize_angle(self.yaw_set - self.yaw)
        # Control
        self.vel.linear.x = self.kp_linear * self.dist_err
        self.vel.angular.z = self.kp_angular * self.yaw_err
        # Limit linear velocity
        if self.vel.linear.x > self.vel_linear:
            self.vel.linear.x = self.vel_linear
        # Limit angular velocity
        # Postive and negative limits
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

            if self.first_time and self.recieved_initial_pose: 
                self.first_time = False

                # Update setpoint
                self.x_set = self.length
                self.y_set = 0.0
                self.yaw_set = 0.0

                # Log current position
                self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.yaw}rad\n')
                self.state = "move_forward" #Change the state to move forward 
                self.get_logger().info("Moving forward")
                return 

        # Move forward for the length of the square
        elif self.state == "move_forward": 
            self.control_combined() # Apply control and pubilsh vel
            # Log current position
            #self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.yaw}rad\n')
            if self.dist_err < 0.05: #Check if the robot is close to the setpoint
                self.state = "turn" #Change the state to turn
                
                # Update setpoint
                self.yaw_set += -1.57 # rad

                # Log current position
                self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.yaw}rad\n')

                self.stabilize_position() # Call the stabilize position function
                self.get_logger().info("Turning")

        # Turn -90 degrees (-1.57 rad)
        elif self.state == "turn":
            self.control_angular() # Apply control and pubilsh vel
            # Log velocity
            #self.get_logger().info(f'Control: [{self.vel.linear.x}] linear vel, [{self.vel.angular.z}] angular vel\n')
            if abs(self.yaw_err) < 0.01:
                self.counter += 1

                # Log current position
                self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.yaw}rad\n')

                if self.counter < 4:
                    self.state = "move_forward"

                    # Update setpoint
                    if self.counter == 1:
                        self.x_set = self.length
                        self.y_set = -self.length
                    elif self.counter == 2:
                        self.x_set = 0.0
                        self.y_set = -self.length
                    elif self.counter == 3:
                        self.x_set = 0.0
                        self.y_set = 0.0

                    self.stabilize_position() # Call the stabilize position function
                    self.get_logger().info("Moving forward")
                else:
                    self.state = "stop"
                    self.get_logger().info("Stopping")
                
                self.start_time = self.get_clock().now() #Update the time when the robot started moving 

                
def main(args=None): 
    rclpy.init(args=args) 
    move_square = MoveSquare() 
    try:
        rclpy.spin(move_square)
    except KeyboardInterrupt:
        pass
    finally:
        move_square.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__': 
    main() 