
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

        # Handle shutdown gracefully 
        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function 

        # Parameters
        self.declare_parameter('linear_vel', 0.2) # (m/s)
        self.declare_parameter('angular_vel', 0.8) # (rad/s)

        # Publishers
        # Publisher for sending velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10) 
        # Publisher for sending confirmation to get next point
        self.confirmation_pub = self.create_publisher(String, "confirmation", 10)

        # Subscribers
        # Subscriber for receiving actual pose of the robot
        self.pose2D_sub = self.create_subscription(Pose2D, "pose", self.pose2D_callback, qos.qos_profile_sensor_data)
        # Subscriber for receiving point to move to
        self.point_sub = self.create_subscription(Pose2D, "point", self.point_callback, 10)

        # Timer
        timer_period = 0.05 #Time in seconds to call the timer_callback function 
        self.timer = self.create_timer(timer_period, self.timer_callback) 

        # Variables
        self.state = 'stop' # Initial state
        self.counter = 0 
        self.first_time = True # Flag to check if the first time is called
        self.recieved_initial_pose = False # Flag to check if the initial pose is received
        self.recieved_point = False # Flag to check if the point is received
        # Constants
        self.vel_linear = self.get_parameter('linear_vel').value # (rad/s)
        self.vel_angular = self.get_parameter('angular_vel').value # (rad/s)
        # Control
        self.kp_linear = 0.9
        self.kp_angular = 0.8
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

    def pose2D_callback(self, msg):
        if not self.recieved_initial_pose:
            self.recieved_initial_pose = True
        # Update current position
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        #self.get_logger().info(f'Values recieved: {self.x}x, {self.y}y, {self.yaw}rad\n')

    def point_callback(self, msg):
        # Update setpoint
        self.recieved_point = True
        self.setpoint = [msg.x, msg.y]
        self.get_logger().info(f'Point recieved: {self.setpoint[0]}x, {self.setpoint[1]}y\n')

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def angle2wide(self, x, y, x2, y2, angle):
        # Calculate the angle to the goal
        angle2goal = np.arctan2(y2 - y, x2 - x)
        angle2goal = self.normalize_angle(angle2goal - self.theta) # Normalize the angle to [-pi, pi]
        return True if abs(angle2goal) > angle else False
    
    def distance2short(self, x, y, x2, y2, distance):
        # Calculate the distance to the goal
        distance2goal = np.sqrt((x2 - x)**2 + (y2 - y)**2)
        return True if distance2goal < distance else False
    
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
    
    def condition_1(self, x, y, x2, y2, angle=0, distance=0):
        # Check if the angle is too wide
        return self.angle2wide(x, y, x2, y2, angle)
    
    def condition_2(self, x, y, x2, y2, angle=0, distance=0):
        # Check if the distance is too short and the angle is too wide
        cond1 = self.distance2short(x, y, x2, y2, distance) 
        cond2 = self.angle2wide(x, y, x2, y2, angle)
        return cond1 and cond2
    
    def condition_3(self, x, y, x2, y2, angle=0, distance=0):
        return self.distance2short(x, y, x2, y2, distance) 
    
    

    def timer_callback(self): 
        # Main loop, state machine

        # Initial state
        if self.state == "stop": 
            self.cmd_vel.linear.x = 0.0 # m/s 
            self.cmd_vel.angular.z = 0.0 # rad/s 
            self.cmd_vel_pub.publish(self.cmd_vel) #publish the message 

            # Wait for initial localization (odometry) pose
            if self.first_time and self.recieved_initial_pose: 
                self.first_time = False
                self.confirmation_pub.publish(String(data='ok')) # Publish confirmation message
                self.get_logger().info("First confirmation message sent")
                return
            elif self.recieved_point: # Check if the point has been received
                self.recieved_point = False
                self.state = "evaluate"
                return
        
        # Evaluate the conditions and decide to turn (just angular) or move (with both angular and linear)     
        elif self.state == "evaluate":
            # Check if the angle is too wider than 2.09 rad
            if (self.condition_1(self.x, self.y, self.setpoint[0], self.setpoint[1], angle=1.48) or 
                self.condition_3(self.x, self.y, self.setpoint[0], self.setpoint[1], distance=0.3) or
                self.condition_2(self.x, self.y, self.setpoint[0], self.setpoint[1], angle=0.4, distance=0.5) or
                self.condition_2(self.x, self.y, self.setpoint[0], self.setpoint[1], angle=0.61, distance=0.7) or 
                self.condition_2(self.x, self.y, self.setpoint[0], self.setpoint[1], angle=0.7, distance=0.9) or 
                self.condition_2(self.x, self.y, self.setpoint[0], self.setpoint[1], angle=0.78, distance=1.1) or
                self.condition_2(self.x, self.y, self.setpoint[0], self.setpoint[1], angle=0.87, distance=1.3)):
                
                self.state = "turn"
                # Log current position
                self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.theta}rad\n')
                self.get_logger().info("Turning first, angle is too wide")
                return
                
            else:
                self.state = "move" #Change the state to move forward 
                # Log current position
                self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.theta}rad\n')
                self.get_logger().info("Moving to point")

        
        # Turn to the yaw setpoint
        elif self.state == "turn":
            # Calculate control signals
            ang_vel = self.pControl_ang(self.x, self.y, self.theta)
            # Publish control signals
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = ang_vel
            self.cmd_vel_pub.publish(self.cmd_vel)
            
            # Check if the robot is close to the setpoint
            if abs(self.yaw_error) < 0.05:
                self.state = "move"
                self.get_logger().info("Turning complete")
                self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.theta}rad\n')

        # Move to the  point setpoint
        elif self.state == "move": 
            lin_vel, ang_vel = self.pControl(self.x, self.y, self.theta)
            # Publish control signals
            self.cmd_vel.linear.x = lin_vel
            self.cmd_vel.angular.z = ang_vel
            self.cmd_vel_pub.publish(self.cmd_vel)

            # Log current position
            #self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.yaw}rad\n')

            # Check if the robot is close to the setpoint
            if self.distance_error < 0.02 and abs(self.yaw_error) < 0.1: #Check if the robot is close to the setpoint
                # Log current position
                self.get_logger().info(f'Current position: {self.x}x, {self.y}y, {self.theta}rad\n')
                # Stop the robot
                stop=Twist()
                self.cmd_vel_pub.publish(stop)
                self.get_logger().info("Moving finished")
                # Send confirmation message and wait for next point
                self.state = "wait" 
                self.confirmation_pub.publish(String(data='ok')) # Publish confirmation message
                self.get_logger().info("Confirmation message sent")

        # Send confirmation message and wait for next point
        elif self.state == "wait":
            #stop=Twist()
            #self.cmd_vel_pub.publish(stop)
            if self.recieved_point: # Check if the point has been received
                self.state = "evaluate" # Change the state to turn
                self.recieved_point = False
            
        
                
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