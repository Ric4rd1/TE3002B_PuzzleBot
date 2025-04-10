# Recieve points 
import numpy as np
import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from numpy import pi as pi
import time

class State:
    MOVE_FORWARD = 0
    TURN = 1
    STOP = 2
 

class ControllerPoints(Node): 
    def __init__(self): 
        time.sleep(1) #Wait for 1 second before starting the node

        super().__init__('controller_points') #Init the node with the name "move_forward" 
        
        # Declare necessary variables 
        self.start_time = self.get_clock().now() #Indicate the time when the robot starts moving.   

        # Publisher 
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.confirmation_pub = self.create_publisher(String, "confirmation", 10)

        # Subscriber
        self.points_sub = self.create_subscription(Pose, "pose", self.points_callback, 10)

        # Declare the ROS timer 
        timer_period = 0.05 #Time in seconds to call the timer_callback function 
        self.timer = self.create_timer(timer_period, self.timer_callback) 

        # Declare necessary variables
        self.curr_point_x = 0.0
        self.curr_point_y = 0.0
        self.curr_angle = 0.0

        self.linear_distance = 0.0
        self.angular_distance = 0.0

        self.state = State.STOP # Initial state
        self.go = False # flag

        self.vel_linear = 0.2 # (m/s)
        self.vel_angular = 0.5 # (rad/s)
        self.calibration_factor = 0.93 # Calibration factor for the robot


        # Message
        self.vel = Twist() 
        self.confirm_msg = String()
        self.confirm_msg.data = "ok" # Confirmation message to be sent to the path generator
        self.confirmation_pub.publish(self.confirm_msg) # Publish the confirmation message to the path generator
        self.get_logger().info("Controller point node initialized!!!")


    def points_callback(self, msg):
        # Calculate the distance and angle to the next point
        self.linear_distance = np.sqrt((msg.position.x - self.curr_point_x)**2 + (msg.position.y - self.curr_point_y)**2)
        target_angle = np.arctan2(msg.position.y - self.curr_point_y, msg.position.x - self.curr_point_x)
        #self.get_logger().info(f"Target angle: {target_angle}")
        self.angular_distance = self._normalize_angle(target_angle - self.curr_angle)
        self.get_logger().info(f"Angular distance: {self.angular_distance}")
        # Update the current point and angle
        self.curr_point_x = msg.position.x
        self.curr_point_y = msg.position.y
        self.curr_angle = target_angle
        # Calculate the time to turn and move forward
        self.t_angular = abs(self.angular_distance / self.vel_angular) #* self.calibration_factor
        self.t_linear = self.linear_distance / self.vel_linear

        self.get_logger().info(f"Received point: ({msg.position.x}, {msg.position.y})")
        self.start_time = self.get_clock().now() #Update the time when the robot started moving
        self.go = True # Set the flag to true
    
    def _normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def timer_callback(self): 

        if self.state == State.STOP: 
            self.vel.linear.x = 0.0 # m/s 
            self.vel.angular.z = 0.0 # rad/s 
            self.cmd_vel_pub.publish(self.vel) #publish the message 

            if self.go: 
                self.go = False
                self.state = State.TURN #Change the state to move forward 
                self.start_time = self.get_clock().now() #Update the time when the robot started moving 
                self.get_logger().info(f'Moving forward to point ({self.curr_point_x}, {self.curr_point_y})')
                return 

        elif self.state == State.TURN:
            self.vel.linear.x = 0.0 # m/s 
            self.vel.angular.z = self.vel_angular if self.angular_distance > 0 else -self.vel_angular # rad/s
            self.cmd_vel_pub.publish(self.vel)

            if self.get_clock().now().nanoseconds - self.start_time.nanoseconds >= self.t_angular*10**9:
                self.state = State.MOVE_FORWARD #Change the state to move forward
                self.start_time = self.get_clock().now()

        elif self.state == State.MOVE_FORWARD: 
            self.vel.linear.x = self.vel_linear # m/s 
            self.vel.angular.z = 0.0 # rad/s 
            self.cmd_vel_pub.publish(self.vel) 
             
            if self.get_clock().now().nanoseconds - self.start_time.nanoseconds >= self.t_linear*10**9: 
                self.state = State.STOP #Change the state to stop 
                self.start_time = self.get_clock().now()
                self.confirmation_pub.publish(self.confirm_msg)


                
def main(args=None): 
    rclpy.init(args=args) 
    move_square = ControllerPoints() 
    try:
        rclpy.spin(move_square)
    except KeyboardInterrupt:
        pass
    finally:
        move_square.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__': 
    main() 