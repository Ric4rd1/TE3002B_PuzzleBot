# Move on its own axis a certain distance given by the topic /angle (open loop)

import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
 

class MoveForwardClass(Node): 
    def __init__(self): 

        super().__init__('move_angle') #Init the node with the name "move_forward" 
        # Declare necessary variables 
        self.start_time = self.get_clock().now() #Indicate the time when the robot starts moving.   

        # Init ROS subscriber and publisher 
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10) 
        self.distance_sub = self.create_subscription(Float32, "angle", self.distance_callback, 10)

        # Declare the ROS timer 
        timer_period = 0.05 #Time in seconds to call the timer_callback function 
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("Node initialized!!!") 

        self.state = 'stop'
        self.recieved_angle= False
        self.angle = 0
        self.t = 0
        self.tplus = 0.42
        self.default_angular_velocity = 0.2  # Always positive
        self.vel = Twist() 

    def distance_callback(self, msg):
        self.angle = msg.data
        self.vel_angular = self.default_angular_velocity if self.angle >= 0 else -self.default_angular_velocity
        self.t = abs(self.angle / self.vel_angular) + self.tplus
        self.recieved_angle = True
        print("I received this message in the callback: " + str(self.angle)) 
        
        

    def timer_callback(self): 

        if self.state == "stop": 
            self.vel.linear.x = 0.0 # m/s 
            self.vel.angular.z = 0.0 # rad/s 
            self.cmd_vel_pub.publish(self.vel) #publish the message 

            if self.recieved_angle: 
                self.state = "move_angle" #Change the state to move forward 
                self.start_time = self.get_clock().now() #Update the time when the robot started moving 
                self.get_logger().info("Moving on z axis") 

        elif self.state == "move_angle": 
            self.vel.linear.x = 0.0 # m/s 
            self.vel.angular.z = self.vel_angular # rad/s 
            self.cmd_vel_pub.publish(self.vel) #publish the message 

            if self.get_clock().now().nanoseconds - self.start_time.nanoseconds >= self.t*10**9: 
                self.state = "stop" #Change the state to stop 
                self.recieved_angle = False
                self.angle = 0
                self.get_logger().info("Stopping") 

def main(args=None): 
    rclpy.init(args=args) 
    m_p=MoveForwardClass() 
    rclpy.spin(m_p) 
    m_p.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 