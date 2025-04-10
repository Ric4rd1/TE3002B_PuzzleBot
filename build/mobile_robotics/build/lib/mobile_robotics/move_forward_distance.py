# Move forward a certain distance given by the topic /distance (open loop)

import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
 

class MoveForwardClass(Node): 
    def __init__(self): 

        super().__init__('move_forward') #Init the node with the name "move_forward" 
        # Declare necessary variables 
        self.start_time = self.get_clock().now() #Indicate the time when the robot starts moving.   

        # Init ROS subscriber and publisher 
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10) 
        self.distance_sub = self.create_subscription(Float32, "distance", self.distance_callback, 10)

        # Declare the ROS timer 
        timer_period = 0.05 #Time in seconds to call the timer_callback function 
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("Node initialized!!!") 

        self.state = 'stop'
        self.recieved_distance = False
        self.distance = 0
        self.t = 0
        self.vel_linear = 0.2
        self.vel = Twist() 

    def distance_callback(self, msg):
        self.distance = msg.data
        self.t = self.distance / self.vel_linear
        self.recieved_distance = True
        print("I received this message in the callback: " + str(self.distance)) 
        
        

    def timer_callback(self): 

        if self.state == "stop": 
            self.vel.linear.x = 0.0 # m/s 
            self.vel.angular.z = 0.0 # rad/s 
            self.cmd_vel_pub.publish(self.vel) #publish the message 

            if self.recieved_distance: 
                self.state = "move_forward" #Change the state to move forward 
                self.start_time = self.get_clock().now() #Update the time when the robot started moving 
                self.get_logger().info("Moving forward") 

        elif self.state == "move_forward": 
            self.vel.linear.x = self.vel_linear # m/s 
            self.vel.angular.z = 0.0 # rad/s 
            self.cmd_vel_pub.publish(self.vel) #publish the message 

            if self.get_clock().now().nanoseconds - self.start_time.nanoseconds >= self.t*10**9: 
                self.state = "stop" #Change the state to stop 
                self.recieved_distance = False
                self.distance = 0
                self.get_logger().info("Stopping") 

def main(args=None): 
    rclpy.init(args=args) 
    m_p=MoveForwardClass() 
    rclpy.spin(m_p) 
    m_p.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 