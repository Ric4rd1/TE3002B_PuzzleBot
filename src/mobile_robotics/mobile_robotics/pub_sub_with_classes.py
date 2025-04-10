import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist  
from std_msgs.msg import String  

 

class PubSubClass(Node): 
    def __init__(self): 

        super().__init__('pub_sub_with_classes') 
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10) 
        self.sub = self.create_subscription(String, "string_topic", self.listener_callback, 10) 
        timer_period = 0.5 
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("Node initialized!!!") 
        self.vel = Twist() 
        self.my_string = "stop" 
     

    def timer_callback(self): 

        if self.my_string == "move forward": 
            print("moving forward") 
            self.vel.linear.x = 0.1 
            self.vel.angular.z = 0.0 

        else: #stop 
            print("stopped") 
            self.vel.linear.x = 0.0 
            self.vel.angular.z = 0.0 

        self.cmd_vel_pub.publish(self.vel) #publish the message 

         

    def listener_callback(self, msg): 
        ## This function receives the ROS message as the msg variable.  
        self.my_string =  msg.data #msg.data is the string contained inside the ROS message  
        print("I received this message in the callback: " + self.my_string) 

 

 

def main(args=None): 
    rclpy.init(args=args) 
    m_p=PubSubClass() 
    rclpy.spin(m_p) 
    m_p.destroy_node() 
    rclpy.shutdown() 

     

if __name__ == '__main__': 
    main() 