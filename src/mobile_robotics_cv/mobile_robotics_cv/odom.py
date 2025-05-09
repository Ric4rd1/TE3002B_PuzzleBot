import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Pose2D 
from std_msgs.msg import Float32 
from rclpy import qos 
import numpy as np 
 
#This class will compute the pose of the robot from the encoder readings. 
# This node subscribes to the /VelocityEncR and /VelocityEncL topics 
# This node publishes the pose of the robot to the /pose topic.  
class Odometry(Node):  
    def __init__(self):  
        super().__init__('odometry_node') 
        ###########  INIT PUBLISHERS ################ 
        self.pub_pose = self.create_publisher(Pose2D, 'pose', 10)  
        ############## SUBSCRIBERS ##################  
        self.create_subscription(Float32, "VelocityEncR",  self.wr_cb, qos.qos_profile_sensor_data)  
        self.create_subscription(Float32, "VelocityEncL",  self.wl_cb, qos.qos_profile_sensor_data)  
        ############ ROBOT CONSTANTS ################  
        self.r=0.0505 #wheel radius for our simulated robot[m] 
        self.L=0.175 #wheel separation for our simulated robot [m] 
        self.wl = 0.0 #Left wheel speed [rad/s] 
        self.wr = 0.0 #Right wheel speed [rad/s] 
        self.x = 0.0 #Robot position in x-axis [m] 
        self.y = 0.0 #Robot position in y-axis [m] 
        self.theta = 0.0 #Robot orientation [rad] 
        self.ang_correction = 1.1
        self.lin_correction = 1.12
        self.robot_pose = Pose2D() 
 
        self.prev_time_ns = self.get_clock().now().nanoseconds #Previous time in nanoseconds 
        timer_period = 0.05 
 
        self.create_timer(timer_period, self.main_timer_cb) 
        self.get_logger().info("Node initialized!!") 
 
 
             
    def main_timer_cb(self): 
         
        v,w = self.get_robot_velocity(self.wl, self.wr) #get the robot's speed 
        self.update_robot_pose(v, w) # get the pose of the robot returns [xr, yr, theta_r] 
        #print("xr: " + str(self.x)) 
        #print("yr: " + str(self.y)) 
        #print("theta_r: " + str(self.theta)) 
        self.get_logger().info(f"Pose: x={self.x:.4g}, y={self.y:.4g}, theta={self.theta:.4g}")
 
        self.robot_pose.x = self.x 
        self.robot_pose.y = self.y 
        self.robot_pose.theta = self.theta 
 
 
        # Publish the robot pose 
        self.pub_pose.publish(self.robot_pose) 
         
 
 
    def wl_cb(self, wl):  
        ## This function receives the left wheel speed from the encoders
        '''
        if wl.data > 0.0001: # This is to avoid the robot to move when it is not moving
            self.wl = wl.data
        else:
            self.wl = 0.0
        '''
        self.wl = wl.data
         
    def wr_cb(self, wr):  
        ## This function receives the right wheel speed from the encoders
        '''
        if wr.data > 0.0001:
            self.wr = wr.data
        else:
            self.wr = 0.0
        ''' 
        self.wr = wr.data
 
    def get_robot_velocity(self, wl, wr): 
        v = self.r * (wr + wl) / 2.0 #Compute the robot linear velocity [m/s] 
        w = self.r * (wr - wl) / self.L #Compute the robot angular speed [rad/s] 
        #print("v: " + str(v)) 
        #print("w: " + str(w)) 
        return v, w 
     
    def update_robot_pose(self, v, w): 
        dt = (self.get_clock().now().nanoseconds - self.prev_time_ns)*10**-9 #Delta t in seconds 
        self.x = self.x + v*np.cos(self.theta)*dt*self.lin_correction
        self.y = self.y + v*np.sin(self.theta)*dt*self.lin_correction
        self.theta = self.theta + w*dt*self.ang_correction 
        # Crop this angle to [-pi,pi] 
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta)) 

        self.prev_time_ns = self.get_clock().now().nanoseconds 
 
 
def main(args=None): 
    rclpy.init(args=args) 
    odom_node=Odometry() 
    try:
        rclpy.spin(odom_node)
    except KeyboardInterrupt:
        pass
    finally:
        odom_node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__': 
    main() 