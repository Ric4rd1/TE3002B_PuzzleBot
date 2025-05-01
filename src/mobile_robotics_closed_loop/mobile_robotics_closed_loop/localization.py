import rclpy
import numpy as np
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

class Localization(Node):
    def __init__(self):
        super().__init__('localization_node')

        # Subscribers
        self.vel_encL_sub = self.create_subscription(Float32, 'VelocityEncL', self.vel_encL_callback, qos.qos_profile_sensor_data)
        self.vel_encR_sub = self.create_subscription(Float32, 'VelocityEncR', self.vel_encR_callback, qos.qos_profile_sensor_data)

        # Publisher
        self.pose2D_pub = self.create_publisher(Pose2D, 'pose2D', qos.qos_profile_sensor_data) 

        # Timer
        self.sample_time = 0.02 # seconds
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.sample_time, self.timer_callback)

        # Constants
        self.radius = 0.0505 #radius of the wheel [m]
        #self.l = 0.172 # Distance between wheels [m]
        self.l = 0.172/2.0 # Distance between wheels [m]
        self.ang_correction = 1.09
        self.lin_correction = 1.09

        # Variables
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        # Position
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # velocity
        self.xp = 0.0
        self.yp = 0.0
        self.yawp = 0.0

        # Internal state
        self.current_time = 0.0
        self.last_time = 0.0

        # Messages
        self.encL_vel = Float32()
        self.encR_vel = Float32()
        self.pose2D = Pose2D()

        

        self.get_logger().info("Localization node initailized!!!") 



    def vel_encL_callback(self, msg):
        if msg.data > 0.001:
            self.encL_vel = msg
        else:
            self.encL_vel.data = 0.0

    def vel_encR_callback(self, msg):
        if msg.data > 0.001:
            self.encR_vel = msg
        else:
            self.encR_vel.data = 0.0

    def timer_callback(self):
        # Get current time and compute dt
        current_time = self.get_clock().now().nanoseconds
        dt = (current_time - self.last_time) * 10**-9 # Convert to seconds
        if dt > self.sample_time:
            # Calculate linear and angular vel
            self.linear_vel = self.radius*((self.encR_vel.data + self.encL_vel.data)/2.0)
            self.angular_vel = self.radius*((self.encR_vel.data - self.encL_vel.data)/self.l)

            # Calculate velcocity components
            self.xp = self.linear_vel*np.cos(self.yaw)
            self.yp = self.linear_vel*np.sin(self.yaw)
            self.yawp = self.angular_vel

            # Estimate position with euler 
            self.x = self.x + self.xp * dt * self.lin_correction
            self.y = self.y + self.yp * dt * self.lin_correction
            self.yaw = self.yaw + self.yawp * dt * self.ang_correction
            self.yaw = (self.yaw + np.pi) % (2 * np.pi) - np.pi # Normalize to [-pi,pi]

            self.last_time = current_time

            # Publish the message
            self.pose2D.x = round(self.x, 4)
            self.pose2D.y = round(self.y, 4)
            self.pose2D.theta = round(self.yaw, 4)
            self.pose2D_pub.publish(self.pose2D)

            #self.get_logger().info(f"DT: {dt:.4f}, LinVel: {self.linear_vel:.4f}, AngVel: {self.angular_vel:.4f}")
            #self.get_logger().info(f"Yaw: {self.yaw:.4f}, X: {self.x:.4f}, Y: {self.y:.4f}")


def main(args=None):
    rclpy.init(args=args)
    Localization_node = Localization()

    try:
        rclpy.spin(Localization_node)
    except KeyboardInterrupt:
        pass
    finally:
        Localization_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()