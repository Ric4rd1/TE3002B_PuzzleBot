import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Pose2D, Twist
from rclpy import qos 
import numpy as np 

class MoveTest(Node):
    def __init__(self):
        super().__init__('move_test_node')

        self.pose_sub = self.create_subscription(Pose2D, 'pose', self.pose_callback, qos.qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(Twist, 'cmd_vel', qos.qos_profile_sensor_data)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.moving = True
        self.move_speed = 0.2 #[rad/s]
        self.turn_speed = 0.8 #[rad/s]
        self.setpoint = [-1.2, -1.2] # [x, y]

        dt = 0.05
        self.timer = self.create_timer(dt, self.timer_callback)

        self.cmd_vel = Twist()

        self.get_logger().info("Move test node initialized!!")
        self.get_logger().info("Moving to point")

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.get_logger().info(f"Pose: x={self.x:.4g}, y={self.y:.4g}, theta={self.theta:.4g}")

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def pControl(self, x, y, theta):
        # Calculate errors
        angle2goal = np.arctan2(self.setpoint[1] - y, self.setpoint[0] - x)
        self.yaw_error = self.normalize_angle(angle2goal - theta)
        self.distance_error = np.sqrt((self.setpoint[0] - x)**2 + (self.setpoint[1] - y)**2)

        # Control signals
        control_ang = 0.8 * self.yaw_error
        control_lin = 0.9 * self.distance_error
        
        # Limit velocities
        min_vel_lin = 0.1
        max_vel_lin = self.move_speed
        if abs(control_lin) > max_vel_lin:
            control_lin = np.sign(control_lin) * max_vel_lin
        elif abs(control_lin) < min_vel_lin:
            control_lin = np.sign(control_lin) * min_vel_lin

        min_vel_ang = 0.1
        max_vel_ang = self.turn_speed
        if abs(control_ang) > max_vel_ang:
            control_ang = np.sign(control_ang) * max_vel_ang
        elif abs(control_ang) < min_vel_ang:
            control_ang = np.sign(control_ang) * min_vel_ang

        return control_lin, control_ang
    
    def pControl_ang(self):
        control_ang = 0.8 * self.yaw_error
        min_vel = 0.35
        max_vel = self.turn_speed

        if abs(control_ang) > max_vel:
            control_ang = np.sign(control_ang) * max_vel
        elif abs(control_ang) < min_vel:
            control_ang = np.sign(control_ang) * min_vel
        return control_ang

    def timer_callback(self):
        if self.moving:
            # Calculate control signals
            lin_vel, ang_vel = self.pControl(self.x, self.y, self.theta)
            # Publish control signals
            self.cmd_vel.linear.x = lin_vel
            self.cmd_vel.angular.z = ang_vel
            self.pose_pub.publish(self.cmd_vel)

            # Check if the robot has reached the setpoint
            if self.distance_error < 0.02 and abs(self.yaw_error) < 0.1: 
                stop=Twist()
                self.pose_pub.publish(stop)
                self.moving = False
                self.get_logger().info("Moving finished")

                # Stop the timer and shut down
                self.timer.cancel()
                rclpy.shutdown()
            
def main(args=None):
    rclpy.init(args=args)
    move_test = MoveTest()

    try:
        rclpy.spin(move_test)
    except KeyboardInterrupt:
        pass
    finally:
        move_test.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
