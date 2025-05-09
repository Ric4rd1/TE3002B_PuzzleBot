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
        self.move_distance = 2.0

        dt = 0.05
        self.timer = self.create_timer(dt, self.timer_callback)

        self.cmd_vel = Twist()

        self.get_logger().info("Move test node initialized!!")
        self.get_logger().info("Moving")

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.get_logger().info(f"Pose: x={self.x:.4g}, y={self.y:.4g}, theta={self.theta:.4g}")

    def pControl_lin(self, x, y, theta):
        lin_error = self.move_distance - x
        control_lin = 0.9 * lin_error

        min_vel = 0.1
        max_vel = self.move_speed
        if abs(control_lin) > max_vel:
            control_lin = np.sign(control_lin) * max_vel
        elif abs(control_lin) < min_vel:
            control_lin = np.sign(control_lin) * min_vel
        return control_lin

    def timer_callback(self):
        if self.moving:
            lin_vel = self.pControl_lin(self.x, self.y, self.theta)
            self.cmd_vel.linear.x = lin_vel
            self.cmd_vel.angular.z = 0.0
            self.pose_pub.publish(self.cmd_vel)
            if abs(self.x) > self.move_distance: 
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
