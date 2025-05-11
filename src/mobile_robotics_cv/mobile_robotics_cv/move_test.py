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

        self.turning = True
        self.turn_speed = 0.8 #[rad/s]
        self.turn_angle = -1.57 # [rad]
        self.turn_correction_positive = self.turn_angle * 0.025 # Compensate for the robot turning too much
        self.turn_correction_negative = abs(self.turn_angle) * 0.029 # Compensate for the robot turning too little

        if self.turn_angle > 0:
            self.turn_correction = self.turn_correction_positive
        else:
            self.turn_correction = self.turn_correction_negative

        dt = 0.05
        self.timer = self.create_timer(dt, self.timer_callback)

        self.cmd_vel = Twist()

        self.get_logger().info("Move test node initialized!!")
        self.get_logger().info("Turning")

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.get_logger().info(f"Pose: x={self.x:.4g}, y={self.y:.4g}, theta={self.theta:.4g}")

    def pControl_ang(self, x, y, theta):
        yaw_error = self.turn_angle - theta
        # Normalize yaw_error to [-pi, pi]
        if yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        elif yaw_error < -np.pi:
            yaw_error += 2 * np.pi
        control_ang = 0.8 * yaw_error

        min_vel = 0.35
        max_vel = self.turn_speed
        if abs(control_ang) > max_vel:
            control_ang = np.sign(self.turn_angle) * max_vel
        elif abs(control_ang) < min_vel:
            control_ang = np.sign(self.turn_angle) * min_vel
        return control_ang

    def timer_callback(self):
        if self.turning:
            ang_vel = self.pControl_ang(self.x, self.y, self.theta)
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = ang_vel
            self.pose_pub.publish(self.cmd_vel)

            
            if abs(self.theta) > (abs(self.turn_angle) - self.turn_correction):  
                stop=Twist()
                self.pose_pub.publish(stop)
                self.turning = False
                self.get_logger().info("Turning finished")

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
