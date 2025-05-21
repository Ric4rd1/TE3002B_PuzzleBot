"""
Author: Gerardo Sanchez A.k.A SirDuck
File: line_traffic_controller.py
Description: ROS2 node implementing PID line following
with traffic light logic. Subscribes to line_error and
sphere_color topics to adjust behavior and publishes
cmd_vel commands based on PID control and traffic state.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


class LineTrafficController(Node):
    """
    ROS2 node for line traffic control.
    Combines PID control on line_error with
    traffic light state from sphere_color.
    """

    def __init__(self):
        """
        Initialize controller: declare parameters,
        set up subscriptions, publishers, and timer.
        """
        super().__init__('line_traffic_controller')

        # Declare ROS parameters for speeds and PID gains
        self.declare_parameter('base_speed', 0.13)      # m/s on green
        self.declare_parameter('slow_speed', 0.07)      # m/s on yellow
        self.declare_parameter('k_p', 0.001)            # proportional gain
        self.declare_parameter('k_i', 0.00001)         # integral gain
        self.declare_parameter('k_d', 0.00005)         # derivative gain
        self.declare_parameter('max_angular', 0.5)     # rad/s limit
        self.declare_parameter('filter_alpha', 0.7)     # low-pass filter alpha

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribe to line_error (Float32) and sphere_color (String)
        self.create_subscription(Float32, 'line_error', self._on_line_error, 10)
        self.create_subscription(String, 'sphere_color', self._on_sphere_color, 10)

        # Load parameter values
        params = self.get_parameters([
            'base_speed', 'slow_speed',
            'k_p', 'k_i', 'k_d',
            'max_angular', 'filter_alpha'
        ])
        (
            self.base_speed,
            self.slow_speed,
            self.k_p,
            self.k_i,
            self.k_d,
            self.max_angular,
            self.alpha
        ) = [p.value for p in params]

        # Internal PID state
        self.line_error = 0.0
        self.prev_error = 0.0
        self.error_integral = 0.0
        self.filtered_derivative = 0.0

        # Traffic light state: 'green', 'yellow', or 'red'
        self.traffic_state = 'green'

        # Time tracking for derivative/integral
        self.prev_time = self.get_clock().now()

        # Timer at 20 Hz
        self.create_timer(0.05, self._on_timer)

        self.get_logger().info('LineTrafficController listo 🚀')

    def _on_line_error(self, msg: Float32):  # noqa: D102
        # Update the current line error
        self.line_error = msg.data

    def _on_sphere_color(self, msg: String):  # noqa: D102
        # Update traffic state based on detected color
        color = msg.data.lower()
        if color in ('red', 'yellow', 'green'):
            self.traffic_state = color
            self.get_logger().info(f'{color.capitalize()} detected')

    def _on_timer(self):  # noqa: D102
        """
        Timer callback: compute PID, apply traffic rules,
        and publish cmd_vel Twist message.
        """
        now = self.get_clock().now()
        # Compute delta time in seconds, fallback to timer period
        dt = (now - self.prev_time).nanoseconds * 1e-9 or 0.05

        # Raw derivative of error
        raw_derivative = (self.line_error - self.prev_error) / dt

        # Low-pass filter on derivative
        self.filtered_derivative = (
            self.alpha * self.filtered_derivative +
            (1 - self.alpha) * raw_derivative
        )

        # Integral with anti-windup clamped to [-100, 100]
        self.error_integral += self.line_error * dt
        self.error_integral = max(min(self.error_integral, 100.0), -100.0)

        cmd = Twist()

        if self.traffic_state == 'red':
            # Stop on red
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # PID control: omega = -(k_p*e + k_i*I + k_d*D)
            omega = -(
                self.k_p * self.line_error +
                self.k_i * self.error_integral +
                self.k_d * self.filtered_derivative
            )
            # Clamp angular speed
            omega = max(min(omega, self.max_angular), -self.max_angular)

            # Choose base linear speed
            base_speed = self.slow_speed if self.traffic_state == 'yellow' else self.base_speed

            # Reduce linear speed based on turn intensity
            lin_speed = base_speed - abs(omega)
            cmd.linear.x = lin_speed if lin_speed >= 0.05 else 0.05
            cmd.angular.z = omega

        # Debug logging at DEBUG level
        self.get_logger().debug(
            f'e={self.line_error:.3f}  '
            f'i={self.error_integral:.3f}  '
            f'd={self.filtered_derivative:.3f}  '
            f'lin={cmd.linear.x:.3f}  '
            f'ang={cmd.angular.z:.3f}'
        )

        # Publish command and update state
        self.cmd_vel_pub.publish(cmd)
        self.prev_error = self.line_error
        self.prev_time = now


def main(args=None):
    """
    Main entry: initialize ROS2 and spin node.
    """
    rclpy.init(args=args)
    node = LineTrafficController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  
    main()

