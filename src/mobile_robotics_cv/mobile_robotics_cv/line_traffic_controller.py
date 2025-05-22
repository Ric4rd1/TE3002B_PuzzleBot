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
    def __init__(self):
        super().__init__('line_traffic_controller')

        # Parámetros de velocidad y control
        self.declare_parameter('base_speed', 0.13)     # velocidad en línea recta (m/s)
        self.declare_parameter('slow_speed', 0.07)     # velocidad en amarillo (m/s)
        self.declare_parameter('k_p', 0.001)           # ganancia proporcional
        self.declare_parameter('k_i', 0.00001)         # ganancia integral (muy pequeña)
        self.declare_parameter('k_d', 0.00005)         # ganancia derivativa (pequeña)
        self.declare_parameter('max_angular', 0.5)     # límite de giro (rad/s)
        self.declare_parameter('filter_alpha', 0.7)    # coeficiente LPF para derivada

        # Publicador de cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Suscripciones
        self.create_subscription(Float32, 'line_error', self._on_line_error, 10)
        self.create_subscription(String, Float32, self._on_line_error, 10)
        self.create_subscription(String, 'sphere_color', self._on_sphere_color, 10)

        # Carga de parámetros
        params = self.get_parameters([
            'base_speed', 'slow_speed',
            'k_p', 'k_i', 'k_d',
            'max_angular', 'filter_alpha'
        ])
        self.base_speed = params[0].value
        self.slow_speed = params[1].value
        self.k_p = params[2].value
        self.k_i = params[3].value
        self.k_d = params[4].value
        self.max_angular = params[5].value
        self.alpha = params[6].value

        # Estados internos
        self.line_error = 0.0
        self.prev_error = 0.0
        self.error_integral = 0.0
        self.filtered_derivative = 0.0
        self.traffic_state = 'green'
        self.prev_time = self.get_clock().now()

        # Temporizador a 20 Hz
        self.create_timer(0.05, self._on_timer)

        self.get_logger().info('LineTrafficController listo 🚀')

    def _on_line_error(self, msg: Float32):
        """Actualiza el error de línea."""
        self.line_error = msg.data

    def _on_sphere_color(self, msg: String):
        """Gestiona el semáforo según color de esfera."""
        c = msg.data.lower()
        if c in ('red', 'yellow', 'green'):
            self.traffic_state = c
            self.get_logger().info(f'{c.capitalize()} detected')

    def _on_timer(self):
        """Publica cmd_vel combinando PID y estado de semáforo."""
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9 or 0.05

        # Derivada cruda
        raw_der = (self.line_error - self.prev_error) / dt

        # Filtro pasa-bajos para derivada
        self.filtered_derivative = (
            self.alpha * self.filtered_derivative +
            (1 - self.alpha) * raw_der
        )

        # Integral con anti-windup
        self.error_integral += self.line_error * dt
        self.error_integral = max(min(self.error_integral, 100.0), -100.0)

        cmd = Twist()

        if self.traffic_state == 'red':
            # Frenar completamente
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # Control PID: ω = -(k_p·e + k_i·∫e dt + k_d·d e/dt)
            omega = -(
                self.k_p * self.line_error +
                self.k_i * self.error_integral +
                self.k_d * self.filtered_derivative
            )

            # Saturación de velocidad angular
            omega = max(min(omega, self.max_angular), -self.max_angular)

            # Selección de velocidad lineal base
            base = self.slow_speed if self.traffic_state == 'yellow' else self.base_speed

            # Reducir velocidad lineal en giros y mantener mínimo
            lin = base - abs(omega)
            cmd.linear.x = lin if lin >= 0.05 else 0.05
            cmd.angular.z = omega

        # Log para depuración (nivel DEBUG)
        self.get_logger().debug(
            f'e={self.line_error:.3f}  '
            f'i={self.error_integral:.3f}  '
            f'd={self.filtered_derivative:.3f}  '
            f'lin={cmd.linear.x:.3f}  '
            f'ang={cmd.angular.z:.3f}'
        )

        # Publicar y actualizar estados
        self.cmd_vel_pub.publish(cmd)
        self.prev_error = self.line_error
        self.prev_time = now


def main(args=None):
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
