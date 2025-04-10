import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveSquare(Node):
    def __init__(self):
        super().__init__('move_square')
        # Parámetros para la trayectoria
        self.side_length = 0.5          # Longitud de cada lado en metros
        self.linear_speed = 0.2         # Velocidad lineal en m/s
        self.angular_speed = 0.5        # Velocidad angular en rad/s
        self.turn_angle = 1.5708        # Ángulo de giro (90° en radianes)

        # Factor de calibración experimental
        self.calibration_factor = 0.93

        # Cálculo de tiempos
        self.move_time = self.side_length / self.linear_speed
        self.turn_time = (self.turn_angle / self.angular_speed) * self.calibration_factor

        # Variables para la FSM (máquina de estados)
        # Estados posibles: "move" (avanzar), "turn" (girar) y "stop" (detenerse)
        self.state = 'move'
        self.leg_count = 0          # Contador de lados completados
        self.state_start_time = self.get_clock().now()

        # Publicador para enviar comandos al robot
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer para actualizar la FSM cada 50 ms
        timer_period = 0.05  # 50 milisegundos
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Trayectoria cuadrada iniciada de forma automática.")

    def timer_callback(self):
        current_time = self.get_clock().now()
        elapsed = (current_time.nanoseconds - self.state_start_time.nanoseconds) / 1e9
        twist = Twist()

        if self.state == 'move':
            # Moverse en línea recta
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            if elapsed >= self.move_time:
                self.get_logger().info("Movimiento lineal completado. Iniciando giro.")
                self.state = 'turn'
                self.state_start_time = current_time

        elif self.state == 'turn':
            # Realizar giro
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)
            if elapsed >= self.turn_time:
                self.leg_count += 1
                self.get_logger().info(f"Giro completado. Lado completado: {self.leg_count}")
                if self.leg_count < 4:
                    self.state = 'move'
                else:
                    self.get_logger().info("Trayectoria cuadrada completada.")
                    self.state = 'stop'
                self.state_start_time = current_time

        elif self.state == 'stop':
            # Detener el robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            # Una vez en "stop" ya no se realizan más acciones.

def main(args=None):
    rclpy.init(args=args)
    node = MoveSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
