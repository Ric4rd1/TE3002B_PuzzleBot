import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        # Parámetros para movimiento y giro
        self.linear_speed = 0.2    # velocidad lineal en m/s
        self.move_distance = 0.5   # distancia fija para avanzar (metros)
        self.angular_speed = 0.5   # velocidad angular (rad/s)
        self.turn_angle = 1.5708   # ángulo de giro (90° en radianes)
        
        # Factor de calibración (valor experimental inicial)
        self.calibration_factor = 0.93
        # Tiempo para completar el giro calibrado
        self.turn_time = (self.turn_angle / self.angular_speed) * self.calibration_factor
        # Tiempo para recorrer la distancia hacia adelante
        self.move_time = self.move_distance / self.linear_speed

        # Estados posibles: "idle", "move", "turn"
        self.state = "idle"       
        self.state_start_time = self.get_clock().now()
        
        # Publicador para enviar comandos de velocidad
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Se crea un timer con período de 50 ms
        timer_period = 0.05  # 50 milisegundos
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(
            "Nodo de prueba iniciado.\n"
            "Comandos disponibles:\n"
            "  'f' - avanzar distancia fija\n"
            "  't' - girar 90°\n"
            "  'a' - aumentar ganancia de calibración (+0.01)\n"
            "  'd' - disminuir ganancia de calibración (-0.01)\n"
            "  'q' - salir\n"
        )

    def timer_callback(self):
        """Ejecuta la acción correspondiente según el estado actual."""
        current_time = self.get_clock().now()
        elapsed = (current_time.nanoseconds - self.state_start_time.nanoseconds) / 1e9
        twist = Twist()

        if self.state == "move":
            # Mientras no se complete el tiempo requerido para avanzar, se sigue moviendo hacia adelante.
            if elapsed < self.move_time:
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
            else:
                self.get_logger().info("Movimiento hacia adelante completado.")
                self.state = "idle"
                self.cmd_vel_pub.publish(Twist())  # detener
                self.state_start_time = current_time

        elif self.state == "turn":
            # Durante el giro, se publica una velocidad angular constante
            if elapsed < self.turn_time:
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
                self.cmd_vel_pub.publish(twist)
            else:
                self.get_logger().info("Giro completado.")
                self.state = "idle"
                self.cmd_vel_pub.publish(Twist())  # detener
                self.state_start_time = current_time

        elif self.state == "idle":
            # En estado idle, se asegura que el robot se encuentre detenido
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

    def start_move(self):
        """Inicia el movimiento hacia adelante si el robot se encuentra inactivo."""
        if self.state == "idle":
            self.state = "move"
            self.state_start_time = self.get_clock().now()
            self.get_logger().info("Iniciando movimiento hacia adelante.")
        else:
            self.get_logger().warn("Ya hay una acción en curso, espera a que termine.")

    def start_turn(self):
        """Inicia el giro de 90° si el robot se encuentra inactivo."""
        if self.state == "idle":
            self.state = "turn"
            self.state_start_time = self.get_clock().now()
            self.get_logger().info(
                f"Iniciando giro de 90° (turn_time: {self.turn_time:.3f}s, ganancia: {self.calibration_factor:.3f})."
            )
        else:
            self.get_logger().warn("Ya hay una acción en curso, espera a que termine.")

    def adjust_calibration(self, delta):
        """Ajusta la ganancia de calibración y recalcula el tiempo de giro."""
        self.calibration_factor += delta
        self.turn_time = (self.turn_angle / self.angular_speed) * self.calibration_factor
        self.get_logger().info(
            f"Ajuste de ganancia: {self.calibration_factor:.3f}. Nuevo tiempo de giro: {self.turn_time:.3f}s."
        )

def keyboard_listener(node):
    """
    Hilo que escucha la entrada del teclado para:
      - Iniciar movimiento ('f')
      - Iniciar giro ('t')
      - Aumentar ('a') o disminuir ('d') la ganancia de calibración
      - Salir ('q')
    """
    while rclpy.ok():
        command = input(
            "Ingresa comando ('f' mover, 't' girar, 'a' aumentar ganancia, 'd' disminuir ganancia, 'q' salir): "
        )
        if command.lower() == 'q':
            node.get_logger().info("Saliendo...")
            rclpy.shutdown()
            break
        elif command.lower() == 'f':
            node.start_move()
        elif command.lower() == 't':
            node.start_turn()
        elif command.lower() == 'a':
            node.adjust_calibration(0.01)
        elif command.lower() == 'd':
            node.adjust_calibration(-0.01)
        else:
            node.get_logger().info("Comando no reconocido.")

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    keyboard_thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    keyboard_thread.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
