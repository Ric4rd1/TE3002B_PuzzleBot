import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveForwardClass(Node): 
    def __init__(self):
        super().__init__('move_square')
        # Parámetros para la trayectoria
        self.side_length = 0.5          # longitud de cada lado en metros
        self.linear_speed = 0.2         # velocidad lineal (m/s)
        self.angular_speed = 0.5        # velocidad angular (rad/s)
        self.turn_angle = 1.5708        # ángulo de giro, 90° en radianes

        # Factor de calibración experimental determinado previamente
        self.calibration_factor = 0.93

        # Tiempo calculado para avanzar y girar
        self.move_time = self.side_length / self.linear_speed   # tiempo para recorrer un lado
        self.turn_time = (self.turn_angle / self.angular_speed) * self.calibration_factor  # tiempo para girar 90° ajustado

        # Variables para la FSM
        self.state = 'move_linear'      # estado inicial: avanzar
        self.leg_count = 0              # contador de lados completados
        self.state_start_time = self.get_clock().now()  # marca de tiempo al iniciar el estado

        # Publicador para enviar comandos de movimiento
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Crear timer para actualizar la FSM cada 50 ms
        timer_period = 0.05  # 50 milisegundos
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("FSM para trayectoria cuadrada iniciada.")

    def timer_callback(self):
        current_time = self.get_clock().now()
        # Calcular el tiempo transcurrido en el estado actual (en segundos)
        elapsed = (current_time.nanoseconds - self.state_start_time.nanoseconds) / 1e9
        twist = Twist()

        if self.state == 'move_linear':
            # En el estado move_linear, avanzar a velocidad constante
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
            # Si se ha avanzado el tiempo necesario, cambiar al estado de giro
            if elapsed >= self.move_time:
                self.get_logger().info("Movimiento lineal completado. Iniciando giro.")
                self.state = 'turn'
                self.state_start_time = current_time  # reiniciar el cronómetro para el giro

        elif self.state == 'turn':
            # En el estado turn, aplicar giro a velocidad angular constante
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)
            
            # Si se ha completado el giro (tiempo suficiente para 90°)
            if elapsed >= self.turn_time:
                self.leg_count += 1  # se cuenta un lado completado
                self.get_logger().info(f"Giro completado. Lado completado: {self.leg_count}")
                # Si aún no se han completado los 4 lados del cuadrado
                if self.leg_count < 4:
                    self.state = 'move_linear'
                else:
                    self.get_logger().info("Trayectoria cuadrada completada.")
                    self.state = 'stop'
                self.state_start_time = current_time  # reiniciar el cronómetro para el siguiente estado

        elif self.state == 'stop':
            # En estado stop, se detiene el robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            # Una vez detenido, no se realizan más acciones.

def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardClass() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
