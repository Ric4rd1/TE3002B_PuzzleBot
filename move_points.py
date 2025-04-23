#!/usr/bin/env python3
"""
Nodo ROS2 optimizado para navegar a puntos usando odometría de Pose2D.
Estrategia de combate estilo Fortnite y Mozos de Halo: girar en su lugar,
mover en línea recta con corrección mínima y desaceleración suave.
Objetivo: derrotar al Covenant con movimientos impecables.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose2D
import math

class MovePointsOptimized(Node):
    def __init__(self):
        super().__init__('move_points_optimized')
        # Parámetros de control
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('dist_tolerance', 0.1)
        self.declare_parameter('angle_tolerance', 0.05)
        self.declare_parameter('slow_down_dist', 0.5)

        self.lin_max   = self.get_parameter('linear_speed').value
        self.ang_max   = self.get_parameter('angular_speed').value
        self.dist_tol  = self.get_parameter('dist_tolerance').value
        self.ang_tol   = self.get_parameter('angle_tolerance').value
        self.slow_dist = self.get_parameter('slow_down_dist').value

        # Ganancias P
        self.kp_lin    = 1.2
        self.kp_ang    = 4.0

        # Estado máquina: STOP -> TURN -> MOVE -> WAIT
        self.state     = 'STOP'
        self.point_received = False

        # Poses
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.x_target = 0.0; self.y_target = 0.0

        # Subscripciones y publicaciones
        self.create_subscription(Pose2D, 'pose2D', self.pose_callback, qos_profile_sensor_data)
        self.create_subscription(Pose2D, 'point', self.point_callback, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.conf_pub = self.create_publisher(String, 'confirmation', 10)

        # Timer de control
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('[Optimized] MovePoints inicializado')

    def wrap(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def pose_callback(self, msg: Pose2D):
        self.x = msg.x; self.y = msg.y; self.yaw = msg.theta

    def point_callback(self, msg: Pose2D):
        # Recibimos nuevo objetivo
        self.x_target = msg.x; self.y_target = msg.y
        self.state = 'TURN'
        self.point_received = True
        self.get_logger().info(f'Objetivo recibido: ({self.x_target:.2f}, {self.y_target:.2f})')

    def control_loop(self):
        cmd = Twist()
        if self.state == 'STOP':
            # En espera de punto
            pass

        elif self.state == 'TURN' and self.point_received:
            # Calcular ángulo deseado
            desired_yaw = math.atan2(self.y_target - self.y, self.x_target - self.x)
            error = self.wrap(desired_yaw - self.yaw)
            # Girar en sitio
            if abs(error) > self.ang_tol:
                cmd.angular.z = max(-self.ang_max, min(self.ang_max, self.kp_ang * error))
            else:
                # Listo para avanzar
                cmd = Twist()  # parar giro
                self.state = 'MOVE'
                self.get_logger().info('Giro completado, avanzando')

        elif self.state == 'MOVE':
            # Calcular distancia y corrección de rumbo
            dx = self.x_target - self.x; dy = self.y_target - self.y
            dist = math.hypot(dx, dy)
            desired_yaw = math.atan2(dy, dx)
            error = self.wrap(desired_yaw - self.yaw)
            # Desaceleración suave
            if dist < self.slow_dist:
                lin = self.lin_max * (dist / self.slow_dist)
                lin = max(lin, 0.05)
            else:
                lin = self.lin_max
            # Estabilizar rumbo con pequeña corrección angular
            ang = self.kp_ang * error
            # Limitar
            cmd.linear.x  = max(0.0, min(lin, self.lin_max))
            cmd.angular.z = max(-self.ang_max, min(self.ang_max, ang))
            # Verificar llegada
            if dist < self.dist_tol:
                cmd = Twist()
                self.state = 'WAIT'
                self.get_logger().info(f'Llegó al punto en {dist:.2f} m')

        elif self.state == 'WAIT':
            # Enviar confirmación y volver a STOP
            self.conf_pub.publish(String(data='ok'))
            self.get_logger().info('Confirmación enviada')
            self.point_received = False
            self.state = 'STOP'

        # Publicar comando
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MovePointsOptimized()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
