import rclpy
from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import Twist, Pose2D
import numpy as np
import math
import time

class SquareController(Node):
    def __init__(self):
        super().__init__('square_controller')
        
        # Parámetros del control
        self.target_distance = 2.0         # metros (tamaño del lado del cuadrado)
        self.linear_speed = 0.2            # m/s (velocidad lineal moderada)
        self.angular_speed = 0.3           # rad/s (velocidad angular moderada)
        self.tolerance_dist = 0.1          # margen de error en metros
        self.tolerance_angle = 0.1         # margen de error en radianes
        self.target_angle = np.pi/2        # 90 grados en radianes
        
        # Factores de calibración basados en las pruebas más recientes
        self.distance_calibration = 2.0 / 0.15  # Factor para corregir la distancia (≈ 13.33)
        self.angle_calibration = (np.pi/2) / 0.10  # Factor para corregir el ángulo (≈ 15.7)
        
        # Parámetros de control PID para distancia y ángulo
        self.Kv = 0.3  # Ganancia proporcional para velocidad lineal
        self.Kw = 0.5  # Ganancia proporcional para velocidad angular
        
        # Inicialización del estado
        self.state = "INIT"
        self.sides_completed = 0
        self.start_pose = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0
        self.start_angle = 0.0
        
        # Variables de tiempo para timeout y estabilización
        self.state_start_time = time.time()
        self.timeout_duration = 30.0       # Tiempo suficiente para movimientos
        self.stabilize_timer = None
        self.stabilizing = False
        
        # Variables para depuración
        self.initial_pose_received = False
        self.last_pose_time = None
        self.last_error_distance = 0.0  # Para cálculo de derivada
        self.last_error_angle = 0.0     # Para cálculo de derivada
        self.accumulated_error_distance = 0.0  # Para cálculo integral
        self.accumulated_error_angle = 0.0     # Para cálculo integral
        
        # Publicadores y suscriptores
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Suscribirse al tópico pose2D que publica el nodo de localización
        # Usar el mismo perfil QoS que el nodo de localización
        self.pose_sub = self.create_subscription(
            Pose2D, 
            'pose2D', 
            self.pose_callback, 
            qos.qos_profile_sensor_data
        )
        
        # Temporizador de control
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Controlador de trayectoria cuadrada inicializado")
        self.get_logger().info(f"Configuración: Cuadrado de {self.target_distance}x{self.target_distance} metros")
        self.get_logger().info(f"Factores de calibración: Distancia={self.distance_calibration:.2f}, Ángulo={self.angle_calibration:.2f}")
        
        # Esperar un poco antes de comenzar
        self.get_logger().info("Esperando datos de posición iniciales del nodo de localización...")
    
    def wrap_to_pi(self, angle):
        """Normaliza un ángulo al rango [-pi, pi]"""
        result = np.fmod((angle + np.pi), (2 * np.pi))
        if result < 0:
            result += 2 * np.pi
        return result - np.pi
    
    def send_stop_command(self):
        """Envía un comando de parada al robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info("Comando de PARADA enviado")
    
    def stabilize_position(self):
        """Pausa para estabilizar las lecturas de posición"""
        if not self.stabilizing:
            self.stabilizing = True
            self.send_stop_command()
            self.get_logger().info("Estabilizando lecturas de posición...")
            self.stabilize_timer = self.create_timer(1.0, self.end_stabilization)
    
    def end_stabilization(self):
        """Finaliza el período de estabilización"""
        self.stabilizing = False
        self.stabilize_timer.cancel()
        self.stabilize_timer = None
        self.get_logger().info("Estabilización completada, continuando...")
        
        # Continuar con la transición de estado que se estaba realizando
        if self.state == "STABILIZE_FORWARD":
            # Cambiar a estado de giro
            self.state = "TURN"
            self.start_angle = self.yaw
            self.state_start_time = time.time()
            self.accumulated_error_angle = 0.0  # Resetear integral para nuevo estado
            self.get_logger().info(f"Iniciando giro desde ángulo {self.start_angle:.3f}")
        elif self.state == "STABILIZE_TURN":
            # Giro completado
            self.sides_completed += 1
            self.get_logger().info(f"Giro completado. Ángulo final: {self.yaw:.3f}rad")
            
            # Guardar nueva posición de inicio para el siguiente lado
            self.start_pose = Pose2D()
            self.start_pose.x = self.current_x
            self.start_pose.y = self.current_y
            self.start_pose.theta = self.yaw
            
            self.state_start_time = time.time()
            self.accumulated_error_distance = 0.0  # Resetear integral para nuevo estado
            
            if self.sides_completed < 4:
                # Pasar al siguiente lado
                self.state = "MOVE_FORWARD"
                self.get_logger().info(f"Iniciando lado {self.sides_completed + 1}")
            else:
                # Cuadrado completado
                self.state = "STOP"
                self.get_logger().info("¡Cuadrado completado!")
    
    def pose_callback(self, msg):
        """Procesa los mensajes de pose2D del nodo de localización"""
        # Actualizar la posición actual
        self.current_x = msg.x
        self.current_y = msg.y
        self.yaw = msg.theta  # En el mensaje Pose2D, la orientación está en theta
        
        # Registrar que hemos recibido datos de posición
        if not self.initial_pose_received:
            self.initial_pose_received = True
            self.get_logger().info(f"¡Datos de posición iniciales recibidos! Posición: x={self.current_x:.3f}, y={self.current_y:.3f}, theta={self.yaw:.3f}")
        
        # Inicializar la posición de inicio si estamos en el estado INIT
        if self.state == "INIT" and self.initial_pose_received:
            self.start_pose = Pose2D()
            self.start_pose.x = self.current_x
            self.start_pose.y = self.current_y
            self.start_pose.theta = self.yaw
            
            self.get_logger().info(f"Posición inicial establecida: x={self.current_x:.3f}, y={self.current_y:.3f}, theta={self.yaw:.3f}")
            self.state = "MOVE_FORWARD"
            self.state_start_time = time.time()
            self.get_logger().info("Iniciando movimiento del lado 1")
    
    def check_timeout(self):
        """Verifica si el estado actual ha excedido el tiempo máximo permitido"""
        if self.stabilizing:
            return False
            
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time
        
        if elapsed_time > self.timeout_duration:
            self.get_logger().warn(f"¡TIMEOUT en estado {self.state} después de {elapsed_time:.1f} segundos!")
            
            # Decidir qué hacer según el estado
            if self.state == "MOVE_FORWARD":
                self.get_logger().info("Forzando cambio a estado TURN por timeout")
                self.state = "STABILIZE_FORWARD"
                self.stabilize_position()
            elif self.state == "TURN":
                self.get_logger().info("Forzando cambio a siguiente lado por timeout")
                self.state = "STABILIZE_TURN"
                self.stabilize_position()
            
            return True
        return False
    
    def apply_pid_control(self, error_distance, error_angle, dt):
        """Aplica control PID para velocidad lineal y angular"""
        # Términos proporcionales
        v_p = self.Kv * error_distance
        w_p = self.Kw * error_angle
        
        # Límites de velocidad
        v = min(max(v_p, -self.linear_speed), self.linear_speed)
        w = min(max(w_p, -self.angular_speed), self.angular_speed)
        
        return v, w
    
    def control_loop(self):
        """Bucle principal de control"""
        # Verificar que tenemos datos de posición
        if not self.initial_pose_received:
            return
            
        # Si estamos en modo de estabilización, no hacer nada
        if self.stabilizing:
            return
        
        # Verificar timeout del estado actual
        self.check_timeout()
        
        # Crear comando de velocidad
        cmd = Twist()
        current_time = time.time()
        dt = 0.1  # Tiempo aproximado entre iteraciones
        
        if self.state == "INIT":
            # Esperando datos iniciales de posición
            return
            
        elif self.state == "MOVE_FORWARD":
            # Calcular distancia recorrida
            dx = self.current_x - self.start_pose.x
            dy = self.current_y - self.start_pose.y
            distance = np.sqrt(dx**2 + dy**2)
            
            # Calcular la distancia ajustada con el factor de calibración
            calibrated_distance = distance * self.distance_calibration
            
            # Error de distancia (cuánto nos falta para llegar al objetivo)
            error_distance = self.target_distance - calibrated_distance
            
            self.get_logger().info(f"Avanzando: {distance:.3f}m (calibrado: {calibrated_distance:.3f}m) / {self.target_distance}m (lado {self.sides_completed + 1})")
            self.get_logger().info(f"Posición actual: x={self.current_x:.3f}, y={self.current_y:.3f}, θ={self.yaw:.3f}")
            
            if error_distance > self.tolerance_dist:
                # Seguir avanzando, aplicando PID
                cmd.linear.x, _ = self.apply_pid_control(error_distance, 0, dt)
            else:
                # Preparar para estabilizar y luego girar
                self.get_logger().info(f"Distancia objetivo alcanzada: {calibrated_distance:.3f}m")
                self.state = "STABILIZE_FORWARD"
                self.stabilize_position()
                return
                
        elif self.state == "TURN":
            # Calcular ángulo girado (normalizado)
            angle_turned = self.yaw - self.start_angle
            # Normalizar el ángulo al rango [-pi, pi]
            angle_turned = self.wrap_to_pi(angle_turned)
            
            # Calcular el ángulo ajustado con el factor de calibración
            calibrated_angle = angle_turned * self.angle_calibration
            
            # Error de ángulo (cuánto nos falta para llegar al ángulo objetivo)
            error_angle = self.target_angle - calibrated_angle
            
            self.get_logger().info(f"Girando: {angle_turned:.3f}rad (calibrado: {calibrated_angle:.3f}rad) / {self.target_angle:.3f}rad")
            self.get_logger().info(f"Ángulo actual: {self.yaw:.3f}, ángulo inicial: {self.start_angle:.3f}")
            
            if abs(error_angle) > self.tolerance_angle:
                # Seguir girando, aplicando PID
                _, cmd.angular.z = self.apply_pid_control(0, error_angle, dt)
            else:
                # Preparar para estabilizar y continuar
                self.get_logger().info(f"Ángulo objetivo alcanzado: {calibrated_angle:.3f}rad")
                self.state = "STABILIZE_TURN"
                self.stabilize_position()
                return
                    
        elif self.state == "STOP":
            # Mantener el robot detenido
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("Robot detenido. Trayectoria cuadrada completada.")
            
        # Publicar comando de velocidad
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = SquareController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Nodo terminado por usuario")
    except Exception as e:
        controller.get_logger().error(f"Error en el nodo: {str(e)}")
    finally:
        # Enviar comando de parada antes de terminar
        controller.send_stop_command()
        
        # Limpiar recursos
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()