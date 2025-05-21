import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from rclpy.qos import qos_profile_sensor_data

class SphereDetector(Node):
    def __init__(self):
        super().__init__('sphere_detector')
        
        self.bridge = CvBridge()
        
        # Parámetros mejorados basados en el código de ejemplo
        self.declare_parameter('min_area', 100)
        self.declare_parameter('max_distance', 1.0)
        
        # Rangos HSV para cada color (basados en el ejemplo)
        # Mejorados para detectar esferas con más precisión
        self.declare_parameter('red_hsv_low1', [0, 70, 50])
        self.declare_parameter('red_hsv_high1', [10, 255, 255])
        self.declare_parameter('red_hsv_low2', [170, 70, 50])  # 2do rango para rojo (envuelve hue)
        self.declare_parameter('red_hsv_high2', [180, 255, 255])
        
        self.declare_parameter('green_hsv_low', [45, 90, 90])
        self.declare_parameter('green_hsv_high', [75, 255, 255])
        
        self.declare_parameter('yellow_hsv_low', [20, 100, 100])
        self.declare_parameter('yellow_hsv_high', [35, 255, 255])
        
        # Suscripción a la cámara
        self.cam_sub = self.create_subscription(Image, 'camera', self.camera_callback, 10)
        
        # Publicadores
        self.img_pub = self.create_publisher(Image, 'sphere_detection', qos_profile=qos_profile_sensor_data)
        self.sphere_pub = self.create_publisher(String, 'sphere_color', 10)
        self.distance_pub = self.create_publisher(Float32, 'sphere_distance', 10)
        
        # Obtener parámetros
        self.min_area = self.get_parameter('min_area').value
        self.max_distance = self.get_parameter('max_distance').value
        
        # Límites HSV
        self.red_low1 = np.array(self.get_parameter('red_hsv_low1').value)
        self.red_high1 = np.array(self.get_parameter('red_hsv_high1').value)
        self.red_low2 = np.array(self.get_parameter('red_hsv_low2').value)
        self.red_high2 = np.array(self.get_parameter('red_hsv_high2').value)
        
        self.green_low = np.array(self.get_parameter('green_hsv_low').value)
        self.green_high = np.array(self.get_parameter('green_hsv_high').value)
        
        self.yellow_low = np.array(self.get_parameter('yellow_hsv_low').value)
        self.yellow_high = np.array(self.get_parameter('yellow_hsv_high').value)
        
        # Variables
        self.cv_img = None
        self.image_received = False
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Sphere detector initialized')
    
    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f'Failed to get image: {e}')
    
    def estimate_distance(self, contour):
        """Estima la distancia a un objeto basado en su tamaño aparente"""
        # Obtener rectángulo que envuelve el contorno (como en el ejemplo)
        x, y, w, h = cv2.boundingRect(contour)
        
        # Estimar distancia basada en ancho del rectángulo
        # Valores de referencia: a 1m, una esfera tiene un ancho de aproximadamente 100px
        known_width_at_1m = 100  # (calibrar este valor)
        
        if w > 0:
            distance = known_width_at_1m / w
        else:
            distance = float('inf')
            
        return distance, (x, y, w, h)
    
    def timer_callback(self):
        if not self.image_received or self.cv_img is None:
            return
        
        try:
            # Crear copia para visualización
            output_img = self.cv_img.copy()
            
            # Convertir a HSV
            hsv_img = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2HSV)
            
            # Crear máscaras para cada color
            # Rojo (combinar dos rangos porque el rojo envuelve el espacio Hue)
            red_mask1 = cv2.inRange(hsv_img, self.red_low1, self.red_high1)
            red_mask2 = cv2.inRange(hsv_img, self.red_low2, self.red_high2)
            red_mask = cv2.add(red_mask1, red_mask2)
            
            # Verde
            green_mask = cv2.inRange(hsv_img, self.green_low, self.green_high)
            
            # Amarillo
            yellow_mask = cv2.inRange(hsv_img, self.yellow_low, self.yellow_high)
            
            # Operaciones morfológicas para reducir ruido (como en el ejemplo)
            kernel = np.ones((5, 5), np.uint8)
            for mask in [red_mask, green_mask, yellow_mask]:
                cv2.erode(mask, kernel, iterations=2, dst=mask)
                cv2.dilate(mask, kernel, iterations=4, dst=mask)
            
            # Información sobre máscaras
            masks = {
                'red': red_mask,
                'green': green_mask,
                'yellow': yellow_mask
            }
            
            detected_sphere = None
            min_sphere_distance = float('inf')
            
            # Procesar cada máscara
            for color_name, mask in masks.items():
                # Encontrar contornos
                contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                # Filtrar contornos pequeños (como en el ejemplo)
                valid_contours = []
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > self.min_area:
                        valid_contours.append(contour)
                
                if valid_contours:
                    # Encontrar el contorno más grande
                    largest = max(valid_contours, key=cv2.contourArea)
                    
                    # Estimar distancia y obtener rectángulo
                    distance, (x, y, w, h) = self.estimate_distance(largest)
                    
                    # Si está dentro del rango para detección
                    if distance <= self.max_distance:
                        # Dibujar rectángulo
                        color_bgr = (0, 0, 255) if color_name == 'red' else \
                                   (0, 255, 0) if color_name == 'green' else \
                                   (0, 255, 255)  # amarillo
                        
                        cv2.rectangle(output_img, (x, y), (x + w, y + h), color_bgr, 2)
                        cv2.putText(output_img, f"{color_name}: {distance:.2f}m", 
                                   (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)
                        
                        # Actualizar esfera más cercana
                        if distance < min_sphere_distance:
                            min_sphere_distance = distance
                            detected_sphere = color_name
            
            # Publicar color de esfera detectada más cercana
            if detected_sphere and min_sphere_distance <= self.max_distance:
                self.sphere_pub.publish(String(data=detected_sphere))
                self.distance_pub.publish(Float32(data=float(min_sphere_distance)))
                
                # Mostrar información en pantalla
                cv2.putText(output_img, f"Detected: {detected_sphere} ({min_sphere_distance:.2f}m)", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Publicar imagen procesada
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(output_img, "bgr8"))
        
        except Exception as e:
            self.get_logger().error(f"Error en timer_callback: {e}")
        
        self.image_received = False

def main(args=None):
    rclpy.init(args=args)
    detector = SphereDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()