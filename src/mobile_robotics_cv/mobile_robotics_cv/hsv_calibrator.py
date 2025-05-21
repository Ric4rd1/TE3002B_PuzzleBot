# hsv_calibrator.py (mejorado)
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

class HSVCalibrator(Node):
    def __init__(self):
        super().__init__('hsv_calibrator')
        self.bridge = CvBridge()
        
        # Suscribirse a la cámara
        self.subscription = self.create_subscription(Image, 'camera', self.image_callback, 10)
        
        # Publicador de imagen procesada
        self.processed_pub = self.create_publisher(Image, 'hsv_calibration', 10)
        
        self.image = None
        self.processed = False
        
        # Crear ventanas
        cv2.namedWindow("HSV Controls", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("HSV Controls", 600, 300)
        
        # Configuración para múltiples colores
        colors = ["Red", "Green", "Yellow"]
        self.current_color = "Red"
        
        # Crear combobox para seleccionar color
        cv2.createTrackbar("Color", "HSV Controls", 0, len(colors)-1, 
                          lambda x: self.set_current_color(colors[x]))
        
        # Crear trackbars para HSV
        cv2.createTrackbar("H Min", "HSV Controls", 0, 179, lambda x: None)
        cv2.createTrackbar("H Max", "HSV Controls", 10, 179, lambda x: None)
        cv2.createTrackbar("S Min", "HSV Controls", 100, 255, lambda x: None)
        cv2.createTrackbar("S Max", "HSV Controls", 255, 255, lambda x: None)
        cv2.createTrackbar("V Min", "HSV Controls", 100, 255, lambda x: None)
        cv2.createTrackbar("V Max", "HSV Controls", 255, 255, lambda x: None)
        
        # Valores para cada color
        self.hsv_values = {
            "Red": [0, 10, 100, 255, 100, 255],
            "Green": [40, 80, 100, 255, 100, 255],
            "Yellow": [20, 35, 100, 255, 100, 255]
        }
        
        # Establecer valores iniciales
        self.update_trackbars()
        
        self.timer = self.create_timer(0.05, self.update_display)
        self.get_logger().info("HSV Calibrator node started!")
    
    def set_current_color(self, color):
        """Cambiar el color actual y actualizar los trackbars"""
        self.current_color = color
        self.update_trackbars()
    
    def update_trackbars(self):
        """Actualizar los trackbars con los valores del color actual"""
        values = self.hsv_values[self.current_color]
        cv2.setTrackbarPos("H Min", "HSV Controls", values[0])
        cv2.setTrackbarPos("H Max", "HSV Controls", values[1])
        cv2.setTrackbarPos("S Min", "HSV Controls", values[2])
        cv2.setTrackbarPos("S Max", "HSV Controls", values[3])
        cv2.setTrackbarPos("V Min", "HSV Controls", values[4])
        cv2.setTrackbarPos("V Max", "HSV Controls", values[5])
    
    def save_current_values(self):
        """Guardar los valores actuales de los trackbars"""
        self.hsv_values[self.current_color] = [
            cv2.getTrackbarPos("H Min", "HSV Controls"),
            cv2.getTrackbarPos("H Max", "HSV Controls"),
            cv2.getTrackbarPos("S Min", "HSV Controls"),
            cv2.getTrackbarPos("S Max", "HSV Controls"),
            cv2.getTrackbarPos("V Min", "HSV Controls"),
            cv2.getTrackbarPos("V Max", "HSV Controls")
        ]
        self.get_logger().info(f"Saved {self.current_color} HSV: {self.hsv_values[self.current_color]}")
    
    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.processed = False
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
    
    def update_display(self):
        if self.image is None:
            return
        
        if not self.processed:
            # Obtener valores de los trackbars
            h_min = cv2.getTrackbarPos("H Min", "HSV Controls")
            h_max = cv2.getTrackbarPos("H Max", "HSV Controls")
            s_min = cv2.getTrackbarPos("S Min", "HSV Controls")
            s_max = cv2.getTrackbarPos("S Max", "HSV Controls")
            v_min = cv2.getTrackbarPos("V Min", "HSV Controls")
            v_max = cv2.getTrackbarPos("V Max", "HSV Controls")
            
            # Guardar valores actuales
            self.hsv_values[self.current_color] = [h_min, h_max, s_min, s_max, v_min, v_max]
            
            # Procesar imagen
            image_copy = self.image.copy()
            hsv = cv2.cvtColor(image_copy, cv2.COLOR_BGR2HSV)
            lower = np.array([h_min, s_min, v_min])
            upper = np.array([h_max, s_max, v_max])
            mask = cv2.inRange(hsv, lower, upper)
            
            # Operaciones morfológicas
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=2)
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            # Aplicar máscara
            result = cv2.bitwise_and(image_copy, image_copy, mask=mask)
            
            # Encontrar contornos
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Dibujar contornos grandes
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 100:  # Filtrar ruido
                    # Dibujar círculo
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(result, center, radius, (0, 255, 255), 2)
                    
                    # Estimar "distancia" basada en tamaño
                    known_radius = 20  # Radio a 1m (calibrar)
                    distance = known_radius / radius if radius > 0 else float('inf')
                    
                    # Mostrar distancia estimada
                    cv2.putText(result, f"{distance:.2f}m", (center[0] - 20, center[1] - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Agregar información de color actual
            cv2.putText(result, f"Color: {self.current_color}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Mostrar imágenes
            cv2.imshow("Original", image_copy)
            cv2.imshow("Mask", mask)
            cv2.imshow("Result", result)
            
            # Publicar imagen procesada
            self.processed_pub.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))
            
            # Marcar como procesado
            self.processed = True
            
            # Imprimir valores actuales del color al presionar 's'
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                self.save_current_values()
                self.get_logger().info("Current HSV values:")
                for color, values in self.hsv_values.items():
                    self.get_logger().info(f"{color}: {values}")
            elif key == ord('q'):
                self.get_logger().info("Shutting down...")
                self.destroy_node()
                rclpy.shutdown()

    
def main(args=None):
    rclpy.init(args=args)
    calibrator_node = HSVCalibrator()
    try:
        rclpy.spin(calibrator_node)
    except KeyboardInterrupt:
        pass
    finally:
        calibrator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
