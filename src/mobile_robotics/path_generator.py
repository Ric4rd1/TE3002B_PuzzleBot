import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class State:
    MOVE_FORWARD = 0
    TURN = 1
    STOP = 2

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.calibration_factor = 0.93
        
        self.waypoints = [
            (2.0, 0.0),
            (0.0, 0.5),
            (2.0, 0.5),
            (0.0, 0.0)
        ]
        
        self.movement_times = []
        for i in range(len(self.waypoints)):
            if i == 0:
                distance = math.sqrt(self.waypoints[i][0]**2 + self.waypoints[i][1]**2)
            else:
                dx = self.waypoints[i][0] - self.waypoints[i-1][0]
                dy = self.waypoints[i][1] - self.waypoints[i-1][1]
                distance = math.sqrt(dx**2 + dy**2)
            self.movement_times.append(distance / self.linear_speed)
        
        self.turn_angles = []
        self.turn_times = []
        last_angle = math.atan2(self.waypoints[0][1], self.waypoints[0][0])
        for i in range(1, len(self.waypoints)):
            dx = self.waypoints[i][0] - self.waypoints[i-1][0]
            dy = self.waypoints[i][1] - self.waypoints[i-1][1]
            new_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(new_angle - last_angle)
            self.turn_angles.append(angle_diff)
            self.turn_times.append((abs(angle_diff) / self.angular_speed) * self.calibration_factor)
            last_angle = new_angle
        
        self.state = State.MOVE_FORWARD
        self.current_waypoint_idx = 0
        self.action_start_time = time.time()
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.control_timer = self.create_timer(0.05, self.control_callback)
        
        self.get_logger().info("Path Generator iniciado. Siguiendo trayectoria basada en coordenadas.")
        self.get_logger().info(f"Moviéndose hacia el punto 1: ({self.waypoints[0][0]}, {self.waypoints[0][1]})")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_callback(self):
        twist = Twist()
        current_time = time.time()
        elapsed_time = current_time - self.action_start_time
        
        if self.state == State.MOVE_FORWARD:
            if elapsed_time < self.movement_times[self.current_waypoint_idx]:
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
            else:
                self.get_logger().info(f"Llegó al punto {self.current_waypoint_idx + 1}: ({self.waypoints[self.current_waypoint_idx][0]}, {self.waypoints[self.current_waypoint_idx][1]})")
                if self.current_waypoint_idx < len(self.waypoints) - 1:
                    self.state = State.TURN
                    self.action_start_time = current_time
                    self.get_logger().info(f"Girando {math.degrees(self.turn_angles[self.current_waypoint_idx]):.1f} grados")
                else:
                    self.state = State.STOP
                    self.get_logger().info("Trayectoria completa. Deteniendo robot.")

        elif self.state == State.TURN:
            if elapsed_time < self.turn_times[self.current_waypoint_idx]:
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed * (1 if self.turn_angles[self.current_waypoint_idx] > 0 else -1)
            else:
                self.get_logger().info("Giro completado")
                self.current_waypoint_idx += 1
                self.state = State.MOVE_FORWARD
                self.action_start_time = current_time
                if self.current_waypoint_idx < len(self.waypoints):
                    self.get_logger().info(f"Moviéndose hacia el punto {self.current_waypoint_idx + 1}: ({self.waypoints[self.current_waypoint_idx][0]}, {self.waypoints[self.current_waypoint_idx][1]})")

        elif self.state == State.STOP:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if elapsed_time > 1.0:
                self.control_timer.cancel()

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción del usuario. Deteniendo el robot.")
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

