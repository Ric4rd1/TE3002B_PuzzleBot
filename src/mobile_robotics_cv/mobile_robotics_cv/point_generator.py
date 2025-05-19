import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String

class PointGenerator(Node):
    def __init__(self):
        super().__init__('point_generator')
        
        # Parameters
        self.declare_parameter('point_1', [0.0, 0.0])
        self.declare_parameter('point_2', [0.0, 0.0])
        self.declare_parameter('point_3', [0.0, 0.0])
        self.declare_parameter('point_4', [0.0, 0.0])

        # Get parameters
        self.point_1 = self.get_parameter('point_1').value
        self.point_2 = self.get_parameter('point_2').value
        self.point_3 = self.get_parameter('point_3').value
        self.point_4 = self.get_parameter('point_4').value
        self.points = [self.point_1, self.point_2, self.point_3, self.point_4]
        self.get_logger().info(f'Points: {self.points}')

        # Publisher
        self.pose_pub = self.create_publisher(Pose2D, 'point', 10)

        # Subscriber
        self.confirmation_sub = self.create_subscription(String, 'confirmation', self.confirmation_callback, 10)

        self.pose = Pose2D()
        self.curr_point_index = 0


        self.get_logger().info('Path generator initialized!')


    def confirmation_callback(self, msg):
        if msg.data == 'ok':
            self.get_logger().info('Received confirmation from controller.')
            # Check if all points have been published
            if self.curr_point_index >= len(self.points):
                self.get_logger().info('All points have been published.')
                return
            # Publish the next point
            self.pose.x = self.points[self.curr_point_index][0]
            self.pose.y = self.points[self.curr_point_index][1]
            self.pose_pub.publish(self.pose)
            self.get_logger().info(f'Publishing point {self.curr_point_index + 1}: {self.points[self.curr_point_index]}')
            self.curr_point_index += 1

def main(args=None):
    rclpy.init(args=args)
    point_generator = PointGenerator()

    try:
        rclpy.spin(point_generator)
    except KeyboardInterrupt:
        pass
    finally:
        point_generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()