# line_traffic_launch.py (actualizado)
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de odometría
        Node(
            package='mobile_robotics_cv',  # Cambiado de 'robot_control' a 'mobile_robotics_cv'
            executable='odom',
            name='odometry_node',
            output='screen'
        ),
        
        # Detector de línea
        Node(
            package='mobile_robotics_cv',  # Cambiado de 'robot_control' a 'mobile_robotics_cv'
            executable='line_follower',
            name='line_follower',
            output='screen',
            parameters=[
                {'roi_height': 0.3},
                {'binary_threshold': 60}
            ]
        ),
        
        # Detector de esferas
        Node(
            package='mobile_robotics_cv',  # Cambiado de 'robot_control' a 'mobile_robotics_cv'
            executable='sphere_detector',
            name='sphere_detector',
            output='screen',
            parameters=[
                {'min_area': 100},
                {'known_radius_at_1m': 20},
                {'max_distance': 1.0},
                {'red_hsv_low': [0, 100, 100]},
                {'red_hsv_high': [10, 255, 255]},
                {'green_hsv_low': [40, 100, 100]},
                {'green_hsv_high': [80, 255, 255]},
                {'yellow_hsv_low': [20, 100, 100]},
                {'yellow_hsv_high': [35, 255, 255]}
            ]
        ),
        
        # Controlador de línea y tráfico
        Node(
            package='mobile_robotics_cv',  # Cambiado de 'robot_control' a 'mobile_robotics_cv'
            executable='line_traffic_controller',
            name='line_traffic_controller',
            output='screen',
            parameters=[
                {'base_speed': 0.2},
                {'slow_speed': 0.1},
                {'k_angular': 0.01},
                {'max_angular': 0.8}
            ]
        )
    ])