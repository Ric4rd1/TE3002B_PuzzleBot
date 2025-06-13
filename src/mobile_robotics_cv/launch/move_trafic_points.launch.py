import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import TimerAction

def generate_launch_description(): 
    # Get the address of the yaml file 
    config = os.path.join(
        get_package_share_directory('mobile_robotics_cv'),
        'config',
        'params2.yaml')
    
    odom = Node( 
        package='mobile_robotics_cv', 
        executable='odom', 
        output='screen', 
        emulate_tty=True, 
        parameters=[config] 
    ) 

    move_points_traffic = Node( 
        package='mobile_robotics_cv', 
        executable='move_points_traffic', 
        output='screen', 
        emulate_tty=True, 
        parameters=[config] 
    ) 

    point_generator = Node( 
        package='mobile_robotics_cv', 
        executable='point_generator', 
        output='screen', 
        emulate_tty=True, 
        parameters=[config] 
    )
    stopLight_detector = Node( 
        package='mobile_robotics_cv', 
        executable='stopLight_detector', 
        output='screen', 
        emulate_tty=True, 
        parameters=[config] 
    ) 

    # Delay the start of 'point_generator' node by 1 seconds
    delayed_point_generator = TimerAction(
        period=1.0,
        actions=[point_generator]
    )

    # Delay the start of 'point_generator' node by 2 seconds
    delayed_move_points = TimerAction(
        period=2.0,
        actions=[move_points_traffic]
    )
    
    ld = LaunchDescription([odom, stopLight_detector, delayed_point_generator, delayed_move_points]) 

    return ld 