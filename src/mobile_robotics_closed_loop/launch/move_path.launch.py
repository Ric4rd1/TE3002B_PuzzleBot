import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 
    # Get the address of the yaml file 
    config = os.path.join(
        get_package_share_directory('mobile_robotics_closed_loop'),
        'config',
        'params.yaml')

    
    localization = Node( 
        package='mobile_robotics_closed_loop', 
        executable='localization', 
        output='screen', 
        emulate_tty=True, 
        parameters=[config] 
    ) 
    move_points = Node( 
        package='mobile_robotics_closed_loop', 
        executable='move_points', 
        output='screen', 
        emulate_tty=True, 
        parameters=[config] 
    ) 
    point_generator = Node( 
        package='mobile_robotics_closed_loop', 
        executable='point_generator', 
        output='screen', 
        emulate_tty=True, 
        parameters=[config] 
    ) 
    ld = LaunchDescription([localization, move_points, point_generator]) 

    return ld 