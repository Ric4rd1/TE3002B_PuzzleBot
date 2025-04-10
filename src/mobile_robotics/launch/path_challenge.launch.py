import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 

    # Get the address of the yaml file 
    config = os.path.join(
        get_package_share_directory('mobile_robotics'),
        'config',
        'params.yaml')

    path_generator = Node( 
        package='mobile_robotics', 
        executable='path_generator', 
        output='screen', 
        emulate_tty=True, 
        parameters=[config] 
    ) 

    controller_points = Node( 
        package='mobile_robotics', 
        executable='controller_points', 
        output='screen', 
        emulate_tty=True, 
        parameters=[ 
            {'use_sim_time': True}, 
        ] 
    ) 
    ld = LaunchDescription([path_generator, controller_points]) 

    return ld 