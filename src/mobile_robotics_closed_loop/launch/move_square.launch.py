import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 
    # Get the address of the yaml file 
    '''
    config = os.path.join(
        get_package_share_directory('mobile_robotics'),
        'config',
        'params.yaml')
    '''
    
    localization = Node( 
        package='mobile_robotics_closed_loop', 
        executable='localization', 
        output='screen', 
        emulate_tty=True, 
        parameters=[ 
            {'use_sim_time': False}, 
        ] 
    ) 
    move_square = Node( 
        package='mobile_robotics_closed_loop', 
        executable='move_square', 
        output='screen', 
        emulate_tty=True, 
        parameters=[ 
            {'use_sim_time': False}, 
        ] 
    ) 
    ld = LaunchDescription([localization, move_square]) 

    return ld 