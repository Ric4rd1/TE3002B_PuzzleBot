import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 
    move_forward_node = Node( 
        package='mobile_robotics', 
        executable='move_forward', 
        output='screen', 
        emulate_tty=True, 
        parameters=[ 
            {'use_sim_time': True}, 
        ] 
    ) 
    ld = LaunchDescription([move_forward_node]) 

    return ld 