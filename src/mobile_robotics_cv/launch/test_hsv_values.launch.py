import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 
    # Get the address of the yaml file 
    config = os.path.join(
        get_package_share_directory('mobile_robotics_cv'),
        'config',
        'params2.yaml')

    stopLight_detector = Node( 
        package='mobile_robotics_cv', 
        executable='stopLight_detector', 
        output='screen', 
        emulate_tty=True, 
        parameters=[config] 
    ) 
    
    ld = LaunchDescription([stopLight_detector]) 

    return ld 