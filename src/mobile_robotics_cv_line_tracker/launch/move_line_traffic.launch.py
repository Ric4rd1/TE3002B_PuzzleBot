import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import TimerAction

def generate_launch_description(): 
    # Get the address of the yaml file 
    config = os.path.join(
        get_package_share_directory('mobile_robotics_cv_line_tracker'),
        'config',
        'params.yaml')


    move_line_traffic = Node( 
        package='mobile_robotics_cv_line_tracker', 
        executable='move_line_traffic', 
        output='screen', 
        emulate_tty=True, 
        parameters=[config] 
    ) 

    stopLight_detector = Node( 
        package='mobile_robotics_cv_line_tracker', 
        executable='stopLight_detector', 
        output='screen', 
        emulate_tty=True, 
        parameters=[config] 
    ) 

    # Delay the start of 'move_line_traffic' node by 1 seconds
    delayed_move_line_traffic = TimerAction(
        period=1.0,
        actions=[move_line_traffic]
    )

    
    ld = LaunchDescription([stopLight_detector, delayed_move_line_traffic]) 

    return ld 