import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import TimerAction

def generate_launch_description(): 
    # Get the address of the yaml file 
    config = os.path.join(
        get_package_share_directory('mobile_robotics_autonomous'),
        'config',
        'params.yaml')


    move_points_traffic = Node( 
        package='mobile_robotics_autonomous', 
        executable='move_points_traffic', 
        output='screen', 
        emulate_tty=True, 
        parameters=[config] 
    ) 

    odom = Node( 
        package='mobile_robotics_autonomous', 
        executable='odom', 
        output='screen', 
        emulate_tty=True, 
        parameters=[config] 
    ) 
    '''
    streetSign_YOLOv8 = Node( 
        package='mobile_robotics_autonomous', 
        executable='streetSign_YOLOv8', 
        output='log',  # <-- Don't print anything to screen
        parameters=[config] 
    ) 

    stopLight_YOLOv8 = Node( 
        package='mobile_robotics_autonomous', 
        executable='stopLight_YOLOv8', 
        output='log',  # <-- Don't print anything to screen
        parameters=[config] 
    ) 
    streetSign_YOLOv8, stopLight_YOLOv8
    '''

    # Delay the start of 'move_line_traffic' node by 1 seconds
    delayed_move_points_traffic = TimerAction(
        period=1.0,
        actions=[move_points_traffic]
    )

    
    ld = LaunchDescription([odom, delayed_move_points_traffic,]) 

    return ld 