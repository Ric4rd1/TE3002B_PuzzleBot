import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    odometry = Node(
        package='mobile_robotics_cv',
        executable='odom',
        output='screen',
        emulate_tty=True
    )

    move = Node(
        package='mobile_robotics_cv',
        executable='move_test3',
        output='screen',
        emulate_tty=True
    )

    # Delay the start of 'move' node by 2 seconds
    delayed_turn = TimerAction(
        period=1.0,
        actions=[move]
    )

    return LaunchDescription([
        odometry,
        delayed_turn
    ])
