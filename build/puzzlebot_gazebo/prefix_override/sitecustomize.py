import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ricard/ros2_ws_puzzlebot/install/puzzlebot_gazebo'
