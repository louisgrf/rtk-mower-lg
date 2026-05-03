import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mower/rtk-mower-lg/ros2_ws/install/mower_bringup'
