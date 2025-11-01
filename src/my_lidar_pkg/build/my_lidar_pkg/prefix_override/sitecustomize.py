import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jacob/ros2_ws/src/my_lidar_pkg/install/my_lidar_pkg'
