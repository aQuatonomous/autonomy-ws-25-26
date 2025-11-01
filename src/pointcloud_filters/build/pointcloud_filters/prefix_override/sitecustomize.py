import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jacob/ros2_ws/src/pointcloud_filters/install/pointcloud_filters'
